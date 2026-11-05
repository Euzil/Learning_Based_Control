from collections import namedtuple
from typing import Generator, Tuple

import torch

from ics_rl.utils import split_and_pad_trajectories
from ics_rl.utils.buffer import buffer_from_example, buffer_method
from ics_rl.utils.collections import is_namedarraytuple


class AMPReplayBuffer:
    """Circular replay buffer that stores policy (s_t, s_{t+1}) pairs across rollouts.

    Unlike ``AMPStorage`` (which only holds the current rollout), this buffer persists
    between updates so the discriminator trains on a diverse history of policy behaviour.
    """

    def __init__(self, obs_size: int, buffer_size: int, device: str = "cpu"):
        self.obs_size = obs_size
        self.buffer_size = buffer_size
        self.device = device
        self.states = torch.zeros(buffer_size, obs_size, device=device)
        self.next_states = torch.zeros(buffer_size, obs_size, device=device)
        self._pos = 0
        self._full = False

    @property
    def size(self) -> int:
        return self.buffer_size if self._full else self._pos

    def insert(self, states: torch.Tensor, next_states: torch.Tensor) -> None:
        """Insert a batch of (s_t, s_{t+1}) pairs, wrapping around when full."""
        n = states.shape[0]
        end = self._pos + n
        if end <= self.buffer_size:
            self.states[self._pos:end].copy_(states.detach())
            self.next_states[self._pos:end].copy_(next_states.detach())
        else:
            first = self.buffer_size - self._pos
            self.states[self._pos:].copy_(states[:first].detach())
            self.next_states[self._pos:].copy_(next_states[:first].detach())
            rest = n - first
            self.states[:rest].copy_(states[first:].detach())
            self.next_states[:rest].copy_(next_states[first:].detach())
        self._pos = end % self.buffer_size
        if end >= self.buffer_size:
            self._full = True

    def sample(self, batch_size: int) -> Tuple[torch.Tensor, torch.Tensor]:
        """Return a random batch of (states, next_states)."""
        if self.size == 0:
            raise RuntimeError("AMPReplayBuffer is empty — no data to sample from.")
        idx = torch.randint(0, self.size, (batch_size,), device=self.device)
        return self.states[idx], self.next_states[idx]

    def mini_batch_generator(
        self, num_mini_batches: int, mini_batch_size: int
    ) -> Generator[Tuple[torch.Tensor, torch.Tensor], None, None]:
        """Yield `num_mini_batches` batches, covering the buffer as evenly as possible.

        `num_mini_batches * mini_batch_size` almost always exceeds the number of stored
        transitions, so this can't sample without replacement outright. Rather than draw each
        minibatch independently with replacement (which leaves some transitions oversampled and
        others skipped entirely within a single update — see torch.randint below), shuffle the
        whole buffer repeated `cycles` times and slice it, so every entry is reused an equal
        (±1) number of times across the generator's minibatches, matching rsl_rl's
        ReplayBuffer.feed_forward_generator.
        """
        if self.size == 0:
            raise RuntimeError("AMPReplayBuffer is empty — no data to sample from.")
        total = num_mini_batches * mini_batch_size
        if total > self.size:
            cycles = (total + self.size - 1) // self.size
            big_perm = torch.randperm(self.size * cycles, device=self.device)
            indices = big_perm[:total] % self.size
        else:
            indices = torch.randperm(self.size, device=self.device)[:total]
        for i in range(num_mini_batches):
            batch_idx = indices[i * mini_batch_size : (i + 1) * mini_batch_size]
            yield self.states[batch_idx], self.next_states[batch_idx]


class AMPStorage:
    class Transition:
        def __init__(self):
            self.actor_states = None
            self.hidden_states = None
            self.dones = None

        def clear(self):
            self.__init__()

    MiniBatch = namedtuple(
        "MiniBatch",
        [
            "actor_states",
            "hidden_states",
            "masks",
        ],
    )

    def __init__(
        self,
        num_envs,
        num_transitions_per_env,
        actor_state_shape,
        device="cpu",
    ):
        self.device = device

        self.actor_states_shape = actor_state_shape

        self.actor_states = torch.zeros(num_transitions_per_env, num_envs, *actor_state_shape, device=self.device)
        self.dones = torch.zeros(num_transitions_per_env, num_envs, 1, device=self.device).byte()

        self.num_transitions_per_env = num_transitions_per_env
        self.num_envs = num_envs

        # rnn
        self.saved_hidden_states = None

        self.step = 0

    def add_transitions(self, transition: Transition):
        if self.step >= self.num_transitions_per_env:
            raise AssertionError("Rollout buffer overflow")
        self.actor_states[self.step].copy_(transition.actor_states)
        self.dones[self.step].copy_(transition.dones.view(-1, 1))
        self._save_hidden_states(transition.hidden_states)
        self.step += 1

    def _save_hidden_states(self, hidden_states):
        """Assuming hidden_states is a torch tensor or a namedarraytuple of torch tensor"""
        if hidden_states is None:
            return
        if is_namedarraytuple(hidden_states):
            try:
                leading_dims = hidden_states.get_leading_dims()
            except AttributeError as e:
                if "None" in str(e):
                    return

        # initialize if needed
        if self.saved_hidden_states is None:
            self.saved_hidden_states = buffer_from_example(hidden_states, self.observations.shape[0])
        # copy the states
        self.saved_hidden_states[self.step] = hidden_states

    def clear(self):
        self.step = 0

    def mini_batch_generator(self, num_mini_batches, num_epochs=8):
        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches
        indices = torch.randperm(num_mini_batches * mini_batch_size, requires_grad=False, device=self.device)
        T_indices = (indices // self.num_envs).to(torch.long)
        B_indices = (indices % self.num_envs).to(torch.long)

        for epoch in range(num_epochs):
            for i in range(num_mini_batches):
                start = i * mini_batch_size
                end = (i + 1) * mini_batch_size
                T_idx = T_indices[start:end]
                B_idx = B_indices[start:end]

                yield self.get_minibatch_from_selection(T_idx, B_idx)

    # for RNNs only
    def recurrent_mini_batch_generator(self, num_mini_batches, num_epochs=8):
        self._padded_actor_trajectories, self._trajectory_masks = split_and_pad_trajectories(
            self.actor_states, self.dones
        )

        mini_batch_size = self.num_envs // num_mini_batches
        for ep in range(num_epochs):
            first_traj = 0
            for i in range(num_mini_batches):
                start = i * mini_batch_size
                stop = (i + 1) * mini_batch_size

                dones = self.dones.squeeze(-1)
                last_was_done = torch.zeros_like(dones, dtype=torch.bool)
                last_was_done[1:] = dones[:-1]
                last_was_done[0] = True
                trajectories_batch_size = torch.sum(last_was_done[:, start:stop])
                last_traj = first_traj + trajectories_batch_size

                yield self.get_minibatch_from_selection(
                    slice(None),
                    slice(start, stop),
                    padded_B_slice=slice(first_traj, last_traj),
                    prev_done_mask=last_was_done,
                )

                first_traj = last_traj

    def get_minibatch_from_selection(self, T_select, B_select, padded_B_slice=None, prev_done_mask=None):
        """Extract minibatch based on selected indices/slice."""
        if padded_B_slice is None:
            actor_batch = self.actor_states[T_select, B_select]
            hid_batch = None
            state_mask_batch = None
        else:
            actor_batch = self._padded_actor_trajectories[T_select, padded_B_slice]

            prev_done_mask = prev_done_mask.permute(1, 0)  # (T, B) -> (B, T)
            hid_batch = buffer_method(
                buffer_method(
                    buffer_method(self.saved_hidden_states, "permute", 2, 0, 1, 3)[prev_done_mask][padded_B_slice],
                    "transpose",
                    1,
                    0,
                ),
                "contiguous",
            )
            state_mask_batch = self._trajectory_masks[T_select, padded_B_slice]

        return AMPStorage.MiniBatch(
            actor_batch,
            hid_batch,
            state_mask_batch,
        )
