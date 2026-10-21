import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from engineai_deploy.agent.config_loader import ConfigLoader
from engineai_deploy.agent.pm01_agent import PM01Agent
from engineai_deploy.runners.policy_skill import PolicySkill


class MultiPolicyRunner:
    """Runs one of several policies and switches between them on the mode button.

    On a switch request the pending skill runs in parallel (shadowing) for
    switch_duration seconds to fill its observation history, while the
    robot-order joint targets and gains are blended from active to pending.
    """
    def __init__(
            self,
            node: Node,
            config_file: str,
            skill_dict: dict[str, str],
            default_skill: str,
            switch_duration: float = 1.0,
            warmup_duration: float = 0.0,
            dry_run: bool = False,
            verbose: bool = False,
        ):
        self._node = node

        # --- Agent
        config = ConfigLoader(config_file)
        self._agent = PM01Agent(node, config, is_dry_run=dry_run, verbose=verbose)
        if dry_run:
            print("[MultiPolicyRunner]: >>> DRY RUN <<<")
        else:
            print("[MultiPolicyRunner]: >>> REAL RUN <<<")

        self._agent.connect(timeout=Duration(seconds=60))
        if not self._agent.is_okay():
            raise Exception("[MultiPolicyRunner]: Agent not okay")

        self._agent.read(0.0)

        # --- Skills
        self._skills: dict[str, PolicySkill] = {}
        for name, path in skill_dict.items():
            self._node.get_logger().info(f"[MultiPolicyRunner]: Loading skill '{name}'")
            self._skills[name] = PolicySkill(name, path, self._agent, verbose)

        if default_skill not in self._skills:
            raise ValueError(f"[MultiPolicyRunner]: Default skill '{default_skill}' not found.")

        # single timer, so all skills must run at the same control rate
        self._dt_control = self._skills[default_skill].dt_control
        for skill in self._skills.values():
            if not np.isclose(skill.dt_control, self._dt_control):
                raise ValueError(
                    f"[MultiPolicyRunner]: Skill '{skill.name}' has dt_control={skill.dt_control} "
                    f"but default skill has dt_control={self._dt_control}"
                )

        self._skill_names = list(self._skills.keys())
        self._skill_index = self._skill_names.index(default_skill)
        self._active_skill: PolicySkill = self._skills[default_skill]
        self._pending_skill: PolicySkill | None = None
        self._active_skill.activate()

        # --- State
        self._switch_duration = switch_duration
        self._switch_start_time = 0.0
        self._prev_mode = self._agent.input.mode

        self._elapsed = 0.0
        self._start_time = None
        self._start_robot_pos = None
        self._warmup_duration = warmup_duration

    def start(self):
        self._active_skill.reset()
        self._start_time = self._node.get_clock().now()
        self._start_robot_pos = self._agent.sensor_state.dof_pos.copy()

        self._control_timer = self._node.create_timer(
            self._dt_control,
            self._control_callback,
        )
        self._node.get_logger().warn(
            f"[MultiPolicyRunner]: Control started in '{self._active_skill.name}' mode."
        )

    def request_switch(self, target_name: str):
        """Start shadowing the target skill before handing off."""
        if self._pending_skill is not None or target_name == self._active_skill.name:
            return

        target = self._skills[target_name]
        target.reset()

        # not running yet: swap directly, nothing to blend
        if not self._agent.is_started:
            self._active_skill = target
            self._active_skill.activate()
            self._node.get_logger().warn(f"[MultiPolicyRunner]: Active skill: '{target_name}'")
            return

        self._node.get_logger().warn(
            f"[MultiPolicyRunner]: Switching '{self._active_skill.name}' -> '{target_name}'"
        )
        self._pending_skill = target
        self._switch_start_time = self._elapsed

    def _complete_switch(self):
        self._pending_skill.activate()
        self._active_skill = self._pending_skill
        self._pending_skill = None
        self._node.get_logger().warn(
            f"[MultiPolicyRunner]: Handoff complete. Active skill: '{self._active_skill.name}'"
        )

    def _handle_mode_button(self):
        """Cycle to the next skill on the rising edge of the mode button."""
        mode = self._agent.input.mode
        if mode and not self._prev_mode and self._pending_skill is None:
            self._skill_index = (self._skill_index + 1) % len(self._skill_names)
            self.request_switch(self._skill_names[self._skill_index])
        self._prev_mode = mode

    def _control_callback(self):
        if not self._agent.is_okay():
            self._control_timer.cancel()
            self._node.get_logger().error("[MultiPolicyRunner]: Agent error detected — shutting down.")
            rclpy.shutdown()
            return

        self._agent.read(self._dt_control)
        self._handle_mode_button()

        if not self._agent.is_started:
            self._active_skill.reset()
            self._start_time = self._node.get_clock().now()
            self._start_robot_pos = self._agent.sensor_state.dof_pos.copy()
            return

        self._elapsed = (self._node.get_clock().now() - self._start_time).nanoseconds / 1e9

        t_start = time.perf_counter()

        robot_pos, robot_vel = self._active_skill.step(self._agent.input)

        # shadow the pending skill and blend robot-order targets and gains
        if self._pending_skill is not None:
            pending_pos, pending_vel = self._pending_skill.step(self._agent.input)

            alpha = 1.0
            if self._switch_duration > 0.0:
                alpha = min(1.0, (self._elapsed - self._switch_start_time) / self._switch_duration)

            robot_pos = (1.0 - alpha) * robot_pos + alpha * pending_pos
            robot_vel = (1.0 - alpha) * robot_vel + alpha * pending_vel
            self._agent.set_gains(
                (1.0 - alpha) * self._active_skill.robot_stiffness + alpha * self._pending_skill.robot_stiffness,
                (1.0 - alpha) * self._active_skill.robot_damping + alpha * self._pending_skill.robot_damping,
            )

            if alpha >= 1.0:
                self._complete_switch()

        inference_dur = time.perf_counter() - t_start
        if inference_dur > self._dt_control:
            self._node.get_logger().warn(f"[MultiPolicyRunner]: High Latency: {inference_dur*1000:.1f}ms")

        # startup interpolation from the initial robot pose
        if self._elapsed < self._warmup_duration:
            alpha = self._elapsed / self._warmup_duration
            robot_pos = (1.0 - alpha) * self._start_robot_pos + alpha * robot_pos
            self._active_skill.zero_last_action()

        self._agent.write(robot_pos, robot_vel)
