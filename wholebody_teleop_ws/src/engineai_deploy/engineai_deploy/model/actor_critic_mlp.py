from engineai_deploy.model.actor_critic_mlp_onnx import Policy as OnnxPolicy
from engineai_deploy.model.actor_critic_mlp_jit import Policy as JitPolicy

class Policy:
    def __init__(self, model_path: str, verbose: bool = False):
        self._backend = None
        
        if model_path.endswith(".onnx"):
            self._backend = OnnxPolicy(model_path, verbose=verbose)
        elif model_path.endswith((".pt", ".jit", ".pth")):
            self._backend = JitPolicy(model_path, verbose=verbose)
        else:
            raise ValueError(f"Unknown model format '{model_path.split('.')[-1]}'")
        
    # Delegate all calls to the underlying backend
    def forward(self, obs):
        return self._backend.forward(obs)

    @property
    def metadata(self):
        return self._backend.metadata

    @property
    def observation_dimension(self):
        return self._backend.observation_dimension

    # Add other properties as needed...