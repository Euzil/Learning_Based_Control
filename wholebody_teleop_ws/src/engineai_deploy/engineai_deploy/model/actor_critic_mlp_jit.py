import logging
import json
import ast
import os
from pathlib import Path
from typing import Optional, Dict, Any, Union

import numpy as np
import torch

def convert_meta_data(s):
    """ 
    Parses metadata into appropriate Python/NumPy types.
    """
    if isinstance(s, (list, tuple)):
        return np.array(s)
    if not isinstance(s, str):
        return s
    
    s = s.strip('"\'')
    if ((s.startswith('[') and s.endswith(']')) or (s.startswith('(') and s.endswith(')'))):
        try:
            evaluated = ast.literal_eval(s)
            if isinstance(evaluated, (list, tuple)):
                return np.array(evaluated)
        except (ValueError, SyntaxError):
            pass
    return s

class Policy:
    """ Wrapper class for loading and executing TorchScript (JIT) RL policies with metadata.
    """
    def __init__(self, file_path: Union[str, Path], verbose: bool = False):
        self._model: Optional[torch.jit.ScriptModule] = None
        self._metadata: Dict[str, Any] = {}
        self._input_dim: int = -1
        self._output_dim: int = -1
        self._device: torch.device = torch.device("cpu")
        
        self._load_model(file_path, verbose)

    def _load_model(self, file_path: Union[str, Path], verbose: bool = False):
        """
        Loads a TorchScript model and its embedded metadata via _extra_files.
        """
        path = Path(file_path)
        if not path.exists():
            raise ValueError(f"[Policy] File not found: {path}")
        
        # Select device
        if torch.cuda.is_available():
            self._device = torch.device("cuda")
            print("[Policy] Using CUDA for execution")
        else:
            self._device = torch.device("cpu")
            print("[Policy] Using CPU for execution")
    
        try:
            # Prepare metadata extraction
            extra_files = {'metadata.json': ''}
            
            # Load JIT Model
            self._model = torch.jit.load(str(path), map_location=self._device, _extra_files=extra_files)
            self._model.eval()
            
            # Parse metadata
            if extra_files['metadata.json']:
                raw_json = json.loads(extra_files['metadata.json'])
                self._metadata = {k: convert_meta_data(v) for k, v in raw_json.items()}
            else:
                print("[Policy] Warning: No metadata.json found in JIT file.")

            # Determine dimensions from metadata (matching original logic)
            if 'default_joint_pos' in self._metadata:
                self._output_dim = len(self._metadata['default_joint_pos'])
            
            if verbose:
                self._print_summary(path)

            # Warmup
            self._input_dim = int(self._metadata['observation_dim'])
            dummy_input = torch.zeros((1, self._input_dim), device=self._device)
            with torch.no_grad():
                for _ in range(10):
                    _ = self._model(dummy_input)

        except Exception as e:
            raise Exception(f"[Policy] Failed to load JIT model: {e}")

    def forward(self, observations: np.ndarray) -> np.ndarray:
        """ Runs inference. 
        Takes NumPy, returns NumPy to maintain compatibility.
        """
        if self._model is None:
            raise RuntimeError("[Policy] Model not loaded. Call load_model() first.")

        # Convert to Tensor and move to device
        obs_tensor = torch.from_numpy(observations).float().to(self._device)

        if obs_tensor.ndim == 1:
            obs_tensor = obs_tensor.unsqueeze(0)

        # Execute
        with torch.no_grad():
            action_tensor = self._model(obs_tensor)
        
        # Move back to CPU and convert to NumPy with original dtype
        return action_tensor.detach().cpu().numpy().astype(observations.dtype)

    def reset(self):
        """ Calls the reset method of the JIT model if it exists (for Recurrent policies) """
        if self._model is not None and hasattr(self._model, 'reset'):
            self._model.reset()

    def _print_summary(self, path: Path):
        print("="*40)
        print(f"[Policy] JIT Model Loaded:  {path.name}")
        print(f"[Policy] Device:            {self._device}")
        print(f"[Policy] Metadata Keys:     {list(self._metadata.keys())}")
        print("-" * 40)
        print("[Policy] Metadata Snippet:")
        for key, value in self._metadata.items():
            val_str = str(value)
            if isinstance(value, np.ndarray):
                val_str = f"Array shape={value.shape}"
            print(f" - {key}: {val_str}")
        print("="*40)

    @property
    def metadata(self) -> Dict[str, Any]:
        return self._metadata    

    @property
    def observation_dimension(self) -> int:
        return self._input_dim    

    @property
    def action_dimension(self) -> int:
        return self._output_dim