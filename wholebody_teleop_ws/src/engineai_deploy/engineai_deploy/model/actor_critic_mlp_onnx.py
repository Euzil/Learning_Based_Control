import logging
import json
import ast
from pathlib import Path
from typing import Optional, Dict, Any, Union

import numpy as np

try:
    import torch  # Preloads the bundled CUDA/cuDNN libs
    print("PyTorch found. CUDA/cuDNN libs preloaded for ONNX.")
except ImportError:
    pass

import onnxruntime as ort

def convert_meta_data(s):
    """ 
    Parses metadata strings into appropriate Python/NumPy types.
    """
    if not isinstance(s, str):
        return s
    s = s.strip('"\'')
    if s.startswith('{') and s.endswith('}'):
        try:
            return ast.literal_eval(s)
        except (ValueError, SyntaxError):
            pass
    if ((s.startswith('[') and s.endswith(']')) or (s.startswith('(') and s.endswith(')'))):
        try:
            evaluated = ast.literal_eval(s)
            if isinstance(evaluated, (list, tuple)):
                return np.array(evaluated)
        except (ValueError, SyntaxError):
            pass
    if ',' in s:
        parts = [p.strip() for p in s.split(',')]
        try:
            return np.array([float(p) for p in parts])
        except (ValueError, TypeError):
            return parts
    try:
        if '.' in s: return float(s)
        return int(s)
    except ValueError:
        pass
    return s

class Policy:
    """ Wrapper class for loading and executing ONNX RL policies with metadata.
    """
    def __init__(self, file_path: Union[str, Path], verbose: bool = False):
        self._session: Optional[ort.InferenceSession] = None
        self._metadata: Dict[str, Any] = {}
        self._input_dim: int = -1
        self._output_dim: int = -1
        
        # ONNX specific internal tracking
        self._input_name: str = ""
        self._output_names: list = []
        
        self._load_model(file_path, verbose)
        

    def _load_model(self, file_path: Union[str, Path], verbose: bool = False):
        """
        Loads an ONNX model and its embedded metadata.
        """
        path = Path(file_path)
        if not path.exists():
            raise ValueError(f"[Policy] File not found: {str(path)}")
        
        # Select providers based on device
        if 'CUDAExecutionProvider' in ort.get_available_providers():
            print("[Policy] Using CUDA for execution")
            providers = ['CUDAExecutionProvider']
        else:
            print("[Policy] Using CPU for execution")
            providers = ['CPUExecutionProvider']
    
        try:
            # Initialize Session
            self._session = ort.InferenceSession(str(path), providers=providers)
            
            # Identify Input/Output names and dimensions
            input_meta = self._session.get_inputs()[0]
            self._input_name = input_meta.name
            self._input_dim = input_meta.shape[-1]
            self._output_names = [o.name for o in self._session.get_outputs()]
            
            # Parse metadata from ONNX custom_metadata_map
            raw_metadata = self._session.get_modelmeta().custom_metadata_map
            self._metadata = {k: convert_meta_data(v) for k, v in raw_metadata.items()}

            # Determine output dimension from metadata if possible (to match original logic)
            if 'default_joint_pos' in self._metadata:
                self._output_dim = len(self._metadata['default_joint_pos'])
            else:
                self._output_dim = self._session.get_outputs()[0].shape[-1]

            if verbose:
                self._print_summary(path)

            # Warmup
            dummy_input = np.zeros((1, self._input_dim), dtype=np.float32)
            for _ in range(10):
                _ = self._session.run(self._output_names, {self._input_name: dummy_input})

        except Exception as e:
            raise Exception(f"[Policy] Failed to load ONNX model: {e}")

    def forward(self, observations: np.ndarray) -> np.ndarray:
        """ Runs inference
        """
        if self._session is None:
            raise RuntimeError("[Policy] Model not loaded. Call load_model() first.")

        # Convert to float32
        obs_np = observations.astype(np.float32)

        if obs_np.ndim == 1:
            obs_np = np.expand_dims(obs_np, axis=0)

        # Execute
        results = self._session.run(self._output_names, {self._input_name: obs_np})
        
        # Convert back to np.ndarray with correct type
        return results[0].astype(observations.dtype)

    def _print_summary(self, path: Path):
        print("="*40)
        print(f"[Policy] ONNX Model Loaded: {path.name}")
        print(f"[Policy] Providers:        {self._session.get_providers()}")
        print(f"[Policy] Input Dim:         {self._input_dim}")
        print(f"[Policy] Output Dim:        {self._output_dim}")
        print("-" * 40)
        print("[Policy] Metadata:")
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