from __future__ import annotations

from typing import Any


def _to_str(v: Any, decimals: int = 6) -> str:
    if isinstance(v, list):
        fmt = f"{{:.{decimals}f}}"
        return ",".join(fmt.format(x) if isinstance(x, (int, float)) else str(x) for x in v)
    return str(v)


def attach_onnx_metadata(onnx_path: str, metadata: dict) -> None:
    """Attach key/value metadata to an ONNX model file in-place."""
    import onnx

    model = onnx.load(onnx_path)
    for k, v in metadata.items():
        entry = onnx.StringStringEntryProto(key=k, value=_to_str(v))
        model.metadata_props.append(entry)
    onnx.save(model, onnx_path)
