from setuptools import find_packages, setup

setup(
    name="ics_rl",
    version="1.0.2",
    author="Ziwen Zhuang",
    author_email="",
    license="CC-BY-NC-4.0",
    license_files=["LICENSE"],
    packages=find_packages(),
    description="Fast and simple RL algorithms implemented in pytorch (ics_rl fork)",
    python_requires=">=3.11",
    install_requires=[
        "torch>=2.4.0",
        "torchvision>=0.5.0",
        "numpy>=1.16.4",
        "tensorboardX",
        "tensorboard",
        "tabulate",
        "GitPython",
        "onnx",
        "onnxscript",
        "regex",
        "PyYAML",
    ],
    extras_require={
        "mjlab": ["mjlab"],
    },
)
