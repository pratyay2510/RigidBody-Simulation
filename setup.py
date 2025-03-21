from setuptools import setup, find_packages
import os

# Load the long description from README.md
with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()

# Parse dependencies from requirements.txt
with open("requirements.txt") as f:
    requirements = f.read().splitlines()

setup(
    name="RigidBody-Simulation-Framework",
    version="1.0.0",
    author="Aditya Gambhir, Pratyay Dutta",
    author_email="agamb031@ucr.edu, pdutt005@ucr.edu",
    description="A robust, production-ready rigid body simulation framework utilizing MuJoCo for visualization with custom impulse-based collision and friction physics.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Aditya-gam/RigidBody-Simulation",
    project_urls={
        "Documentation": "https://github.com/Aditya-gam/RigidBody-Simulation",
        "Source": "https://github.com/Aditya-gam/RigidBody-Simulation",
        "Paper Reference": "https://doi.org/10.1145/882262.882358",
    },
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    include_package_data=True,
    install_requires=requirements,
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Topic :: Scientific/Engineering :: Physics",
        "Topic :: Scientific/Engineering :: Visualization",
        "Operating System :: OS Independent",
    ],
    license="Apache License 2.0",
    keywords=[
        "mujoco",
        "rigid-body simulation",
        "impulse-based collision",
        "friction modeling",
        "physics engine",
        "3D simulation",
        "scientific visualization",
        "simulation framework",
        "custom integrators",
    ],
)
