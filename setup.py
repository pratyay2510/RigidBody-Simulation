from setuptools import setup, find_packages
import os

# Load the long description from README.md if available
with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()

# Parse dependencies from requirements.txt
with open("requirements.txt") as f:
    requirements = f.read().splitlines()

setup(
    name="RigidBody-Simulation",
    version="0.1.0",
    author="Aditya Gambhir, Pratyay Dutta",
    author_email="agamb031@ucr.edu, pdutt005@ucr.edu",
    description="A customizable rigid body simulation framework using MuJoCo with impulse-based collision and friction modeling.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Aditya-gam/RigidBody-Simulation.git",
    project_urls={
        "Documentation": "https://github.com/Aditya-gam/RigidBody-Simulation.git",
        "Source": "https://github.com/Aditya-gam/RigidBody-Simulation.git",
    },
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    include_package_data=True,
    install_requires=requirements,
    python_requires=">=3.8",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Developers",
        "Topic :: Scientific/Engineering :: Physics",
        "Topic :: Scientific/Engineering :: Visualization",
        "Operating System :: OS Independent",
        "Development Status :: 4 - Beta",
    ],
    license="",
    keywords=[
        "mujoco",
        "physics",
        "simulation",
        "rigid-body",
        "collision",
        "impulse",
        "custom physics",
        "visualization",
    ],
)
