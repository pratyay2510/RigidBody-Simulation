# Rigid Body Sphere Simulation (Phase 1) - MuJoCo

## 📖 Project Overview

This project is part of Phase 1 in building a robust rigid body dynamics simulation platform. The objective for this phase is to simulate a single **sphere bouncing on a static floor** using **MuJoCo’s built-in physics engine** and provide a real-time, interactive visualization with adjustable parameters. The simulation allows dynamic exploration of gravity, restitution, and sphere placement.

This work is aligned with the equations of motion, collision handling, and impulse-based contact resolution methods described in the provided research paper.

---

## ✅ Key Features

- **Physics Engine**: Powered by MuJoCo’s `mj_step` for real-time simulation.
- **Adjustable Parameters**:
  - Gravity (cycling between preset modes).
  - Restitution (approximate control via contact solver parameters).
- **Interactive Controls**:
  - Camera control (rotate, pan, zoom).
  - Real-time repositioning of the sphere.
- **Continuous simulation**: Runs indefinitely until manually closed.

---

## 📁 Project Structure

```
RigidBody-Simulation/
├── README.md                # Project overview, setup instructions, etc.
├── .gitignore               # Git ignore file (see below)
├── requirements.txt         # List of required Python packages
├── setup.py                 # (Optional) Setup script for packaging/distribution
├── src/                     # Source code folder
│   ├── __init__.py
│   ├── main.py              # Entry point for running simulations
│   ├── simulation/          # Module containing simulation routines
│   │   ├── __init__.py
│   │   ├── physics.py       # Rigid body dynamics, time integration routines
│   │   ├── collision.py     # Collision detection and impulse resolution routines
│   │   ├── contact.py       # Contact resolution (including shock propagation)
│   │   ├── sdf.py           # SDF computation and triangulated surface helpers
│   │   └── utils.py         # Utility functions (logging, parameter parsing, etc.)
│   └── visualization/       # Module to interface with MuJoCo's rendering/GLFW window
│       ├── __init__.py
│       └── viewer.py        # Functions to set up and run the simulation window
├── models/                  # MuJoCo XML models for different objects (sphere, cube, etc.)
│   ├── sphere.xml
│   └── cube.xml
├── tests/                   # Test suite for unit testing simulation components
│   ├── __init__.py
│   └── test_simulation.py
└── data/                    # (Optional) Folder for logging simulation data/output

```

---

## ⚙️ Requirements

- Python 3.8+
- MuJoCo (version >= 2.3)
- GLFW (installed via `pip install glfw`)
- NumPy

### Installation:

```bash
pip install mujoco glfw numpy
```

> Make sure MuJoCo is installed and configured correctly with `MUJOCO_GL` environment variable if needed.

---

## 📜 XML Model: `sphere.xml`

- **Floor**:
  - Defined as a plane (`y=0`), large area, gray color.
- **Sphere**:
  - Positioned initially at `y = 1.0`.
  - Radius: `0.5 m`, mass `1.0 kg`.
  - Defined `free joint` to allow unrestricted motion.
  - Contact parameters are adjustable via `solimp` and `solref`.

---

## 🎮 Interactive Controls

| Control              | Action                                                                              |
| -------------------- | ----------------------------------------------------------------------------------- |
| `Left Mouse Button`  | Rotate the camera (vertical and horizontal).                                        |
| `Right Mouse Button` | Pan the camera.                                                                     |
| `Scroll Wheel`       | Zoom the camera in and out.                                                         |
| `P` Key              | Place the sphere at the current camera lookat (slightly above the floor).           |
| `R` Key              | Increase restitution (approximated using `solimp` parameter).                       |
| `G` Key              | Cycle through different gravity modes (`Earth gravity`, `Zero G`, `Light gravity`). |
| `Backspace`          | Reset simulation state to initial conditions.                                       |
| `Close Window`       | End simulation.                                                                     |

---

## 🔬 How Restitution & Gravity Adjustment Works

- **Restitution**:

  - Controlled by modifying the third value of `geom_solimp` for `ball_geom`.
  - Approximate elastic collision control between `0.0` and `1.0`.

- **Gravity Modes**:

  1. `Earth Gravity` (0, -9.81, 0)
  2. `Zero Gravity` (0, 0, 0)
  3. `Light Gravity` (0, -3.0, 0)

> These settings can be cycled in real-time using the `G` key.

---

## ✅ Next Steps for Future Phases

- Extend simulation to multiple spheres.
- Implement custom physics solver and collision detection using impulse-based methods (as per the paper).
- Add UI sliders for real-time parameter tweaking.
- Integrate contact friction controls.

---

## 📚 Reference

This project is based on the rigid body simulation methods described in the provided paper (`rigid_bodies.pdf`).

> It covers forward Euler integration, impulse-based collision resolution, effective mass computation, and simulation stability criteria.

---

## 🤝 Contribution

Contributions and ideas for improvement are welcome! Feel free to open an issue or pull a request.

---

## 📢 Author

- Simulation and integration by: *Aditya*
- Interactive controls and parameter tuning guided by: *rigid\_bodies.pdf* formulation and MuJoCo best practices.

---

## 🚀 Run the Simulation

```bash
cd src
python main.py
```
