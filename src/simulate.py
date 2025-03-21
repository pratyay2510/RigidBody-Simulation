# src/simulate.py

import argparse
import subprocess
import os
import sys


def run_simulation(sim_name):
    """
    Runs the specified simulation by invoking the corresponding simulation script.
    """
    simulation_map = {
        "cube_incline": "src/simulation/cube_incline.py",
        "ball_collision": "src/simulation/ball_collision.py",
        "single_sphere": "src/simulation/single_sphere_bounce.py",
        "compare_builtin": "src/simulation/compare_builtin_simulation.py",
        "multi_sphere": "src/simulation/multi_sphere_bounce.py"
    }

    if sim_name not in simulation_map:
        print(f"❌ Unknown simulation name: '{sim_name}'")
        print("Available simulations:")
        for sim in simulation_map:
            print(f"  ➡️ {sim}")
        sys.exit(1)

    sim_script_path = simulation_map[sim_name]
    sim_script_full_path = os.path.abspath(
        os.path.join(os.getcwd(), sim_script_path))

    if not os.path.isfile(sim_script_full_path):
        print(f"❌ Simulation script not found at {sim_script_full_path}")
        sys.exit(1)

    print(f"🚀 Running simulation: {sim_name}\n")
    subprocess.run(["python", sim_script_full_path], check=True)


def main():
    """
    Main function to parse CLI arguments and trigger simulations.
    """
    parser = argparse.ArgumentParser(
        description="MuJoCo Simulation Runner CLI"
    )
    parser.add_argument(
        "--sim",
        type=str,
        required=True,
        help="Simulation to run. Available: cube_incline, ball_collision, single_sphere, compare_builtin, multi_sphere"
    )

    args = parser.parse_args()
    run_simulation(args.sim)


if __name__ == "__main__":
    main()
