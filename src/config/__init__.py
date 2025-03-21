from .global_sim_params import *
from .camera_params import CAMERA_SETTINGS
from .recording_paths import RECORDING_PATHS
from .sim_overrides import SIMULATION_OVERRIDES


def load_sim_config(simulation_name):
    config = {
        "FRICTION_COEFFICIENT": FRICTION_COEFFICIENT,
        "RESTITUTION": RESTITUTION,
        "TIMESTEP": TIMESTEP,
        "INCLINE_ANGLE_RAD": INCLINE_ANGLE_RAD,
        "RECORD_VIDEO": RECORD_VIDEO,
        "CAMERA": CAMERA_SETTINGS.get(simulation_name, CAMERA_SETTINGS["default"]),
        "RECORDING_PATH": RECORDING_PATHS.get(simulation_name, None)
    }
    overrides = SIMULATION_OVERRIDES.get(simulation_name, {})
    config.update(overrides)
    return config
