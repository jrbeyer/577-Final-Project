import numpy.random as rand
import numpy as np

def get_single_agent_test_cfg():
    cfg = {
    "name": "single_agent_test",
    "world": "SimpleUnderwater",
    "package_name": "Ocean",
    "main_agent": "auv0",
    "ticks_per_sec": 100,
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "IMUSensor"
                },
                {
                    "sensor_type": "LocationSensor"
                },
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": [0, 0, -5]
        }
    ]
    }
    return cfg

def get_multi_agent_test_cfg():
    cfg = {
    "name": "test_rgb_camera",
    "world": "SimpleUnderwater",
    "package_name": "Ocean",
    "main_agent": "auv0",
    "ticks_per_sec": 100,
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": [10, 10, -5]
        },
        {
            "agent_name": "auv1",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": [10, -10, -5]
        },
        {
            "agent_name": "auv2",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": [-10, 10, -5]
        },
        {
            "agent_name": "auv3",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": [-10, -10, -5]
        },
        {
            "agent_name": "auv4",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": [0, 12, -5]
        }
    ]
    }
    return cfg


def get_many_agent_test_cfg():
    seed = 0
    rand.seed(seed)
    locations = []
    # bound = 30
    bound = 20
    # bound = 15
    # bound = 10
    # bound_z = 2
    bound_z = 1
    while len(locations) < 10:
        bad_point = False
        potential_x = rand.randint(-bound, bound)
        potential_y = rand.randint(-bound, bound)
        potential_z = -10 + rand.randint(-bound_z, bound_z)

        # make sure more than 4 meters from another agent
        for l in locations:
            if np.linalg.norm(np.array([potential_x, potential_y, potential_z]) - np.array(l)) < 4:
                bad_point = True
        if bad_point: continue
        locations.append([potential_x, potential_y, potential_z])

    cfg = {
    "name": "test_rgb_camera",
    "world": "SimpleUnderwater",
    "package_name": "Ocean",
    "main_agent": "auv0",
    "ticks_per_sec": 100,
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[0],
            "rotation_randomization": [0, 0, 3.14]
        },
        {
            "agent_name": "auv1",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[1],
            "rotation_randomization": [0, 0, 3.14]
        },
        {
            "agent_name": "auv2",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[2],
            "rotation_randomization": [0, 0, 3.14]
        },
        {
            "agent_name": "auv3",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[3],
            "rotation_randomization": [0, 0, 3.14]
        },
        {
            "agent_name": "auv4",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[4],
            "rotation_randomization": [0, 0, 3.14]
        },
        {
            "agent_name": "auv5",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[5],
            "rotation_randomization": [0, 0, 3.14]
        },
        {
            "agent_name": "auv6",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[6],
            "rotation_randomization": [0, 0, 3.14]
        },
        {
            "agent_name": "auv7",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[7],
            "rotation_randomization": [0, 0, 3.14]
        },
        {
            "agent_name": "auv8",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[8],
            "rotation_randomization": [0, 0, 3.14]
        },
        {
            "agent_name": "auv9",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "PoseSensor"
                }
            ],
            "control_scheme": 0,
            "location": locations[9],
            "rotation_randomization": [0, 0, 3.14]
        }
    ]
    }
    return cfg