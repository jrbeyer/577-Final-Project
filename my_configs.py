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
            "location": [0, 0, -5]
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
            "location": [0, 2, -5]
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
            "location": [0, 4, -5]
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
            "location": [0, 6, -5]
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
            "location": [0, 8, -5]
        }
    ]
    }
    return cfg