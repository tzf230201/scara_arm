import os
import json
import shutil

def origin_load_from_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "config_origin.json")
    print(f"{config_path}")
    
    default_config = {
        "origin_1": -1, #-3306079
        "origin_2": 0,
        "origin_3": 0,
        "origin_4": 0
    } 

    if not os.path.exists(config_path):
        with open(config_path, "w") as f:
            json.dump(default_config, f, indent=4)
        print("config_origin.json not found. Created new file with defaults.")
        return tuple(default_config.values())

    try:
        with open(config_path, "r") as f:
            config_data = json.load(f)

        origin_1 = config_data.get("origin_1", 0)
        origin_2 = config_data.get("origin_2", 0)
        origin_3 = config_data.get("origin_3", 0)
        origin_4 = config_data.get("origin_4", 0)

        print(f"Loaded origins: {origin_1}, {origin_2}, {origin_3}, {origin_4}")
        return origin_1, origin_2, origin_3, origin_4

    except json.JSONDecodeError:
        # Backup the invalid file
        backup_path = config_path + ".backup_invalid"
        shutil.copy(config_path, backup_path)
        print(f"Invalid JSON format. Backup saved to {backup_path}. Rewriting with defaults.")
        
        with open(config_path, "w") as f:
            json.dump(default_config, f, indent=4)
        return tuple(default_config.values())


def origin_save_to_config(encoders):
    config_data = {
        "origin_1": encoders[0],
        "origin_2": encoders[1],
        "origin_3": encoders[2],
        "origin_4": encoders[3],
    }
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_origin.json")
    with open(config_path, "w") as f:
        json.dump(config_data, f, indent=4)

    print(f"Origin saved to {config_path}")
    
origins = origin_load_from_config()

def get_origins():
    global origins
    
    return origins
