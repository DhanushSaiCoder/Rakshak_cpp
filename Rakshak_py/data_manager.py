import os
import json

# Paths for depth calculation
DATA_DIR = "data"
DATASETS_CONFIG = {
    "person": (os.path.join(DATA_DIR, "person_depth_data.json"), "person_data"),
    "target": (os.path.join(DATA_DIR, "target_depth_data.json"), "target_data"),
    "pid_data": (os.path.join(DATA_DIR, "pid_data.json"), "pid_data"),
    "fov_data": (os.path.join(DATA_DIR, "fov_data_m.json"), "fov_data"),
    # "dynamic_zero_pos": (os.path.join(DATA_DIR, "fov_data_m.json"), "fov_data"),
}

class DataManager:
    def __init__(self):
        self.datasets = {}
    
    def _convert_keys_to_int_tuples(self, data_dict):
        converted = {}
        for k, v in data_dict.items():
            try:
                converted[int(k)] = v  
            except ValueError:
                converted[k] = v
        return converted
    
    def load_all(self):
        for name in DATASETS_CONFIG:
            self.load(name)

    def load(self, dataset_name):
        path, root_key = DATASETS_CONFIG[dataset_name]
        if not os.path.exists(path):
            print(f"[ERROR] {dataset_name}: file not found at {path}")
            self.datasets[dataset_name] = {}
            return

        try:
            with open(path, 'r') as f:
                full_data = json.load(f)
            data = full_data.get(root_key, {})
            if not data:
                print(f"[WARNING] Root key '{root_key}' missing in {path}")
    
            # Automatically convert string keys to int for specific datasets
            if dataset_name == "fov_data":
                if "static_fov" in data:
                    data["static_fov"] = self._convert_keys_to_int_tuples(data["static_fov"])
                if "pid_fov" in data:
                    data["pid_fov"] = self._convert_keys_to_int_tuples(data["pid_fov"])

            elif dataset_name == "pid_data":
                data = self._convert_keys_to_int_tuples(data)

            self.datasets[dataset_name] = data
        except json.JSONDecodeError:
            print(f"[ERROR] {dataset_name}: JSON decode error in {path}")
            self.datasets[dataset_name] = {}


    def get(self, dataset_name, zoom_level=None, subkey=None):
        root = self.datasets.get(dataset_name, {})
        if subkey:
            root = root.get(subkey, {})
        if zoom_level is None:
            return root
        # allow int or str keys
        return root.get(str(zoom_level), root.get(int(zoom_level)))


    def update_zoom(self, dataset_name, zoom_level, data, subkey=None):
        zoom_key = str(zoom_level)
        if dataset_name not in DATASETS_CONFIG:
            print(f"[ERROR] Unknown dataset: {dataset_name}")
            return

        path, root_key = DATASETS_CONFIG[dataset_name]

        # Load existing file content
        try:
            with open(path, 'r') as f:
                file_data = json.load(f)
        except Exception as e:
            print(f"[ERROR] Reading {path} failed: {e}")
            return

        # Ensure root key exists
        if root_key not in file_data:
            file_data[root_key] = {}

        # Navigate to subkey if needed
        if subkey:
            if subkey not in file_data[root_key]:
                file_data[root_key][subkey] = {}
            file_data[root_key][subkey][zoom_key] = data
        else:
            file_data[root_key][zoom_key] = data

        # Write back to JSON
        try:
            with open(path, 'w') as f:
                json.dump(file_data, f, indent=2)
            print(f"[INFO] {dataset_name} zoom {zoom_key} updated in {path}")
        except Exception as e:
            print(f"[ERROR] Writing to {path} failed: {e}")
            return

        # Update in-memory
        if subkey:
            self.datasets.setdefault(dataset_name, {}).setdefault(subkey, {})[int(zoom_key)] = data
        else:
            self.datasets.setdefault(dataset_name, {})[int(zoom_key)] = data

