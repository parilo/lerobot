import json
import numpy as np

def read_json_file(file_path):
    """
    Reads a JSON file and returns its contents as a dictionary.

    Parameters:
    file_path (str): The path to the JSON file to read.

    Returns:
    dict: The contents of the JSON file as a dictionary.
    """
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print(f"Error: File not found at {file_path}")
        return {}
    except json.JSONDecodeError as e:
        print(f"Error: Failed to decode JSON. {e}")
        return {}


def trim_repeated_values(trajectory: np.ndarray) -> np.ndarray:
    """
    Trim the repeated starting values in the trajectory.
    Removes consecutive duplicate observations from the list.
    """
    if len(trajectory) < 2:
        return trajectory

    first_step = trajectory[0]  # Always keep the first value

    # Loop through the trajectory and add only non-duplicate consecutive points
    for i in range(1, len(trajectory)):
        if not np.allclose(trajectory[i], first_step):
            return trajectory[i-1:]

    return trajectory[:1]
