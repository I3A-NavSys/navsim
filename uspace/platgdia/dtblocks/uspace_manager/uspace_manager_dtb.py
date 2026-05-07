import os
import json

def parse_str_list(str_list: str) -> list:
    '''
    Parses a string representation of a list into an actual list.
    Args:
        str_list (str): A string representation of a list, e.g., "[1, 2, 3]" or '[["DELIVERY", "PASSENGER_TRANSPORT"]]'.
    Returns:
        list: The parsed list.
    Raises:
        ValueError: If the string cannot be parsed into a list.
    '''
    try:
        return json.loads(str_list)
    
    except json.JSONDecodeError:
        raise ValueError(f"Failed to parse list: {str_list}")


# Get Configuration Variables
ROUTING_ALGORITHM = os.getenv("ROUTING_ALGORITHM", "GRID")
MQTT_HOST_ADDRESS = os.getenv("MQTT_HOST_ADDRESS")
MQTT_HOST_PORT = int(os.getenv("MQTT_HOST_PORT", "1883"))