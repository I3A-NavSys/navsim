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
MAX_VERT_OPS = int(os.getenv("MAX_VERT_OPS", 2))
PRIVACY = parse_str_list(os.getenv("PRIVACY", '[True, False]'))
SERVICE_TYPES = parse_str_list(os.getenv("SERVICE_TYPES", '[["DELIVERY", "PASSENGER_TRANSPORT"], ["DELIVERY"]]'))
MAX_PADS = parse_str_list(os.getenv("MAX_PADS", '[10, 5]'))
LOCATION = parse_str_list(os.getenv("LOCATION", '[[100, 500], [-600, -200]]'))
MQTT_HOST_ADDRESS = os.getenv("MQTT_HOST_ADDRESS", "127.0.0.1")
MQTT_HOST_PORT = int(os.getenv("MQTT_HOST_PORT", "1883"))