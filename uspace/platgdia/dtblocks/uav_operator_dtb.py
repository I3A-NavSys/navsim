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
MAX_UAV_OPS = int(os.getenv("MAX_UAV_OPS", 1))
SERVICE_TYPES = parse_str_list(os.getenv("SERVICE_TYPES", '[["DELIVERY", "PASSENGER_TRANSPORT"]]'))
UAV_FLEET = parse_str_list(os.getenv("UAV_FLEET", '[["DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "DELIVERY", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT", "PASSENGER_TRANSPORT"]]'))
MQTT_HOST_ADDRESS = os.getenv("MQTT_HOST_ADDRESS", "127.0.0.1")
MQTT_HOST_PORT = int(os.getenv("MQTT_HOST_PORT", "1883"))