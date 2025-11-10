import os
import sys

def get_navsim_root_path():
    """
    Get the path to NAVSIM root project, append it to sys.path if not already in
    and return it with standar slashes.
    """
    
    current_file_path = os.path.dirname(__file__)
    navsim_root_path = os.path.abspath(os.path.join(current_file_path, ".."))
    
    return navsim_root_path.replace("\\", "/")