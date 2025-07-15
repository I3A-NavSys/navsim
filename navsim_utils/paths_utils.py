from os import path
import sys

def get_navsim_root_path():
    """
    Get the path to NAVSIM root project, append it to sys.path if not already in
    and return it with standar slashes.
    """
    
    current_file_path = path.dirname(__file__)
    navsim_root_path = path.abspath(path.join(current_file_path, ".."))
    
    if navsim_root_path not in sys.path:
        sys.path.append(navsim_root_path)
    
    return navsim_root_path.replace("\\", "/")