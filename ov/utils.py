import os
from pathlib import Path

# -- FUNCTION find_file ------------------------------------------------------------------------------------
# This method finds the absolute path to a specific file given its name and the name of the root folder of the project
# The output is given in using fordward slashes (/). Example: C:/Users/Victor/...
# The data returned is a list composed of as many paths as files with the same name
# When choosing which path is the one you need, take into account that the finding process is done topdown
# If the file does not exists or the project root folder is not found, it will return an empty list
# ----------------------------------------------------------------------------------------------------------
def find_file(root_project_name, file_name):
    file_paths = []
    root_project_folder = get_root_folder(root_project_name)

    if root_project_folder is not None:
        for dirpath, dirnames, filenames in os.walk(root_project_folder):
            if file_name in filenames:
                file_paths.append(Path(os.path.join(dirpath, file_name)).resolve().as_posix())
    
    return file_paths


# -- FUNCTION get_root_folder ------------------------------------------------------------------------------
# This method returns the absolute path to the project root folder given its name
# It is CASE SENSITIVE -> NAME != name
# Again, the format used is forward slashes (/)
# If the folder is not found (we arrived to C:/), it will return None
# ----------------------------------------------------------------------------------------------------------
def get_root_folder(root_name):
    parent = Path(__file__).resolve().parent
    last_checked = Path(__file__).resolve()

    while parent.name != last_checked.name:        
        if parent.name == root_name:
            return parent.as_posix()
        
        last_checked = parent
        parent = parent.parent
        
    return None