import os
from pathlib import Path

# -- FUNCTION find_file ------------------------------------------------------------------------------------
# This method finds the absolute path to a specific file or directory given its name and the name of the root folder
# The output is given in using fordward slashes (/). Example: C:/Users/Victor/...
# The data returned is the absolute path to the specified location
# If the file does not exists or the project root folder is not found, it will return None
# 
# Params:
# - root_project_name   -> It is the folder from the search is started (it will be usually 'ov')
# - parent_folder       -> It is the folder where you know the file or directory you are searching is in
# - file_name           -> It is the file or directory name you want
# - is_file             -> Boolean parameter to set the file_name is wheter a file or a directory
# 
# Example Use Case:
# - We want the path to FlightPlan.py
# - Then we must do 'find_file("ov", "uspace", "FlightPlan.py", True)
# - We set "ov" as root_project folder because this script is location dependent, that is, to find the specified 
#   root_project_folder is goes from bot to top from the file that is calling this function (get_root_folder).
#   So if we use this function from command_generator extension.py and we set the root as "uspace", it will fail
#   because "uspace" will not be found.
# - We set "uspace" as parent_folder to avoid searching in other folders such as 'assets', 'extensions', etc.
# - "FlightPlan.py" is the file whose path we want.
# - We set is_file to True as "FlightPlan.py" is a file.
# ----------------------------------------------------------------------------------------------------------
def find_file(root_project_name, parent_folder, file_name, is_file):
    root_project_folder = get_root_folder(root_project_name)
    found_parent_folder = False
    if is_file: search_index = 2
    else:       search_index = 1

    if root_project_folder is not None:
        for data in os.walk(root_project_folder):
            dirpath = data[0]
            search_range = data[search_index]

            path = Path(dirpath).resolve().as_posix()
            path_splitted = path.split("/")
            current_folder = path_splitted[-1]

            if not found_parent_folder and current_folder != parent_folder:
                continue

            found_parent_folder = True

            if file_name in search_range:
                return path + "/" + file_name
    
    return None


# -- FUNCTION get_root_folder ------------------------------------------------------------------------------
# This method returns the absolute path to the root folder given its name
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
