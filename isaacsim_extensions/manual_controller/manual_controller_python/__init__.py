import sys
from os import path

current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../.."))

if project_root_path not in sys.path:
    sys.path.append(project_root_path)

from .extension import *
