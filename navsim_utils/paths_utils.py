import os


# Navsim Root Path
current_file_path = os.path.dirname(__file__)
abs_path = os.path.abspath(os.path.join(current_file_path, ".."))
project_root_path = abs_path.replace("\\", "/")

# Assets Paths
assets_path = os.path.join(project_root_path, "assets")
ui_icons_path = os.path.join(assets_path, "ui_icons")