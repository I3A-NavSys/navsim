# Standard library imports
import sys
import os

# Related third party imports
from isaacsim.gui.components import ui
from isaacsim.gui.components.element_wrappers import DropDown
from omni.isaac.core.utils.stage import get_current_stage
from isaacsim.core.utils.prims import find_matching_prim_paths, get_prim_at_path


from .paths_utils import project_root_path


class MinimalComboBoxItem(ui.AbstractItem):
    def __init__(self, text):
        super().__init__()
        self.model = ui.SimpleStringModel(text)

class MinimalComboBoxModel(ui.AbstractItemModel):
    def __init__(self):
        super().__init__()

        self._current_index = ui.SimpleIntModel()
        self._current_index.add_value_changed_fn(
            lambda a: self._item_changed(None))

        self._items = []

    def get_item_children(self, item):
        return self._items

    def get_item_value_model(self, item, column_id):
        if item is None:
            return self._current_index
        return item.model
    
    def append_child_item(self, value):
        self._items.append(MinimalComboBoxItem(value))
        self._item_changed(None)
        
    def remove_children(self):
        self._items.clear()
        
    def set_children(self, items):
        self._items = [MinimalComboBoxItem(item) for item in items]
        self._item_changed(None)
    
    def get_selection(self):
        if self._items:
            item = self._items[self._current_index.get_value_as_int()]
            return item.model.get_value_as_string()

        return None

class ExtensionUtils:

    def __init__(self):
         # Color
        self.KIT_GREEN = 0xFF8A8777

        # Label
        self.LABEL_PADDING = 120

        # Spacing
        self.SPACING_S = 8
        self.SPACING_M = self.SPACING_S * 2
        self.SPACING_L = self.SPACING_M * 2
        self.SPACING_XL = self.SPACING_L * 2

        # Height
        self.MINIMAL_HEIGHT = 0

        # Width
        self.MINIMAL_WIDTH = 0

        self.Window_dark_style = {
            "Window": {"background_color": 0xFF444444}
        }


        self.VStack_A = {
            "VStack": {
                "margin_width": 10, 
                "margin_height": 0
            }
        }


        self.VStack_B = {
            "VStack": {
                "margin_width": 10,
                "margin_height": 5
            }
        }


        self.HStack_A = {
            "HStack": {
                "margin_width": 10,
                "margin_height": 5
            }
        }


        self.Label_A = {
            "Label": {
                "font_size": 12,
                "color": 0xFFDDDDDD
            }
        }


        self.colors = {
            "R": 0xFF5555AA,
            "G": 0xFF76A371,
            "B": 0xFFA07D4F
        }


        self.CollapsableFrame_style = {
            "CollapsableFrame": {
                "background_color": 0xFF343432,
                "secondary_color": 0xFF343432,
                "color": 0xFFAAAAAA,
                "border_radius": 4.0,
                "border_color": 0x0,
                "border_width": 0,
                "font_size": 14,
                "padding": 0,
            },
            "HStack::header": {"margin": 5},
            "CollapsableFrame:hovered": {"secondary_color": 0xFF3A3A3A},
            "CollapsableFrame:pressed": {"secondary_color": 0xFF343432},
        }

        self.ScrollingFrame_style = {
            "ScrollingFrame": {
                "background_color": 0xFF343432,
                "secondary_color": 0xFF343432,
                "color": 0xFFAAAAAA,
                "border_radius": 4.0,
                "border_color": 0x0,
                "border_width": 0,
                "font_size": 14,
                "padding": 0,
            },
            "HStack::header": {"margin": 5},
            "ScrollingFrame:hovered": {"secondary_color": 0xFF3A3A3A},
            "ScrollingFrame:pressed": {"secondary_color": 0xFF343432},
        }

    #------------------------------------------------------------------------------------------------------------------
    # USER INTERFACE

    def build_uav_selector(self):
        with ui.HStack(spacing=5):
            # Dropdown selector
            self.combobox_model = MinimalComboBoxModel()
            self.UAV_dropdown = ui.ComboBox(self.combobox_model)
            icon_path = os.path.join(project_root_path, "assets/ui_icons/reload.png")
            
            # Button to refresh manipulable UAVs
            ui.Button(
                image_url=icon_path,
                image_height=15,
                width=50,
                style={"Button.Image": {"alignment": ui.Alignment.CENTER}},
                clicked_fn=self.get_navsim_UAV_names
            )

        return self.UAV_dropdown
    
    def get_navsim_UAV_names(self):
        from omni.isaac.core.utils.stage import get_current_stage
        stage = get_current_stage()
        
        self.UAV_dropdown.model.remove_children()
        
        uavs = []
        if stage is not None:
            for prim in stage.Traverse():
                att = prim.GetAttribute("NavSim:type")
                if att.IsValid() and att.Get() == "UAV":
                        # self.UAV_dropdown.model.append_child_item(prim.GetName())
                        uavs.append(prim.GetName())
                        
        self.UAV_dropdown.model.set_children(uavs)

    #------------------------------------------------------------------------------------------------------------------
    # MISCELLANEOUS UTILS

    def get_prim_by_name(self, name):
        stage = get_current_stage()
        for prim in stage.Traverse():
            if prim.GetName() == name:
                    return prim
            
        return None
    
    def get_private_vertiport_prims(self):
        prim_paths = find_matching_prim_paths("/World/Vertiports/Private/*/PrivPad_*")
        prims = [get_prim_at_path(path) for path in prim_paths]

        return prims
    
    def get_public_vertiport_prims(self):
        prim_paths = find_matching_prim_paths("/World/Vertiports/Public/*/PubPad_*")
        prims = [get_prim_at_path(path) for path in prim_paths]

        return prims