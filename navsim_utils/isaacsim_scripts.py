from isaacsim.core.utils.prims import find_matching_prim_paths
import omni.usd


def set_vertiport_operator_id_to_pads():
    stage = omni.usd.get_context().get_stage()

    vertiport_prim_paths = find_matching_prim_paths("/*/*/VERT_OP_*")

    for vertiport_prim_path in vertiport_prim_paths:
        vertiport_prim = stage.GetPrimAtPath(vertiport_prim_path)

        vertiport_operator_id = vertiport_prim.GetAttribute("NavSim:id").Get()

        prim_pads = vertiport_prim.GetChildren()[1].GetChildren()

        for prim_pad in prim_pads:
            operator_id_attribute = prim_pad.GetAttribute("NavSim:operator_id")
            current_operator_id = operator_id_attribute.Get()
            
            if not current_operator_id:
                operator_id_attribute.Set(vertiport_operator_id)


def set_uav_attributes():
    stage = omni.usd.get_context().get_stage()
    
    uav_operator_prim_paths = find_matching_prim_paths("/*/*/UAV_OP_*")

    for operator_prim_path in uav_operator_prim_paths:
        operator_prim = stage.GetPrimAtPath(operator_prim_path)

        uav_operator_id = operator_prim.GetAttribute("NavSim:id").Get()
        
        prim_uavs = operator_prim.GetChildren()[0].GetChildren()

        for prim_uav in prim_uavs:
            operator_id_attribute = prim_uav.GetAttribute("NavSim:operator_id")
            id_attribute = prim_uav.GetAttribute("NavSim:id")
            
            current_operator_id = operator_id_attribute.Get()
            current_id = id_attribute.Get()
            
            if not current_operator_id:
                operator_id_attribute.Set(uav_operator_id)

            if not current_id:
                id_attribute.Set(prim_uav.GetName())
            
