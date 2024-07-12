import numpy as np

from omni.isaac.core.world import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.objects.cuboid import FixedCuboid
from omni.isaac.core.utils.stage import get_current_stage, add_reference_to_stage
from pxr import UsdGeom, Gf, UsdLux, Sdf

import random

class Scenes:
    def one_hundred_drones(self):
        world = World()
        self.add_light_to_stage()
        world.scene.add_default_ground_plane()

        # Create a cuboid
        self._cuboid_list = []
        abejorro_usd_path = "C:/Users/ikern/Documents/Repositorios/navsim/ov/extensions/assets/abejorro.usd"
        abejorro_ref_path = "/World/abejorros/abejorro"

        for i in range(10):
            for j in range(10):
                self._cuboid_list.append(
                    FixedCuboid("/World/Cubes/cube_" + str(i) + str(j), position=np.array([i, j, 0.25]), size=0.5)
                )

                add_reference_to_stage(usd_path = abejorro_usd_path, prim_path = abejorro_ref_path + str(i) + str(j))
                prim = get_current_stage().GetPrimAtPath(abejorro_ref_path + str(i) + str(j))

                # Randomly set if this drone is able to receive commands or not
                self.set_random_manipulable_UAV(prim)

                xformable = UsdGeom.Xformable(prim)

                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3f(i, j, 0.55))
                        
                    if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                        op.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))

        # Add user-loaded objects to the World
        world.scene.add(self._cuboid_list[0])
    
    def add_light_to_stage(self):
        """
        A new stage does not have a light by default.  This function creates a spherical light
        """
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        XFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])

    # This function is for testing purposes, it just assign a random drone a variable to indicate that it is able to receive commands
    def set_random_manipulable_UAV(self, prim):
        rand = random.randint(0, 1)

        if rand >= 0.5:
            prim.SetCustomDataByKey("manipulable", True)