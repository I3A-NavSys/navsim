import carb
from carb.input import KeyboardEventType

from omni.kit.scripting import BehaviorScript

from pxr import Sdf, Usd, UsdGeom, Gf

class UamMinidrone(BehaviorScript):
    def on_init(self):
        self.xform = UsdGeom.Xformable(self.prim)

        # Mass
        self.mass_attr = self.prim.GetAttribute("physics:mass")

        # Force
        self.force_atr = self.prim.CreateAttribute("physxForce:force", Sdf.ValueTypeNames.Float3)
        self.force_atr = self.prim.GetAttribute("physxForce:force")

        # Torque
        self.torque_atr = self.prim.CreateAttribute("physxForce:torque", Sdf.ValueTypeNames.Float3)
        self.torque_atr = self.prim.GetAttribute("physxForce:torque")

        # Apply gravity force
        gravity = (0, 0, self.mass_attr.Get() * 9.81)
        self.force_atr.Set(gravity)

    def on_destroy(self):
        pass

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        pass

    def on_update(self, current_time: float, delta_time: float):
        pose: Gf.Matrix4d = self.xform.GetLocalTransformation()           

        # Position XYZ
        self.pos = pose.ExtractTranslation()
        pos_formatted = tuple("{:.2f}".format(value) for value in self.pos)      

        # Rotation XYZ
        rotation: Gf.Rotation = pose.ExtractRotation()      
        self.rot = rotation.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
        rot_formatted = tuple("{:.0f}".format(value) for value in self.rot)