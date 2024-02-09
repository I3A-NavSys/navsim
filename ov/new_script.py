import carb
from omni.kit.scripting import BehaviorScript
import omni.usd
from pxr import Gf, UsdGeom



class NewScript(BehaviorScript):
    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")

    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
        carb.log_info(f"{type(self).__name__}.on_play()->{self.prim_path}")

        stage = omni.usd.get_context().get_stage()
       
        prim_path = "/World/Cube"  
        prim = stage.GetPrimAtPath(prim_path)

        if prim:
            xform = UsdGeom.Xformable(prim) 
            local_transformation: Gf.Matrix4d = xform.GetLocalTransformation()
            translation: Gf.Vec3d = local_transformation.ExtractTranslation()
            rotation: Gf.Rotation = local_transformation.ExtractRotation()
            scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in local_transformation.ExtractRotationMatrix()))
        
            # Imprimir la posición XYZ
            print(f"Posición XYZ de {prim_path}: {translation}")



    def on_pause(self):
        carb.log_info(f"{type(self).__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        carb.log_info(f"{type(self).__name__}.on_stop()->{self.prim_path}")
        print("STOP")

    def on_update(self, current_time: float, delta_time: float):
        carb.log_info(f"{type(self).__name__}.on_update({current_time}, {delta_time})->{self.prim_path}")

        stage = omni.usd.get_context().get_stage()
       
        prim_path = "/World/Cube"  
        prim = stage.GetPrimAtPath(prim_path)

        if prim:
            xform = UsdGeom.Xformable(prim) 
            local_transformation: Gf.Matrix4d = xform.GetLocalTransformation()
            translation: Gf.Vec3d = local_transformation.ExtractTranslation()
            rotation: Gf.Rotation = local_transformation.ExtractRotation()
            scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in local_transformation.ExtractRotationMatrix()))
        
            # Imprimir la posición XYZ
            print(f"Posición XYZ de {prim_path}: {translation}")

