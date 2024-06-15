import carb
from carb.input import KeyboardEventType, GamepadInput

import omni.appwindow
from omni.kit.scripting import BehaviorScript

from pxr import Sdf, Usd, UsdGeom, Gf, UsdPhysics





class UamMinidrone(BehaviorScript):

    xform : UsdGeom.Xformable 

    mass: float
   
    pos: Gf.Vec3d
    rot: Gf.Vec3d
    
    force_atr:  Usd.Attribute
    torque_atr: Usd.Attribute
    CmdVel_atr: Usd.Attribute
  
    #keyboard_sub_id


    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")
   
        self.xform = UsdGeom.Xformable(self.prim) 

        #attrs = self.prim.GetAttributes()

        mass_attr = self.prim.GetAttribute("physics:mass")
        self.mass = mass_attr.Get()

        self.force_atr = self.prim.CreateAttribute(
            "physxForce:force", 
            Sdf.ValueTypeNames.Float3)
        self.force_atr = self.prim.GetAttribute("physxForce:force")
       
        self.torque_atr = self.prim.CreateAttribute(
            "physxForce:torque", 
            Sdf.ValueTypeNames.Float3)
        self.torque_atr = self.prim.GetAttribute("physxForce:torque")
        

        self.CmdVel_atr = self.prim.CreateAttribute(
            "NavSim:CmdVel", 
            Sdf.ValueTypeNames.Float3)
        
        self.CmdVel_atr = self.prim.GetAttribute("NavSim:CmdVel")

        CmdVel = (0,0,0)
        self.CmdVel_atr.Set(CmdVel)





    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")

 


    def on_play(self):
        carb.log_info(f"{type(self).__name__}.on_play()->{self.prim_path}")
        print(f"PLAY  {self.prim_path}")

        g: float = 9.81
        #force: Gf.Vec3d = self.force_atr.Get()
        force: Sdf.ValueTypeNames.Float3 = (0,0,self.mass*g*1.001)
        self.force_atr.Set(force)

        #torque: Gf.Vec3d = self.torque_atr.Get()
        torque = (0,0,0)
        self.torque_atr.Set(torque)






       
    def on_pause(self):
        carb.log_info(f"{type(self).__name__}.on_pause()->{self.prim_path}")



    def on_stop(self):
        carb.log_info(f"{type(self).__name__}.on_stop()->{self.prim_path}")
        print(f"STOP  {self.prim_path}")




    def on_update(self, current_time: float, delta_time: float):
        carb.log_info(f"{type(self).__name__}.on_update({current_time}, {delta_time})->{self.prim_path}")

        pose: Gf.Matrix4d = self.xform.GetLocalTransformation()           

        # posición XYZ
        self.pos = pose.ExtractTranslation()
        pos_formatted = tuple("{:.2f}".format(value) for value in self.pos)
        #print(f"Posición XYZ de {self.prim_path}: {pos_formatted}")        

        # rotación XYZ
        rotation: Gf.Rotation = pose.ExtractRotation()      
        self.rot = rotation.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
        rot_formatted = tuple("{:.0f}".format(value) for value in self.rot)
        #print(f"Rotación XYZ de {self.prim_path}: {rot_formatted}") 


        CmdVel: Gf.Vec3d = self.CmdVel_atr.Get()
        #print(CmdVel)



