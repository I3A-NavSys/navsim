class Command:

    def __init__(self, on=False, velX=0, velY=0, velZ=0, rotZ=0, duration=None):
        self.set(on, velX, velY, velZ, rotZ, duration)

    def set(self, on, velX, velY, velZ, rotZ, duration):
        self.on   = on              # (bool) motores activos 
        self.velX = velX            # (m/s)  velocidad lineal  deseada en eje X
        self.velY = velY            # (m/s)  velocidad lineal  deseada en eje Y
        self.velZ = velZ            # (m/s)  velocidad lineal  deseada en eje Z
        self.rotZ = rotZ            # (m/s)  velocidad angular deseada en eje Z
        self.duration = duration    # (s)    tiempo de expiracion del comando

    def off(self):
        self.set(False, 0, 0, 0, 0, None)

    def hover(self):
        self.set(True, 0, 0, 0, 0, None)

    def print_command(self):
        try:
            print(f"UAV command: ON: {self.on:.0f} velX: {self.velX:.1f} velY: {self.velY:.1f} velZ: {self.velZ:.1f} rotZ: {self.rotZ:.1f} duration: {self.duration:.1f}")
        except:
            print(f"UAV command: ON: {self.on:.0f} velX: {self.velX:.1f} velY: {self.velY:.1f} velZ: {self.velZ:.1f} rotZ: {self.rotZ:.1f} duration: None")