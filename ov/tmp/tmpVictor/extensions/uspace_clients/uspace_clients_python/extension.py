import omni.ext
import omni.ui as ui
import omni.isaac.ui.element_wrappers as isaac_ui_wrappers
import asyncio
import random
from omni.isaac.core.utils.stage import get_current_stage
import carb.events
import omni.kit.app as app

class NavsimUspaceClientsExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.create_vars()
        self.build_ui()
        

    def on_shutdown(self):
        self.window.destroy()

        for vertiport in self.available_vertiports:
            vertiport.GetAttribute("occupied").Set(False)

    
    def create_vars(self):
        self.amazon_new_nec_loop_constant = 15
        self.amazon_new_nec_loop_current = 15
        self.zara_new_nec_loop_constant = 25
        self.zara_new_nec_loop_current = 25

        self.amazon_nec_list = {}
        self.zara_nec_list = {}

        self.amazon_new_nec_back_counter_constant = 25
        self.zara_new_nec_back_counter_constant = 35

        self.stop_simulation_flag = True

        self.available_vertiports = []
        self.update_available_verts_loop = 5
        self.mutex_available_verts = asyncio.Event()

        self.NECESSITY_EVENT = carb.events.type_from_string("omni.uspace.clients.NECESSITY_EVENT")
        self.event_stream = app.get_app().get_message_bus_event_stream()


    def build_ui(self):
        
        self.window = ui.Window("NavSim - USpace clients", width=300, height=300, raster_policy=ui.RasterPolicy.NEVER)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=10, height=0):
                    ui.Spacer(height=10)
                    # Info
                    ui.Label("This screen shows information about uspace clients", alignment=ui.Alignment.CENTER)

                    ui.Spacer(height=5)

                    with ui.HStack():
                        ui.Button("START", clicked_fn=self.start_simulation)
                        ui.Button("STOP", clicked_fn=self.stop_simulation)

                    # Amazon client
                    self.amazon_client = ui.CollapsableFrame(title="Amazon", collapsed=False)
                    with self.amazon_client:
                        with ui.VStack(spacing=5, height=0):
                            self.amazon_new_nec_counter_label = ui.Label("New necessity: 15", alignment=ui.Alignment.CENTER)

                            # Vertiports
                            self.amazon_verts_collframe = ui.CollapsableFrame(title="Vertiports", collapsed=True)
                            with self.amazon_verts_collframe:
                                self.amazon_verts_label_container = ui.VStack()
                                # self.amazon_verts_label_counter = 0

                            # Necessities
                            self.amazon_nec_collframe = ui.CollapsableFrame(title="Necessities", collapsed=True)
                            with self.amazon_nec_collframe:
                                self.amazon_nec_label_container = ui.VStack()
                                self.amazon_nec_label_counter = 0


                    # Zara client
                    self.zara_client = ui.CollapsableFrame(title="Zara", collapsed=False)
                    with self.zara_client:
                        with ui.VStack(spacing=5, height=0):
                            self.zara_new_nec_counter_label = ui.Label("New necessity: 25", alignment=ui.Alignment.CENTER)

                            # Vertiports
                            self.zara_verts_collframe = ui.CollapsableFrame(title="Vertiports", collapsed=True)
                            with self.zara_verts_collframe:
                                self.zara_verts_label_container = ui.VStack()
                                # self.zara_verts_label_counter = 0

                            # Necessities
                            self.zara_nec_collframe = ui.CollapsableFrame(title="Necessities", collapsed=True)
                            with self.zara_nec_collframe:
                                self.zara_nec_label_container = ui.VStack()
                                self.zara_nec_label_counter = 0


    def start_simulation(self):
        if self.stop_simulation_flag:
            self.stop_simulation_flag = False
            self.mutex_available_verts.set()

            asyncio.ensure_future(self.amazon_new_nec_back_counter())
            asyncio.ensure_future(self.zara_new_nec_back_counter())
            asyncio.ensure_future(self.update_available_vertiports())


    def stop_simulation(self):
        if not self.stop_simulation_flag:
            self.stop_simulation_flag = True


    async def update_available_vertiports(self):
        while not self.stop_simulation_flag:
            self.amazon_verts_label_container.clear()
            self.zara_verts_label_container.clear()

            # -- Start mutex
            await self.mutex_available_verts.wait()
            self.mutex_available_verts.clear()

            self.available_vertiports = self.get_available_vertiports()

            for vertiport in self.available_vertiports:
                vert_name = vertiport.GetName()
                vert_pos = vertiport.GetAttribute("xformOp:translate").Get()
                vert_pos = (round(vert_pos[0], 2), round(vert_pos[1], 2), round(vert_pos[2], 2))
                vert_occupied = vertiport.GetAttribute("occupied").Get()

                text = f"{vert_name} -> Pos: {vert_pos}\t-\tOccupied: {vert_occupied}"
                self.add_vert(text)

            self.mutex_available_verts.set()
            # -- End mutex

            await asyncio.sleep(self.update_available_verts_loop)

            # Avoid exception when saving file while running simulation
            if not hasattr(self, "stop_simulation_flag"):
                break


    def get_available_vertiports(self, prim=None):
        available_vertiports = []

        # First iteration
        if prim is None:
            stage = get_current_stage()
            prim = stage.GetPseudoRoot()

            available_vertiports.clear()

        # Check current prim
        if prim.GetAttribute("available").IsValid() and prim.GetAttribute("available").Get():
            available_vertiports.append(prim)
        
        else:
            # Check prim's children
            for child in prim.GetAllChildren():
                # First check children
                if len(child.GetAllChildren()) > 0:
                    available_vertiports += self.get_available_vertiports(child)
                
                # Then check current child
                elif child.GetAttribute("available").IsValid() and child.GetAttribute("available").Get():
                    available_vertiports.append(prim)

        return available_vertiports
    

    def get_not_occupied_vertiports(self):
        not_occup_verts = []

        for i in range(len(self.available_vertiports)):
            if not self.available_vertiports[i].GetAttribute("occupied").Get():
                not_occup_verts.append((i, self.available_vertiports[i]))

        return not_occup_verts


    def add_vert(self, vertiport):
        # Amazon
        with self.amazon_verts_label_container:
            ui.Label(vertiport, alignment=ui.Alignment.CENTER)

        # Zara
        with self.zara_verts_label_container:
            ui.Label(vertiport, alignment=ui.Alignment.CENTER)


    async def amazon_new_nec_back_counter(self):
        while not self.stop_simulation_flag:
            await asyncio.sleep(1)

            # Avoid exception when saving file while running simulation
            if not hasattr(self, "stop_simulation_flag"):
                break

            self.amazon_new_nec_loop_current -= 1
            self.amazon_new_nec_counter_label.text = f"New necessity: {self.amazon_new_nec_loop_current}"

            if self.amazon_new_nec_loop_current == 0:
                # -- Start mutex
                await self.mutex_available_verts.wait()
                self.mutex_available_verts.clear()
                
                created, nec, key, origin, destination = self.create_nec("amazon")
                
                self.mutex_available_verts.set()
                # -- End mutex

                if created:
                    # Push event
                    payload = {"client": "amazon", "origin": str(origin.GetPrimPath()), "destination": str(destination.GetPrimPath())}
                    self.event_stream.push(self.NECESSITY_EVENT, payload=payload)

                    self.add_nec("amazon", nec)
                    asyncio.ensure_future(self.new_nec_back_counter("amazon", key))

                self.reset_nec_counter("amazon")

        
    async def zara_new_nec_back_counter(self):
        while not self.stop_simulation_flag:
            await asyncio.sleep(1)

            # Avoid exception when saving file while running simulation
            if not hasattr(self, "stop_simulation_flag"):
                break

            self.zara_new_nec_loop_current -= 1
            self.zara_new_nec_counter_label.text = f"New necessity: {self.zara_new_nec_loop_current}"

            if self.zara_new_nec_loop_current == 0:
                # -- Start mutex
                await self.mutex_available_verts.wait()
                self.mutex_available_verts.clear()
                
                created, nec, key, origin, destination = self.create_nec("zara")
                
                self.mutex_available_verts.set()
                # -- End mutex

                if created:
                    # Push event
                    payload = {"client": "zara", "origin": str(origin.GetPrimPath()), "destination": str(destination.GetPrimPath())}
                    self.event_stream.push(self.NECESSITY_EVENT, payload=payload)

                    self.add_nec("zara", nec)
                    asyncio.ensure_future(self.new_nec_back_counter("zara", key))

                self.reset_nec_counter("zara")


    def create_nec(self, client):
        not_occup_verts = self.get_not_occupied_vertiports()

        if len(not_occup_verts) > 1:
            start = random.randint(0, len(not_occup_verts)-1)
            end = random.randint(0, len(not_occup_verts)-1)
            while end == start:
                end = random.randint(0, len(not_occup_verts)-1)

            start_vert_index = not_occup_verts[start][0]
            start_vert = not_occup_verts[start][1]

            end_vert_index = not_occup_verts[end][0]
            end_vert = not_occup_verts[end][1]

            self.available_vertiports[start_vert_index].GetAttribute("occupied").Set(True)
            self.available_vertiports[end_vert_index].GetAttribute("occupied").Set(True)

            match client:
                case "amazon":
                    necessity = f"Nec_{self.amazon_nec_label_counter}: {start_vert.GetName()} -> {end_vert.GetName()}\t-\tRelevance = HIGH"
                    key = f"Nec_{self.amazon_nec_label_counter}"
                    self.amazon_nec_list[key] = {"vertiports": (start_vert, end_vert), "label": necessity}
                    return True, necessity, key, start_vert, end_vert
                
                case "zara":
                    necessity = f"Nec_{self.zara_nec_label_counter}: {start_vert.GetName()} -> {end_vert.GetName()}\t-\tRelevance = HIGH"
                    key = f"Nec_{self.zara_nec_label_counter}"
                    self.zara_nec_list[key] = {"vertiports": (start_vert, end_vert), "label": necessity}
                    return True, necessity, key, start_vert, end_vert
                
                case _:
                    pass

        return False, None, None, None, None

            
    def add_nec(self, client, necessity):
        match client:
            case "amazon":
                self.amazon_nec_label_counter += 1
                with self.amazon_nec_label_container:
                    ui.Label(necessity, alignment=ui.Alignment.CENTER)

            case "zara":
                self.zara_nec_label_counter += 1
                with self.zara_nec_label_container:
                    ui.Label(necessity, alignment=ui.Alignment.CENTER)

            case _:
                pass

    
    def reset_nec_counter(self, client):
        match client:
            case "amazon":
                self.amazon_new_nec_loop_current = self.amazon_new_nec_loop_constant
                var = random.randint(-5, 5)
                self.amazon_new_nec_loop_current += var
                self.amazon_new_nec_counter_label.text = f"New necessity: {self.amazon_new_nec_loop_current}"

            case "zara":
                self.zara_new_nec_loop_current = self.zara_new_nec_loop_constant
                var = random.randint(-5, 5)
                self.zara_new_nec_loop_current += var
                self.zara_new_nec_counter_label.text = f"New necessity: {self.zara_new_nec_loop_current}"

            case _:
                pass


    async def new_nec_back_counter(self, client, key):
        match client:
            case "amazon":
                wait = self.amazon_new_nec_back_counter_constant + random.randint(-3, 3)
                await asyncio.sleep(wait)

                # Reset vertiports state
                self.amazon_nec_list[key]["vertiports"][0].GetAttribute("occupied").Set(False)
                self.amazon_nec_list[key]["vertiports"][1].GetAttribute("occupied").Set(False)

                del self.amazon_nec_list[key]

                self.amazon_nec_label_container.clear()

                # Repaint necessities
                for nec in self.amazon_nec_list.values():
                    self.add_nec("amazon", nec["label"])


            case "zara":
                wait = self.zara_new_nec_back_counter_constant + random.randint(-3, 3)
                await asyncio.sleep(wait)

                # Reset vertiports state
                self.zara_nec_list[key]["vertiports"][0].GetAttribute("occupied").Set(False)
                self.zara_nec_list[key]["vertiports"][1].GetAttribute("occupied").Set(False)

                del self.zara_nec_list[key]

                self.zara_nec_label_container.clear()

                # Repaint necessities
                for nec in self.zara_nec_list.values():
                    self.add_nec("zara", nec["label"])

            case _:
                pass