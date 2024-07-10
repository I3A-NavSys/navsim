# Base libraries
import omni.ext
import omni.ui as ui

# Libraries for ploting graphs
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
import numpy as np
import asyncio

# -- IMPORTANT ----------------------------------------------------------------------------------------------------------------------------------------------------

# To install libraries as in python -> omni.kit.pipapi.install("library")
    # To plot matplotlib graphs within omniverse it is required to use a GUI matplotlib backend
    # As you can see in line 7, I use Qt5Agg backend. For this to work properly you need to install PyQt5 package
    # Then, although you already installed in you pc, you will need to run omni.kit.pipapi.install("PyQt5")
    # In order to do this, open the script editor extension within omniverse, write the following code and run the file
        # Code:
        # import omni.kit.pipapi
        # omni.kit.pipapi.install("PyQt5")
    # I assume that you have already installed matplotlib, if not, you can install it using the code above and changing the package name.
    # Or you can also do it adding the path to your local pc python site-packages within the "python.analysis.extraPaths" parameter inside settings.json
        # The path is something like this: C:/Users/Victor/AppData/Local/Programs/Python/Python312/Lib/site-packages

# ------------------------------------------------------------------------------------------------------------------------------------------------------------------

# Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
def some_public_function(x: int):
    print("[victor.parallel.graphs] some_public_function was called with x: ", x)
    return x ** x

def generateGraph():
    # Datos
    x = np.linspace(-5, 5, 100)
    y = np.linspace(-5, 5, 100)
    x, y = np.meshgrid(x, y)
    z = np.sin(np.sqrt(x**2 + y**2))

    # Figura
    figure = plt.figure()

    # Grafica
    graph = figure.add_subplot(111, projection="3d")

    # Añadimos contenido a la grafica
    graph.plot_surface(x, y, z)

    # Limites
        # Si no se indica, se adaptan

    # Ejes
    graph.set_xlabel("EJE X")
    graph.set_ylabel("EJE Y")
    graph.set_zlabel("EJE Z")

    # Titulo
    graph.set_title("mathFunction")

    # Mostrar grafica
    plt.show(block=False)

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class VictorParallelGraphsExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[victor.parallel.graphs] victor parallel graphs startup")

        self._count = 0
        
        self._coroutine_event = asyncio.Event()
        self._automated_count = 0

        self._window = ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                with ui.HStack():
                    with ui.VStack():
                        manual_label = ui.Label("manual_count: 0")
                        manual_label.alignment = ui.Alignment.CENTER

                        def on_click():
                            self._count += 1
                            manual_label.text = f"manual_count: {self._count}"

                        def on_reset():
                            self._count = 0
                            manual_label.text = "manual_count: 0"

                        with ui.HStack():
                            ui.Button("Add", clicked_fn=on_click)
                            ui.Button("Reset", clicked_fn=on_reset)

                    with ui.VStack():
                        automated_counter = ui.Label("automated_count: 0")
                        automated_counter.alignment = ui.Alignment.CENTER

                        def start_automated_counter():
                            self._coroutine_event.set()

                        def stop_automated_counter():
                            self._coroutine_event.clear()

                        async def infinite_counter():
                            while True:
                                await self._coroutine_event.wait()
                                
                                automated_counter.text = f"automated_count: {self._automated_count}"
                                self._automated_count += 1
                                
                                await asyncio.sleep(1)

                        asyncio.ensure_future(infinite_counter())

                        with ui.HStack():
                            ui.Button("Start", clicked_fn=start_automated_counter)
                            ui.Button("Stop", clicked_fn=stop_automated_counter)

                with ui.HStack():
                    ui.Button("GRAPH", clicked_fn=generateGraph)


    def on_shutdown(self):
        print("[victor.parallel.graphs] victor parallel graphs shutdown")