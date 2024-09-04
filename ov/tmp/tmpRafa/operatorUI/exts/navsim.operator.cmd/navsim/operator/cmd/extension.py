import omni.ext
import omni.ui as ui


# Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
def some_public_function(x: int):
    print("[navsim.operator.cmd] some_public_function was called with x: ", x)
    return x ** x


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class NavsimOperatorCmdExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[navsim.operator.cmd] navsim operator cmd startup")

        self.velX = 0

        self._window = ui.Window("Operator CMD", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                label = ui.Label("")

                LvelX = ui.FloatField(tooltip="velX parameter")


                def on_click():
                    self.velX = 1
                    label.text = f"velX: {self.velX}"
                    print (f"velX: {self.velX}")




                with ui.HStack():
                    ui.Button("Send CMD", clicked_fn = on_click)

    def on_shutdown(self):
        print("[navsim.operator.cmd] navsim operator cmd shutdown")
