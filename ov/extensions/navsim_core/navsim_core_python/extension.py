import omni.ext
import os
import platform

from pathlib import Path

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class NavsimCore(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[navsim_core] navsim_core startup")

        # Get current file path
        src = Path(__file__)

        # Move the source path five levels up to the root directory
        for _ in range(5):
            src = src.parent
        
        # Get destination file path
        dst = Path(os.path.expandvars(r"%APPDATA%\..\Local\ov\pkg\isaac-sim-4.1.0"))

        # Create the symlink if it doesn't already exist
        if dst.exists():
            try:
                os.symlink(src.as_posix(), dst.as_posix(), target_is_directory=True)
                print(f"Enlace simbólico creado: {src} -> {dst}")
            except OSError as e:
                print(f"Error al crear el enlace simbólico: {e}")
        else:
            print(f"El enlace simbólico ya existe: {dst}")

    def on_shutdown(self):
        print("[navsim_core] navsim_core shutdown")
