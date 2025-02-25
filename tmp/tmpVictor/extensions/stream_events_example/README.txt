This extension has been created to have a better understanding of how omniverse events work.

You have two main files:
    - /stream_events_example/stream_events_example_python/extension.py
        - Here is where the extension is built.

    - /stream_events_example/stream_events_example_python/usd_scene/scene_example.usd
        - This is the scene you must open before proceeding with any button from the extension.

Once you open the given scene you will see that there are two objects, a cube and a cone.
Each one has a behavior script attached. These scripts are within the same folder of the scene.

If you have a look at the extension UI you will see a note to read this file and three buttons.
    - COMMON EVENT
    - CUBE EVENT
    - CONE EVENT
In order these buttons to work properly the simulation must be running.

From here, you can have a look at the scripts to know what is happening at low level.