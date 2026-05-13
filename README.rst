Example for Illixr. Should be able to add it as an example under /zephyr/samples.

To run, go to your zephyr directory, source the appropriate environment, and build.

The command used is: ``west build -p -b spike_riscv64 samples/illixr_working/ -DYAML_FILE=profiles/default_new.yaml``
You can then proceed to run Spike normally.

After that, can run spike accordingly.


Adding Plugins:
To add plugins, you can add them to the YAML file. For example, if you want to add the "example_plugin", you would add it to the "plugins" section of the YAML file like this:
plugins: "example_plugin"

Writing Plugins:
To write a plugin, you need to create a new C++ file in the "plugins" directory. The file should include the necessary headers and define the plugin class. 
For example, if you wanted to create a plugin called "example_plugin", you would create a file named "example_plugin.cpp" and implement the plugin class within that file.
You should also update the CMakeList.txt in the plugin folder itself. There is no change on the main CMakeList.txt file.

Each plugin class should inherit from the threadloop class and implement the necessary methods for initialization, execution, and cleanup. You can refer to existing plugins in the "plugins" directory for examples of how to structure your plugin.

The data for the offline_cam, offline_imu are not uploaded on github. 
I essentially used embed_euroc_data.py to convert the data into a C++ header file and then included that header file in the respective plugins through CMakeLists.txt. You can do the same for your own data if you want to add more plugins that require data.

