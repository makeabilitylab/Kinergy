# 3D Printing Energy - Getting Familiar with the Codebase & Mini Task (Due 12/22)

To help all members who will contribute in the development of our design tool, I created a codebase with two simple features in a custom Rhino plugin example. After learning the codebase, you will understand the architecture of Rhino plugin development and you will add another feature to the current plugin interface.

## Example Rhino Plugin 

The provided codebase is developed with VS studio 2015 and Rhino 6 on Windows. These two software can be downloaded and installed from this [Google Drive folder](https://drive.google.com/drive/folders/1wYBMWHzkhhNNpjThSnpCZC947AjmC39U?usp=sharing).

### The architecture of the codebase:
- *PluginBarCommand.cs*/*PluginBarDialog.cs*/*PluginBarPlugIn.cs*: They are related to the initialization and registration of the plugin
- *UI folder*: It contains all user interface source files. For example, *ProcessingWarningWindow.cs* is the code-behind of the pop-up warning window and the *ProcessingWarningWindow.Designer.cs* is the design configuration file of this interface.
- *Controller.cs*: It bridges user input on the plugin interface and the underlying Rhino command executions. It defines a set of functions.
- *RhinoModel.cs*: It executes all commands from the interface using [Rhinocommon APIs](https://developer.rhino3d.com/guides/#rhinocommon).

### The example features:
- *Add a random shape in the model editing environment*: Click *Add a shape* button, a 3D shape (sphere or box) with a random color and size would be added to the 3D scene in Rhino.
- *Export as stl*: Select an object first in the scene. Then, click "Export as stl" button, the selected 3D shape would be saved as a STL file named *output.stl* in "./bin/" directory.

### Run the code:
1. Load the project in Visual Studio 2015 by double-clicking "XXX.sln" file in the code directory;
2. Right click the project name (with a green-boarded C# icon on the left) in the tree structure of the project on the side panel in VS, select "Properties";
3. Go to "Debug" tab and add "Start external program" by entering the location of Rhino 6 execution file. If you install Rhino 6 following the default steps, it should locate in: *C:\Program Files\Rhino 6\System\Rhino.exe*;
4. *Start without debugging (or `ctrl+F5`)* to run the code. Rhino 6 will automatically launch;
5. In Rhino, go to *"Tools -> Options -> Plug-in"* tab, click *Install...* button, locate "./bin/" directory and select *XXX.rhp* plugin file, then the custom plugin is loaded in Rhino successfully;
6. You can continue to use the features provided by the plugin.

## Mini Task: Voxelize the Selected Model (Due: 12/22)

This task aims to help you walkthrough the codebase architecture and gain more experience developing a feature for a custom Rhino plugin in C# from end to end. The algorithm for voxelization and required steps are provided below. You are free to raise any question as you learn more about the codebase and work on this task.

### Work on your own solution:
**Please fork a new branch from the *master* branch and implement your own solution in your branch. Don't publish your branch before 12/22.**

### Model voxelization workflow:
1. Select a 3D model in the editting environment;
2. Click a button in the plugin interface (*action: add a new button to the interface*);
3. Convert the selected model from a [B-rep](https://en.wikipedia.org/wiki/Boundary_representation) to a [mesh](https://en.wikipedia.org/wiki/Mesh) (*action: you might need to refer to Rhinocommon APIs to convert the model to a mesh*);
4. Find the triangles in the converted mesh (*action: you might need to triangulate the mesh using Rhinocommon APIs*);
5. Following the [Ray-Triangle Intersection](https://courses.cs.washington.edu/courses/csep557/10a
u/lectures/triangle_intersection.pdf) algorithm (*action: implement the algorithm in C#*);
6. Visualize each voxels (*action: your pick of visualization form, e.g., points, boxes, spheres, etc.*)

### Q&A:
Please try at least one solution before ask any questions. This is a learning-by-practice process and you will gain benefits from trials. Otherwise, Liang can help with learning the codebase, software setups, and explaining the workflow of the task. Liang plan to give a quick demo of the codebase to all students who will involve in the development.
