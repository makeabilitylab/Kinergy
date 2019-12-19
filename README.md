# 3D Printing Energy - Getting Familiar with the Codebase & Mini Task (Due 1/2)

To help all members who will contribute in the development of our design tool, I created a codebase with two simple features in a custom Rhino plugin example. After learning the codebase, you will understand the architecture of Rhino plugin development and you will add another feature to the current plugin interface.

## Example Rhino Plugin 

The provided codebase is developed with VS studio 2015 and Rhino 6 on Windows. These two software can be downloaded and installed from this [Google Drive folder](https://drive.google.com/drive/folders/1wYBMWHzkhhNNpjThSnpCZC947AjmC39U?usp=sharing) (Visual Studio 2017 is uploaded).

### Troubleshooting for Development Environment
**UPDATE:** Since we are using Rhino 6 (SDK), I have updated VS from 2015 to 2017. Please install VS 2017 professional (your choice of uninstalling VS 2015).

If you cannot run the code successfully, check the following project settings before running the project:
1. After the project is loaded in VS 2017, go to "Project -> xxxx properties...", check "Start external program" in "Debug" tab is located to Rhino 6 execution file (the default path is *C:\Program Files\Rhino 6\System\Rhino.exe*);
2. Check "Target framework" is set to ".NET Framework 4.5.2" in "Application" tab;
3. Go to "Solution Explorer" side panel, expand "References", remove "RhinoCommon" and "Rhino_DotNet" references (they should have a small warning icon). Right-click "References" and select "Add reference...", add RhinoCommon.dll and Rhino_DotNet.dll (for Rhino 6) in *C:\Program Files\Rhino 6\System\RhinoCommon.dll* and *C:\Program Files\Rhino 6\System\Rhino_DotNet.dll* in "Browse" tab, and select both of them and hit "OK";
4. Set "Copy Local" property "False" for both new references (you can find the property by clicking the reference and look for "Copy Local" in the property list at the bottom panel); 
5. Clean and rebuild the solution and it should be ready to go.

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

### Pre-task Exercise
Before we jump into the real task, we can add a new button to the plugin and print out "Hello World" in Rhino by clicking that button :)

1. Go to *3DPEnergyPanel.cs* and drag and drop a button control on the graphical interface;
2. Double-click the new button to add a function for the button-click event;
3. In the function, call a new function called *"PrintHelloWorld"*. This function is a member function of the *class Controller* object *controller*.
4. Go to *Controller* class and add the definition of the member function *PrintHelloWorld*;
5. In *"PrintHelloWorld"* function, call *rhinoModel's* member function *"PrintHelloWorld"* to execute the printout;
6. Refer to Rhinocommon API to figure out how to print a line "Hello World" in Rhino's command window in the body of *class RhinoModel's* member function *"PrintHelloWorld"* 

Now you understand how to add a new feature to the plugin and how the end user interface communicates with the backend implementation. Let's move on to the main task.

### Voxel/Voxelization: 
A voxel represents a value on a regular grid in three-dimensional space. More information can be found on [this page](https://en.wikipedia.org/wiki/Voxel). 

### Model voxelization workflow:
1. Select a 3D model in the editting environment;
2. Click a button in the plugin interface (*action: add a new button to the interface*);
3. Convert the selected model from a [B-rep](https://en.wikipedia.org/wiki/Boundary_representation) to a [mesh](https://en.wikipedia.org/wiki/Mesh) (*action: you might need to refer to Rhinocommon APIs to convert the model to a mesh*);
4. Find the triangles in the converted mesh (*action: you might need to triangulate the mesh using Rhinocommon APIs*);
5. Following the [Ray-Triangle Intersection](https://courses.cs.washington.edu/courses/csep557/10au/lectures/triangle_intersection.pdf) algorithm (*action: implement the algorithm in C#*);
6. Visualize each voxels (*action: your pick of visualization form, e.g., points, boxes, spheres, etc.*)

### Q&A:
Please try at least one solution before ask any questions. This is a learning-by-practice process and you will gain benefits from trials. Otherwise, Liang can help with learning the codebase, software setups, and explaining the workflow of the task. Liang plans to give a quick demo of the codebase to all students who will involve in the development.
