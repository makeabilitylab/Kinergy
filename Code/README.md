# Kinergy Codebase

### KinergyMainInterface.gh 

> This is the main window interface, including the entries for each kinectic unit. This Grasshopper file shows the entire strucutre for the user inteface.

### HumanUIforKinergy direcotry

> This folder contains custom Human UI code to support simulation and custom user controls in this project.

### KinergyClassLibrary directory

> Kinergy library contains namespace "Constraints", "Generator", "Geom", "Motion", "Utilities", and class "Movement". 
![Image of the namespace and classes in Kinergy](http://lianghe.me/research/kinergy/Kniergy1-1.png)

> *The class diagram is under development on https://processon.com/diagraming/5eafd5496376897466a3759f*


### KinergyUtilities directory

> This directory contains all operations needed in this project. All operations are implemented and used as Grasshopper batteries.

### MotionSolver directory

> This directory contains all the Grasshopper batteries for all kinetic units. The sub-directories indicate the knietic units. Each kinetic unit has multiple batteries for a complete end-to-end workflow.

### Resources directory

> This directory contains all the image files and 3D model files for the basic elemnet that are used in this project such as locking mechanisms.

### Old_Code directory

> This folder contains all the old code that is not used and reserved as backup of this project.


## Configuration for Developing and Lauching the Project
1. Copy and paste the subdirectory and fiels from the directory "Resources" to "Program Files\Rhino 6\Plug-ins\Grasshopper\Components\Resources\" (you have to create the directory "Resources" for the first time)
1. Copy and paste the compiled *.gha files under the "bin" directory for each project that is used in this project to "Program Files\Rhino 6\Plug-ins\Grasshopper\Components\" (replace the old ones if those files already exist)
1. Copy and paste "Kinergy.dll" (only need to copy and paste one file) to "Program Files\Rhino 6\Plug-ins\Grasshopper\Components\"
1. In GrasshopperDeveloperSetting, add "Code\HumanUIforKinergy\HumanUIforKinergy\bin\"