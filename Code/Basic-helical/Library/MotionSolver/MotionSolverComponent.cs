using System;
using System.Collections.Generic;
using Kinergy.Motion;
using Grasshopper.Kernel;
using Rhino.Geometry;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace MotionSolver
{
    public class MotionSolverComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public MotionSolverComponent()
          : base("MotionSolver", "MS",
              "A general motion solver",
              "Motion", "")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Start", "Start", "Whether to start calculating", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Type", "Type", "Type of motion", GH_ParamAccess.item);
            pManager.AddBrepParameter("Brep", "Brep", "Brep To Place Box", GH_ParamAccess.item);
            pManager.AddVectorParameter("Vector3d", "Vec", "Direction of Movement", GH_ParamAccess.item);
            pManager.AddNumberParameter("Parameter", "Param", "Parameter of motion", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Lock", "Lock", "Whether to add lock", GH_ParamAccess.item);
            
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("Model", "Model", "processed model", GH_ParamAccess.list);
            pManager.AddBrepParameter("Structure", "Str", "added kinetic object", GH_ParamAccess.list);
            pManager.AddBrepParameter("Lock", "Lock", "lock", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool start=false;
            int type = 0;
            Brep model=null;
            Vector3d direction=Vector3d.Unset;
            List<double> parameters = new List<double>();
            bool addLock = false;
            if (!DA.GetData(0, ref start)) { return; }
            if (start == false) { return; }
            if (!DA.GetData(1, ref type)) { return; }
            if (!DA.GetData(2, ref model)) { return; }
            if (!DA.GetData(3, ref direction)) { return; }
            if (!DA.GetDataList(4, parameters)) { return; }
            DA.GetData(5, ref addLock);
            switch(type)
            {
                case 1: 
                    {//case 1 is helical spring

                        HelicalSpring motion = new HelicalSpring(model, parameters[0], parameters[1], direction, addLock);
                        motion.Process();
                        DA.SetDataList(0, motion.GetModel());
                        DA.SetData(1, motion.GetSpring());
                        DA.SetDataList(2, motion.GetLock());
                        break; 
                    }
                case 2: { break; }
                case 3: { break; }
                case 4: { break; }
                case 5: { break; }
                case 6: { break; }
            }
            
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("cedda6c8-56df-4c0c-93ee-0236934e668d"); }
        }
    }
}
