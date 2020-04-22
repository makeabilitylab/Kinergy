using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy.Motion;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace MotionSimulator
{
    public class MotionSimulatorComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public MotionSimulatorComponent()
          : base("MotionSimulator", "Nickname",
              "Simulate and animate motions",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Start", "S", "Turn on the simulation process", GH_ParamAccess.item);
            pManager.AddScriptVariableParameter("Motion", "M", "Motion to simulate", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Load", "L", "Load the motion", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("Models", "M", "Brep models of all generated entities in this motion solver", GH_ParamAccess.list);
            pManager.AddGenericParameter("Entities", "E", "List of entities generated in motion solver. " +
                "Please use EntityReader to classify the entities and extract their brep models.", GH_ParamAccess.list);
            pManager.AddGenericParameter("MotionInstance", "M", "Motion instance generated in motion solver." +
                " U could import it to Simulation cell. ", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool start = false;
            Motion m=null;
            bool load = false;
            if (!DA.GetData(0, ref start)) { return; }
            if (start == false) { return; }
            if (!DA.GetData(1, ref m)) { return; }
            if (!DA.GetData(2, ref load)) { return; }

            m.SetLoad(load);
            DA.SetDataList(0, m.GetModel());
            DA.SetData(1, m.EntityList);
            DA.SetData(2, m);

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
            get { return new Guid("0e0c11e6-e194-4d6a-9b20-afd2bb9abb2a"); }
        }
    }
}
