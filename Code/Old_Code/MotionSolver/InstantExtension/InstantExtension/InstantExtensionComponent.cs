using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy.Motion;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace InstExtension
{
    public class InstantExtensionComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public InstantExtensionComponent()
          : base("InstantExtension", "IE",
              "Solve instant extension motion",
              "Kinergy", "InstantExtension")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Start", "S", "Whether to start calculating", GH_ParamAccess.item);
            pManager.AddBrepParameter("Model", "M", "Model to process", GH_ParamAccess.item);
            pManager.AddVectorParameter("SpringDirection", "SD", "Direction of spring", GH_ParamAccess.item);
            pManager.AddBooleanParameter("CurvedModel", "CM", "If the model should be calculated as curved shape", GH_ParamAccess.item);
            pManager.AddNumberParameter("Energy", "E", "Energy of motion", GH_ParamAccess.item);
            pManager.AddNumberParameter("Distance", "D", "Proportion of spring that's able to be compressed", GH_ParamAccess.item);
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
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
            Brep model = null;
            Vector3d direction = Vector3d.Unset;
            double energy=0;
            double distance=0;
            bool curved = false;
            if (!DA.GetData(0, ref start)) { return; }
            if (start == false) { return; }
            
            if (!DA.GetData(3, ref curved)) { return; }
            if (!DA.GetData(4, ref energy)) { return; }
            if (!DA.GetData(5, ref distance)) { return; }
            if(curved)
            {
                if (!DA.GetData(1, ref model)) { return; }
                InstantExtension motion = new InstantExtension(model,energy, distance);
                DA.SetData(0, motion);
            }
            else
            {
                if (!DA.GetData(1, ref model)) { return; }
                if (!DA.GetData(2, ref direction)) { return; }
                InstantExtension motion = new InstantExtension(model, direction, energy, distance);
                DA.SetData(0, motion);
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
            get { return new Guid("da709922-b416-496f-b555-4a735eb0fe36"); }
        }
    }
}
