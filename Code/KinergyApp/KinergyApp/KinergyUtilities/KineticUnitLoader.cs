using System;
using System.Collections.Generic;
using Kinergy.KineticUnit;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace KinergyUtilities
{
    public class KineticUnitLoader : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MotionLoader class.
        /// </summary>
        public KineticUnitLoader()
          : base("KineticUnitLoader", "KULoader",
              "Load the given Kinetic Unit.",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("Motion", "M", "The motion to load", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Start", "S", "Start simulating", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Motion", "M", "The loaded motion", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            KineticUnit m = null;
            
            bool start = false;
            
            if (!DA.GetData(0, ref m)) { return; }
            if (m == null)
            { return; }
            if (!DA.GetData(1, ref start))
            { DA.SetData(0, m); return; }
            if(start==false)
            { DA.SetData(0, m); return; }
            if (m.Loaded == true)
            { DA.SetData(0, m); return;}
            m.LoadKineticUnit();
            DA.SetData(0, m);
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("3d93b1c0-ac8a-4e49-a540-09cc4fdf645c"); }
        }
    }
}