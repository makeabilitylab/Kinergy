using System;
using System.Collections.Generic;
using Kinergy.Geom;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace KinergyUtilities
{
    public class AdjustSpringParameter : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the AdjustSpringParameter class.
        /// </summary>
        public AdjustSpringParameter()
          : base("AdjustSpringParameter", "",
              "",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("Spring", "S", "The spring entity to adjust", GH_ParamAccess.item);
            pManager.AddNumberParameter("RadiusAdjustment", "R", "Adjustment value of spring radius", GH_ParamAccess.item);
            //pManager.AddNumberParameter("RoundNumberAdjustment", "N", "Adjustment value of spring round number", GH_ParamAccess.item);
            pManager.AddIntegerParameter("RoundNumberAdjustment", "N", "Adjustment value of spring round number", GH_ParamAccess.item);
            pManager.AddNumberParameter("WireRadiusAdjustment", "W", "Adjustment value of wire radius", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Spring", "S", "the adjusted spring entity", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Spring s = null;
            double r = 0;
            double w = 0;
            int n = 0;
            if (!DA.GetData(0, ref s)) { return; }
            if (!DA.GetData(1, ref r)) { return; }
            if (!DA.GetData(2, ref n)) { return; }
            if (!DA.GetData(3, ref w)) { return; }
            s.AdjustParameter(r, n, w);
            DA.SetData(0, s);
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
            get { return new Guid("11d78996-fc42-4a5f-8f21-5b62d9dbcc10"); }
        }
    }
}