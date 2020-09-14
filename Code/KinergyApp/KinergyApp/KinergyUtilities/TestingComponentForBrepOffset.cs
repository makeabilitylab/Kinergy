using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace HumanUIforKinergy.KinergyUtilities
{
    public class TestingComponentForBrepOffset : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the TestingComponentForBrepOffset class.
        /// </summary>
        public TestingComponentForBrepOffset()
          : base("TestingComponentForBrepOffset", "Nickname",
              "Description",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "The brep model to offset", GH_ParamAccess.item);
            pManager.AddNumberParameter("Distance", "D", "The distance to offset", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("Output", "O", "Offset result", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep input = null;
            double distance = 0;
            if (!DA.GetData(0, ref input))
                return;
            if (!DA.GetData(1, ref distance))
                return;
            var output = Brep.CreateOffsetBrep(input, distance, true, true, Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out _);
            DA.SetData(0, output[0]);
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
            get { return new Guid("8df2b7ea-a4e5-4f73-857a-62ac1c3867c8"); }
        }
    }
}