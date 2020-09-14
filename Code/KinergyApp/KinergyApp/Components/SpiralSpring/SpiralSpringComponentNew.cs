using System;
using System.Collections.Generic;
using Kinergy.Geom;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace HumanUIforKinergy.Components.SpiralSpring
{
    public class SpiralSpringComponentNew : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SpiralSpringComponentNew class.
        /// </summary>
        public SpiralSpringComponentNew()
          : base("SpiralSpringComponentNew", "Nickname",
              "Testing cell for spiral spring",
              "Kinergy", "Components")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Base Plane", "P", "The base plane for Spiral Spring. The plane origin would become the center of Spiral Spring.", GH_ParamAccess.item);

            pManager.AddNumberParameter("Maximum Radius of Spiral Spring", "Rmax", "The maximal value of Spiral Spring radius.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Energy", "E", "The thickness of spiral.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Displacement", "D", "The width of spiral section.", GH_ParamAccess.item);
            
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Spiral Spring", "S", "The spiral spring object", GH_ParamAccess.item);
            pManager.AddBrepParameter("Models", "M", "The brep models of spiral", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Plane p = Plane.Unset;
            if (!DA.GetData(0, ref p)) { return; }

            double rmax = 0;int e = 0, d = 0;
            if (!DA.GetData(1, ref rmax)) { return; }
            if (!DA.GetData(2, ref e)) { return; }
            if (!DA.GetData(3, ref d)) { return; }
            Spiral c = new Spiral(p.Normal, p.Origin, rmax, d*Math.PI/180, e);

            DA.SetData(0, c);

            DA.SetData(1, c.GetModelinWorldCoordinate());
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
            get { return new Guid("6fae5be4-9a97-423b-b685-e32afbae4333"); }
        }
    }
}