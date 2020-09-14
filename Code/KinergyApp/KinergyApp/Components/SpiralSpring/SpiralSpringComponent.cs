using System;
using System.Collections.Generic;
using Kinergy.Geom;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace HumanUIforKinergy.Components.SpiralSpring
{
    public class SpiralSpringComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SpiralSpringComponent class.
        /// </summary>
        public SpiralSpringComponent()
          : base("SpiralSpringComponent", "Spiral",
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
            
            pManager.AddNumberParameter("Minimal Radius of Spiral Spring", "Rmin", "The minimal value of Spiral Spring radius.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Maximum Radius of Spiral Spring", "Rmax", "The maximal value of Spiral Spring radius.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Thickness of Spiral", "T", "The thickness of spiral.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Width of Spiral section", "W", "The width of spiral section.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Number of spiral rounds", "N", "The number of spiral rounds,as integer", GH_ParamAccess.item);
            pManager.AddNumberParameter("Loaded angle", "LA", "The initial loaded angle of spiral spring, in degrees", GH_ParamAccess.item);
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
            
            double rmin = 0, rmax = 0, t = 0,w=0,a=0;
            int n = 0;
            if (!DA.GetData(1, ref rmin)) { return; }
            if (!DA.GetData(2, ref rmax)) { return; }
            if (!DA.GetData(3, ref t)) { return; }
            if (!DA.GetData(4, ref w)) { return; }
            if (!DA.GetData(5, ref n)) { return; }
            if (!DA.GetData(6, ref a)) { return; }
            a = Math.PI * a / 180;
            Spiral c = new Spiral(p.Origin, p.Normal, rmax, rmin,n,t,w , a);
            
            DA.SetData(0, c);
            
            DA.SetData(1,  c.GetModelinWorldCoordinate());
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
            get { return new Guid("d6b523fa-288e-45ab-8cfc-5ac22290377a"); }
        }
    }
}