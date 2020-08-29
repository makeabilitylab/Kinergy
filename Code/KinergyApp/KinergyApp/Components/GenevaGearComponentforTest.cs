using System;
using System.Collections.Generic;
using Kinergy.Geom;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace HumanUIforKinergy.Components
{
    public class GenevaGearComponentforTest : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the GenevaGearComponentforTest class.
        /// </summary>
        public GenevaGearComponentforTest()
          : base("GenevaGearComponentforTest", "GG",
              "Description",
              "Kinergy", "Components")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("DriverCenterPoint", "DCP", "The center point of driver wheel.", GH_ParamAccess.item);
            pManager.AddVectorParameter("Normal", "N", "The normal direction of both wheels", GH_ParamAccess.item);
            pManager.AddNumberParameter("DriverWheelRadius", "DWR", "The wheel radius, or the distance between roller center and wheel center", GH_ParamAccess.item);
            pManager.AddNumberParameter("RollerRadius", "RR", "The radius of roller", GH_ParamAccess.item);
            pManager.AddVectorParameter("Centerlink direction", "CD", "The link direction between driver and driven wheel." +
                "Please note that the position of driven wheel is not set by a given point, but a direction and other parameter, " +
                "since the distance between wheels is determined by other params.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Number of slots", "N", "Number of slots in driven wheel.SHould be no less than 3", GH_ParamAccess.item);
            pManager.AddNumberParameter("Initial degree of driver", "ID", "initial degree of driver wheel.In degrees, could be any value", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("DriverWheel", "Driver", "The generated driver wheel object.As entity", GH_ParamAccess.item);
            pManager.AddGenericParameter("DrivenWheel", "Driven", "The generated driven wheel object.As entity", GH_ParamAccess.item);
            pManager.AddBrepParameter("DriverModel", "DriverM", "The brep model of driver wheel.", GH_ParamAccess.item);
            pManager.AddBrepParameter("DrivenModel", "DrivenM", "The brep model of driven wheel.", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Point3d cp = Point3d.Unset;
            if (!DA.GetData(0, ref cp)) { return; }
            Vector3d normal = Vector3d.Unset;
            if (!DA.GetData(1, ref normal)) { return; }
            double r1 = 0, r2 = 0;
            if (!DA.GetData(2, ref r1)) { return; }
            if (!DA.GetData(3, ref r2)) { return; }
            Vector3d centerLinkDirection = Vector3d.Unset;
            if (!DA.GetData(4, ref centerLinkDirection)) { return; }
            int n = 0;
            if (!DA.GetData(5, ref n)) { return; }
            if(n<3)
            {
                throw new Exception("number of slots should never be less than 3!");
            }
            double angle = 0;
            if (!DA.GetData(6, ref angle)) { return; }
            GenevaDrivingWheel driver = new GenevaDrivingWheel(cp, normal, r1, r2);
            GenevaDrivenWheel driven = driver.ConstructDrivenWheel(centerLinkDirection, n);
            driver.SetAngle(angle * Math.PI / 180);
            DA.SetData(0, driver);
            DA.SetData(1, driven);
            DA.SetData(2, driver.GetModelinWorldCoordinate());
            DA.SetData(3, driven.GetModelinWorldCoordinate());
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
            get { return new Guid("802189e5-8f7e-4c41-b305-4417fd76db1d"); }
        }
    }
}