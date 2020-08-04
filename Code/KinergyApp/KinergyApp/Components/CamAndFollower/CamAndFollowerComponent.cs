using System;
using System.Collections.Generic;
using Kinergy.Geom;
using Kinergy.KineticUnit;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Kinergy.Components.CamAndFollower
{
    public class CamAndFollowerComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CamAndFollower class.
        /// </summary>
        public CamAndFollowerComponent()
          : base("CamAndFollower", "C&F",
              "Testing cell for cam and follower",
              "Kinergy", "Components")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Base Plane", "P", "The base plane for cam and follower. The plane origin would become the center of cam.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Cam Type", "T1", "The type of cam,1 for ellipse and 2 for snail", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Follower Type", "T2", "The type of follower. 1 for vertical and 2 for oscillating", GH_ParamAccess.item);
            pManager.AddPointParameter("First point", "P1", "The first given point position for follower. It would be the near point for vertical follower and the connecting position for oscillating follower", GH_ParamAccess.item);
            pManager.AddPointParameter("Second point", "P2", "The second given point position for follower. It would be the far end point for vertical follower and the end point for oscillating follower", GH_ParamAccess.item);
            pManager.AddNumberParameter("Minimal Radius of Cam", "Rmin", "The minimal value of cam radius.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Maximum Radius of Cam", "Rmax", "The maximal value of cam radius.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Rod Radius of Follower", "Rrod", "The radius of Follower rod.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Initial position of Cam", "IP", "The initial position of cam, in degree 0-360.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Cam", "Cam", "The cam object", GH_ParamAccess.item);
            pManager.AddGenericParameter("Follower", "F", "The follower object", GH_ParamAccess.item);
            pManager.AddBrepParameter("Models", "M", "The brep models of cam and follower", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Plane p=Plane.Unset;
            if (!DA.GetData(0, ref p)) { return; }
            int type1 = 0, type2 = 0;
            if (!DA.GetData(1, ref type1)) { return; }
            if (!DA.GetData(2, ref type2)) { return; }
            Point3d p1 = Point3d.Unset;
            Point3d p2 = Point3d.Unset;
            if (!DA.GetData(3, ref p1)) { return; }
            if (!DA.GetData(4, ref p2)) { return; }
            double r1 = 0, r2 = 0,rr=0;
            if (!DA.GetData(5, ref r1)) { return; }
            if (!DA.GetData(6, ref r2)) { return; }
            if (!DA.GetData(7, ref rr)) { return; }
            double degree = 0;
            if (!DA.GetData(8, ref degree)) { return; }
            Cam c = new Cam(type1, r1, r2, p.Origin, p.Normal, p.XAxis,1,rr,degree);
            Follower f = new Follower(type2, c, p1, p2, rr, rr*2);
            f.Follow();
            DA.SetData(0, c);
            DA.SetData(0, f);
            Transform t1 = c.Offset;
            bool r=t1.IsRotation;
            Transform t2 = f.Offset;
            BoundingBox b1 = c.Model.GetBoundingBox(true );
            BoundingBox b2 = f.Model.GetBoundingBox(true );
            BoundingBox b3 = c.GetModelinWorldCoordinate().GetBoundingBox(true);
            BoundingBox b4 = f.GetModelinWorldCoordinate().GetBoundingBox(true);
            double d=c.Degree;
            DA.SetDataList(0, new List<Brep> { c.GetModelinWorldCoordinate(),f.GetModelinWorldCoordinate()});
            
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
            get { return new Guid("6bf786f8-5f23-4ac8-af0a-36718f2a1a91"); }
        }
    }
}