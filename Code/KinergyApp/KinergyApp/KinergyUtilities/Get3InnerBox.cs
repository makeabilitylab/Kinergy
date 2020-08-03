using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Kinergy.Geom;
using Rhino;
using Rhino.Geometry;

namespace KinergyUtilities
{
    public class Get3InnerBox : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the ModelPreprocessing class.
        /// </summary>
        public Get3InnerBox()
          : base("GetInnerBox", "InnerBox",
              "Preprocess given brep model to generate box inner space and skeleton. Beware that skeleton might fail when given brep is not rounded linear shape",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "The brep model to preprocess", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction", "D", "The x direction of model. It is needed when you are using ", GH_ParamAccess.item);
            pManager.AddNumberParameter("Region of interest Start", "St", "The starting point of region of interest on axis of given direction", GH_ParamAccess.item);
            pManager.AddNumberParameter("Region of interest End", "Ed", "The ending point of region of interest on axis of given direction", GH_ParamAccess.item);
            //pManager.AddBooleanParameter("With medial axis", "MA", "Whether to generate medial axis.", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBoxParameter("Inner Box1", "B1", "The maximal inner box1", GH_ParamAccess.item);
            pManager.AddBoxParameter("Inner Box2", "B2", "The maximal inner box2", GH_ParamAccess.item);
            pManager.AddBoxParameter("Inner Box3", "B3", "The maximal inner box3", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Plane1", "p1", "The first cutting plane",GH_ParamAccess.item);
            pManager.AddPlaneParameter("Plane2", "p2", "The second cutting plane", GH_ParamAccess.item);
            pManager.AddBrepParameter("ModelPart1", "M1", "", GH_ParamAccess.item);
            pManager.AddBrepParameter("ModelPart2", "M2", "", GH_ParamAccess.item);
            pManager.AddBrepParameter("ModelPart3", "M3", "", GH_ParamAccess.item);
            //pManager.AddCurveParameter("Medial Axis", "MA", "The generated medial axis", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep m=null;
            if (!DA.GetData(0, ref m))
                return;
            Vector3d v = Vector3d.Unset;
            if (!DA.GetData(1, ref v))
                return;
            double st = 0, ed = 0;
            if (!DA.GetData(2, ref st))
                return;
            if (!DA.GetData(3, ref ed))
                return;
            if (st < 0)
                st = 0;
            if (st > 1)
                st = 1;
            if (ed < 0)
                ed = 0;
            if (ed > 1)
                ed = 1;
            if(st>ed)
            {
                double t = st;st = ed;ed = t;
            }
            //Cut the model with 2 planes
            BoxLike b = new BoxLike(m, v);
            BoundingBox box= b.Bbox;
            box.Transform(b.RotateBack);
            Point3d start=box.PointAt(0, 0.5, 0.5);
            Point3d end = box.PointAt(1, 0.5, 0.5);
            Curve skeleton = new Line(start, end).ToNurbsCurve();
            Plane p1 = new Plane(skeleton.PointAtNormalizedLength(st), v);
            p1.ExtendThroughBox(box,out _,out _);
            Plane p1Reverse = new Plane(skeleton.PointAtNormalizedLength(st), -v);
            p1Reverse.ExtendThroughBox(box, out _, out _);
            Plane p2 = new Plane(skeleton.PointAtNormalizedLength(ed), v);
            p2.ExtendThroughBox(box, out _, out _);
            Plane p2Reverse = new Plane(skeleton.PointAtNormalizedLength(ed), -v);
            p2Reverse.ExtendThroughBox(box, out _, out _);
            Brep[] Cut_Brep1 = m.Trim(p1, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep Brep1=Cut_Brep1[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep[] Cut_Brep1rest = m.Trim(p1Reverse, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep[] Cut_Brep2 = Cut_Brep1rest[0].Trim(p2, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep Brep2 = Cut_Brep2[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep[] Cut_Brep3 = Cut_Brep1rest[0].Trim(p2Reverse, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep Brep3 = Cut_Brep3[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            DA.SetData(3, p1);
            DA.SetData(4, p2);
            DA.SetData(5, Brep1);
            DA.SetData(6, Brep2);
            DA.SetData(7, Brep3);
            b = new BoxLike(Brep1, v);
            double volumn = 0;
            BoundingBox result1=BoundingBox.Empty;
            for(double i=0.2;i<=0.8;i+=0.2)
            {
                if(b.GetInnerEmptySpaceBox(i))
                {
                    BoundingBox bbox = b.InnerEmptySpaceBbox;
                    if (volumn < bbox.Volume)
                    { 
                        volumn = bbox.Volume;
                        result1 = bbox;
                    }
                }
            }
            result1.Transform(b.RotateBack);
            DA.SetData(0, result1);

            b = new BoxLike(Brep2, v);
            volumn = 0;
            BoundingBox result2 = BoundingBox.Empty;
            for (double i = 0.2; i <= 0.8; i += 0.2)
            {
                if (b.GetInnerEmptySpaceBox(i))
                {
                    BoundingBox bbox = b.InnerEmptySpaceBbox;
                    if (volumn < bbox.Volume)
                    {
                        volumn = bbox.Volume;
                        result2 = bbox;
                    }
                }
            }
            result2.Transform(b.RotateBack);
            DA.SetData(1, result2);

            b = new BoxLike(Brep3, v);
            volumn = 0;
            BoundingBox result3 = BoundingBox.Empty;
            for (double i = 0.2; i <= 0.8; i += 0.2)
            {
                if (b.GetInnerEmptySpaceBox(i))
                {
                    BoundingBox bbox = b.InnerEmptySpaceBbox;
                    if (volumn < bbox.Volume)
                    {
                        volumn = bbox.Volume;
                        result3 = bbox;
                    }
                }
            }
            result3.Transform(b.RotateBack);
            DA.SetData(2, result3);
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
            get { return new Guid("cfc8c21b-47ab-4e33-beb3-f4306a3ad87d"); }
        }
    }
}