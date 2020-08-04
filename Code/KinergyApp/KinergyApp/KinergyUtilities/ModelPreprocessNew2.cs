﻿using System;
using System.Collections.Generic;
using Kinergy.Geom;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using System.Linq;
namespace HumanUIforKinergy.KinergyUtilities
{
    public class ModelPreprocessNew2 : GH_Component
    {
        Brep model = null;
        Vector3d v = Vector3d.Unset;
        double t1, t2;
        Guid guid1, guid2;
        bool interacting = false;
        bool generated = false;
        Plane pl1, pl2;
        PlaneSurface s1, s2;
        bool finished;
        Guid selected = Guid.Empty;
        Curve skeleton = null;
        Vector3d skeletonVec = Vector3d.Unset;
        /// <summary>
        /// Initializes a new instance of the ModelPreprocessNew2 class.
        /// </summary>
        public ModelPreprocessNew2()
          : base("ModelPreprocessNew2", "MPN2",
              "Description",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "B", "The brep model to preprocess", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Inner space type", "T", "The type of inner space, 1 for box and 2 for cylinder. Currently we only support these 2 types", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Restart", "R", "Turn this true to restart", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("ModelRegionOfInterest", "M", "", GH_ParamAccess.item);
            pManager.AddBoxParameter("InnerBox", "B", "The biggest inner box generated", GH_ParamAccess.item);
            pManager.AddGeometryParameter("InnerCylinder", "C", "The biggest inner cylinder generated", GH_ParamAccess.item);
            pManager.AddCurveParameter("Skeleton", "S", "", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction", "D", "The main direction of model.", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            RhinoApp.KeyboardEvent+=GetKey;
            t1 = 0;
            t2 = 1;
            finished = false;
            generated = false;
            bool restart = false;
            int type = 0;
            if (!DA.GetData(0, ref model))
                return;
            v = Vector3d.XAxis;

            if (!DA.GetData(1, ref type))
                return;
            if (type != 1 && type != 2)
                throw new Exception("Invalid type value! Currently we only support 1 and 2.");
            if (!DA.GetData(2, ref restart))
                return;
            if (restart)
                return;

            //Generate 6 arrows and record their guids.
            BoundingBox box = model.GetBoundingBox(true);
            box.Transform(Transform.Scale(box.Center, 2));
            double arrowScale = box.Diagonal.Length / 100;
            
            //Construct and activate command.
            Rhino.Input.Custom.GetPoint gp = new Rhino.Input.Custom.GetPoint();
            gp.SetCommandPrompt("Click and drag partition plane to adjust their position. Press enter to confirm and move on.");
            gp.MouseDown += Gp_SelectionMouseDown;
            gp.MouseMove += Gp_SelectionMouseMove;

            //gp.DynamicDraw += Gp_SelectionDynamicDraw;
            //gp.AcceptEnterWhenDone(true);
            gp.AcceptNothing(true);
            Rhino.Input.GetResult r;
            interacting = true;
            do
            {
                if (!generated)
                    GeneratePlanes();
                r = gp.Get(true);

            } while (r != Rhino.Input.GetResult.Nothing);
            interacting = false;
            RhinoDoc.ActiveDoc.Objects.Delete(guid1, true);
            RhinoDoc.ActiveDoc.Objects.Delete(guid2, true);
            
            Plane p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), -v);
            //p1Reverse.ExtendThroughBox(box, out _, out _);
            Plane p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), -v);
            //p2Reverse.ExtendThroughBox(box, out _, out _);
            /*Brep[] Cut_Brep1 = m.Trim(pl1, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep Brep1 = Cut_Brep1[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);*/
            Brep[] Cut_Brep1rest = model.Trim(p1Reverse, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep BrepRest = null;
            try
            {
                BrepRest = Cut_Brep1rest[0];
            }
            catch
            {
                BrepRest = model;
            }
            Brep[] Cut_Brep2 = BrepRest.Trim(pl2, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep Brep2 = null;
            try
            {
                Brep2 = Cut_Brep2[0];
            }
            catch
            {
                Brep2 = BrepRest;
            }
            try
            {
                Brep2 = Cut_Brep2[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            }
            catch
            { }
            /*Brep[] Cut_Brep3 = Cut_Brep1rest[0].Trim(p2Reverse, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep Brep3 = Cut_Brep3[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Rhino.Input.Custom.GetPoint ctrl_first_pt_sel = new Rhino.Input.Custom.GetPoint();*/

            BoxLike b = new BoxLike(Brep2, v);
            double volumn = 0;
            BoundingBox result1 = BoundingBox.Empty;
            Cylinder result2 = Cylinder.Unset;
            if (type == 1)
            {
                for (double i = 0.2; i <= 0.8; i += 0.1)
                {
                    if (b.GetInnerEmptySpaceBox(i))
                    {
                        BoundingBox bbox = b.InnerEmptySpaceBbox;
                        if (volumn < bbox.Volume)
                        {
                            volumn = bbox.Volume;
                            result1 = bbox;
                            result1.Transform(b.RotateBack);
                            DA.SetData(1, result1);
                        }
                    }
                }
            }
            else if (type == 2)
            {
                if (b.GetInnerEmptySpaceCylinder())
                {
                    Cylinder c = b.InnerEmptyCylinder;
                    //result2 = c.ToBrep(true,true);
                    result2 = c;
                    Brep b2 = result2.ToBrep(true, true);
                    b2.Transform(b.RotateBack);
                    DA.SetData(2, b2);
                }
            }
            else
                throw new Exception("Invalid type");
            DA.SetData(0, Brep2);
            DA.SetData(3, skeleton);
            DA.SetData(4, v);
        }
        private void GeneratePlanes()
        {
            generated = true;
            //Delete these before generating new ones
            RhinoDoc.ActiveDoc.Objects.Delete(guid1, true);
            RhinoDoc.ActiveDoc.Objects.Delete(guid2, true);

            BoxLike b = new BoxLike(model, v);
            BoundingBox box = b.Bbox;
            Interval yInterval = new Interval(-(box.Max.Y - box.Min.Y) * 0.6, (box.Max.Y - box.Min.Y) * 0.6), zInterval = new Interval(-(box.Max.Z - box.Min.Z) * 0.6, (box.Max.Z - box.Min.Z) * 0.6);
            box.Transform(b.RotateBack);
            /*Point3d start = box.PointAt(0, 0.5, 0.5);
            Point3d end = box.PointAt(1, 0.5, 0.5);*/ //this doesn't work!

            skeleton = b.Skeleton;
            skeleton.Transform(b.RotateBack);
            skeletonVec = new Vector3d(skeleton.PointAtEnd) - new Vector3d(skeleton.PointAtStart);
            pl1 = new Plane(skeleton.PointAtNormalizedLength(t1), v);
            pl2 = new Plane(skeleton.PointAtNormalizedLength(t2), v);
            s1 = new PlaneSurface(pl1, yInterval, zInterval);
            s2 = new PlaneSurface(pl2, yInterval, zInterval);
            guid1 = RhinoDoc.ActiveDoc.Objects.Add(s1);
            guid2 = RhinoDoc.ActiveDoc.Objects.Add(s2);
            RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();
        }
        private void GetKey(int key)
        {
            if (!interacting)
                return;
            
            if (key  == 0x51)//Q
            {
                v.Transform(Transform.Rotation(Math.PI / 90, Vector3d.ZAxis, Point3d.Origin));
                generated = false;
                GeneratePlanes();
            }
            else if (key == 0x57)//W
            {
                v.Transform(Transform.Rotation(-Math.PI / 90, Vector3d.ZAxis, Point3d.Origin));
                generated = false;
                GeneratePlanes();
            }
            else if (key== 0x41)//A
            {
                v.Transform(Transform.Rotation(Math.PI / 90, Vector3d.XAxis, Point3d.Origin));
                generated = false;
                GeneratePlanes();
            }
            else if (key == 0x53)//S
            {
                v.Transform(Transform.Rotation(-Math.PI /90, Vector3d.XAxis, Point3d.Origin));
                generated = false;
                GeneratePlanes();
            }
            else if (key==0x5A)//Z
            {
                v.Transform(Transform.Rotation(Math.PI / 90, Vector3d.YAxis, Point3d.Origin));
                generated = false;
                GeneratePlanes();
            }
            else if (key == 0x58)//X
            {
                v.Transform(Transform.Rotation(-Math.PI / 90, Vector3d.YAxis, Point3d.Origin));
                generated = false;
                GeneratePlanes();
            }
        }
        private void Gp_SelectionMouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            //在这个函数里，e.Point是当前的鼠标所在的位置的对应的Rhino里的3D的点  
            if (selected != Guid.Empty)
                selected = Guid.Empty;
            else
            {
                var p = e.Point;
                double dis1 = Math.Abs(pl1.DistanceTo(p)), dis2 = Math.Abs(pl2.DistanceTo(p));
                
                List<double> distances = new List<double> { dis1, dis2 };
                double min = distances.Min();
                if (min > 5)
                { return; }
                else if (min == dis1)
                    selected = guid1;
                else if (min == dis2)
                    selected = guid2;
            }

        }
        private void Gp_SelectionMouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (selected == Guid.Empty)
                return;
            //在这个函数里，e.Point是当前的鼠标所在的位置的对应的Rhino里的3D的点  
            double t = 0, tn = 0;
            skeleton.ClosestPoint(e.Point, out t);
            skeleton.NormalizedLengthParameter(t, out tn);
            if (tn < 0)
                tn = 0;
            if (tn > 1)
                tn = 1;
            if (selected == guid1)
            {
                //calculate where is the mouse and change t
                if (Math.Abs(t1 - tn) > 0.01)
                {
                    //move and update t1
                    Transform m = Transform.Translation(skeletonVec * (tn - t1));
                    RhinoDoc.ActiveDoc.Objects.Transform(guid1, m, true);
                    pl1.Transform(m);
                    t1 = tn;
                }

            }
            if (selected == guid2)
            {
                //calculate where is the mouse and change t
                if (Math.Abs(t2 - tn) > 0.01)
                {
                    //move and update t1
                    Transform m = Transform.Translation(skeletonVec * (tn - t2));
                    RhinoDoc.ActiveDoc.Objects.Transform(guid2, m, true);
                    pl2.Transform(m);
                    t2 = tn;
                }

            }
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
            get { return new Guid("d6e72d4f-8c96-45b5-9256-1d919eeececd"); }
        }
    }
}