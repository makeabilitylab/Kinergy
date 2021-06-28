using System;
using System.Collections.Generic;
using Kinergy.Geom;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using System.Linq;
using KinergyUtilities;
using Rhino.DocObjects;
using Rhino.Input;

namespace HumanUIforKinergy.KinergyUtilities
{
    public class RegionSelection : GH_Component
    {
        Brep model = null;
        Vector3d v = Vector3d.Unset;
        double t1, t2;
        double arrowScale = 0;
        Point3d center = Point3d.Unset;
        Guid guid1, guid2,ArrowCurve;
        bool OperatingArrow = false;
        bool PlaneGenerated = false;
        bool ArrowGenerated = false;
        bool PlaneSelected = false;
        Plane pl1, pl2;
        PlaneSurface s1, s2;
        Guid selected = Guid.Empty;
        Curve skeleton = null;
        Vector3d skeletonVec = Vector3d.Unset;
        ProcessingWin processingwin = new ProcessingWin();
        /// <summary>
        /// Initializes a new instance of the ModelPreprocessNew3 class.
        /// </summary>
        public RegionSelection()
          : base("RegionSelection", "RegionSelection",
              "Description",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            //pManager.AddBrepParameter("Brep", "B", "The brep model to preprocess", GH_ParamAccess.item);
            //pManager.AddIntegerParameter("Inner space type", "T", "The type of inner space, 1 for box and 2 for cylinder. Currently we only support these 2 types", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Restart", "R", "Turn this true to restart", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Select Direction", "SD", "Start selecting main direction", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Select Region", "SR", "Start selecting region of interest", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("ModelRegionOfInterest", "M", "", GH_ParamAccess.item);
            pManager.AddGeometryParameter("InnerCavity", "C", "The biggest inner cavity generated", GH_ParamAccess.item);
            pManager.AddCurveParameter("Skeleton", "S", "", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction", "D", "The main direction of model.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Trigger", "S", "trigger instant translation", GH_ParamAccess.item);
            pManager.AddNumberParameter("SkeletonStartPos", "SPos", "the start of the selected segment", GH_ParamAccess.item);
            pManager.AddNumberParameter("SkeletonEndPos", "EPos", "the end of the selected segment", GH_ParamAccess.item);
            pManager.AddBrepParameter("OriginalBrep", "OriB", "original brep", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool IT_trigger = false;
            DA.SetData(4, IT_trigger);

            RhinoApp.KeyboardEvent += GetKey;
            t1 = 0;
            t2 = 1;
            PlaneGenerated = false;
            bool restart = false;
            //int type = 0;
            //if (!DA.GetData(0, ref model))
            //    return;

            //if (!DA.GetData(1, ref type))
            //    return;
            //if (type != 1 && type != 2)
            //    throw new Exception("Invalid type value! Currently we only support 1 and 2.");

            if (!DA.GetData(0, ref restart))
                return;
            if (restart == false)
            {
                v = Vector3d.Unset;
                t1 = 0;
                t2 = 1;
                PlaneSelected = false;
                return;
            }

            ObjRef objSel_ref;
            Guid selObjId = Guid.Empty;
            var rc = RhinoGet.GetOneObject("Select a surface or polysurface", false, ObjectType.AnyObject, out objSel_ref);
            if (rc == Rhino.Commands.Result.Success)
            {
                // select a brep
                selObjId = objSel_ref.ObjectId;
                ObjRef currObj = new ObjRef(selObjId);

                model = currObj.Brep();
                DA.SetData(7, model);
                
                bool selectDirection = false, selectRegion = false;

                if (!DA.GetData(1, ref selectDirection))
                    return;
                if (!DA.GetData(2, ref selectRegion))
                    return;

                // get the inner cavity of the selected brep
                BoundingBox box = model.GetBoundingBox(true);
                box.Inflate(-2.0);
                box.Transform(Transform.Scale(box.Center, 2));
                arrowScale = box.Diagonal.Length / 100;
                center = box.Center;

                if (v == Vector3d.Unset && selectDirection)
                {
                    v = Vector3d.XAxis;
                    Rhino.Input.Custom.GetPoint gp1 = new Rhino.Input.Custom.GetPoint();
                    gp1.SetCommandPrompt("Press AS, ZX, or QW to rotate the partition planes around X, Y, or Z axis (CW and CCW). Press enter to continue.");
                    gp1.AcceptNothing(true);
                    Rhino.Input.GetResult r1;

                    OperatingArrow = true;
                    do
                    {
                        if (!ArrowGenerated)
                            GenerateArrow();
                        r1 = gp1.Get(true);
                        
                    } while (r1 != Rhino.Input.GetResult.Nothing);
                    OperatingArrow = false;
                }

                if (v != Vector3d.Unset && selectRegion)
                {
                    Rhino.Input.Custom.GetPoint gp2 = new Rhino.Input.Custom.GetPoint();
                    gp2.SetCommandPrompt("Click and drag the partition plane to adjust their position. Press enter to confirm and move on");
                    gp2.MouseDown += Gp_SelectionMouseDown;
                    gp2.MouseMove += Gp_SelectionMouseMove;

                    //gp.DynamicDraw += Gp_SelectionDynamicDraw;
                    //gp.AcceptEnterWhenDone(true);
                    gp2.AcceptNothing(true);
                    Rhino.Input.GetResult r2;
                    do
                    {
                        if (!PlaneGenerated)
                            GeneratePlanes();
                        r2 = gp2.Get(true);

                    } while (r2 != Rhino.Input.GetResult.Nothing);
                    PlaneSelected = true;
                }

                RhinoDoc.ActiveDoc.Objects.Delete(ArrowCurve, true);
                RhinoDoc.ActiveDoc.Objects.Delete(guid1, true);
                RhinoDoc.ActiveDoc.Objects.Delete(guid2, true);
                PlaneGenerated = false;
                ArrowGenerated = false;

                // Call out the waiting window
                processingwin.Show();

                if (v != Vector3d.Unset && PlaneSelected)
                {
                    Plane p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), -v);
                    DA.SetData(5, t1);
                    //p1Reverse.ExtendThroughBox(box, out _, out _);
                    Plane p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), -v);
                    DA.SetData(6, t2);
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
                    Brep result1 = null;
                    Cylinder result2 = Cylinder.Unset;
                    Brep b2 = null;
                    double v_box = 0.0, v_cylinder = 0.0;
                    //if (type == 1)
                    //{

                    // Calculate the volume of the inner box
                    for (double i = 0.2; i <= 0.8; i += 0.1)
                    {
                        if (b.GetInnerEmptySpaceBox(i))
                        {
                            BoundingBox bbox = b.InnerEmptySpaceBbox;
                            if (volumn < bbox.Volume)
                            {
                                volumn = bbox.Volume;
                                result1 = b.InnerEmptySpaceBoxBrep;
                                result1.Transform(b.RotateBack);
                                v_box = result1.GetVolume();
                                // DA.SetData(1, result1);
                            }
                        }
                    }
                    //}
                    //else if (type == 2)
                    //{

                    // Calculate the volume of the inner cylinder 
                    if (b.GetInnerEmptySpaceCylinder())
                    {
                        Cylinder c = b.InnerEmptyCylinder;
                        //result2 = c.ToBrep(true,true);
                        result2 = c;
                        b2 = result2.ToBrep(true, true);
                        b2.Transform(b.RotateBack);
                        v_cylinder = b2.GetVolume();
                        //DA.SetData(2, b2);
                    }
                    //}
                    //else
                    //    throw new Exception("Invalid type");

                    if (v_box >= v_cylinder)
                        DA.SetData(1, result1);
                    else
                        DA.SetData(1, b2);

                    DA.SetData(0, Brep2);
                    DA.SetData(2, skeleton);
                    DA.SetData(3, v);
                    IT_trigger = true;
                    DA.SetData(4, IT_trigger);
                }
                // Hide the processing window
                processingwin.Hide();

            }
            
        }
        private void GenerateArrow()
        {
            ArrowGenerated = true;
            RhinoDoc.ActiveDoc.Objects.Delete(ArrowCurve, true);
            Arrow a = new Arrow(v, center, arrowScale);
            ArrowCurve= RhinoDoc.ActiveDoc.Objects.Add(a.ArrowCurve);

            RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();
        }
        private void GeneratePlanes()
        {
            PlaneGenerated = true;
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
            if (!OperatingArrow)
                return;

            if (key == 0x51)//Q
            {
                v.Transform(Transform.Rotation(Math.PI / 90, Vector3d.ZAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x57)//W
            {
                v.Transform(Transform.Rotation(-Math.PI / 90, Vector3d.ZAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x41)//A
            {
                v.Transform(Transform.Rotation(Math.PI / 90, Vector3d.XAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x53)//S
            {
                v.Transform(Transform.Rotation(-Math.PI / 90, Vector3d.XAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x5A)//Z
            {
                v.Transform(Transform.Rotation(Math.PI / 90, Vector3d.YAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x58)//X
            {
                v.Transform(Transform.Rotation(-Math.PI / 90, Vector3d.YAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
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
            get { return new Guid("266e950e-20e6-4979-a37a-0db811fbefeb"); }
        }
    }
}