using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel.Components;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.DocObjects;
using Rhino.Collections;
using Rhino.Input.Custom;
using Rhino;
using System.Drawing;
using Kinergy.Geom;

namespace KinergyUtilities
{
    public class UserSelection
    {
        public static int UserSelectPointInRhino(List<Point3d> pts ,RhinoDoc myDoc)
        {
            List<Guid> addedPoints = new List<Guid>();
            foreach(Point3d pt in pts)
            {
                addedPoints.Add(myDoc.Objects.AddPoint(pt));
            }
            Rhino.RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();
            ObjRef selectedPoint;
            int selection = -1;
            bool success = false;
            while (success == false)
            {
                var command = RhinoGet.GetOneObject("Please select a Point", true, ObjectType.Point, out selectedPoint);
                Guid guid_r = selectedPoint.ObjectId;

                if (command == Rhino.Commands.Result.Success)
                {
                    for (int i = 0; i < addedPoints.Count(); i++)
                    {
                        if (addedPoints[i] == guid_r)
                        {
                            selection = i;
                            success = true;
                            break;
                        }
                    }
                }
            }
            foreach (Guid g in addedPoints)
            {
                myDoc.Objects.Delete(g, true);
            }
            return selection;
        }
        public static int UserSelectCurveInRhino(List<Curve> crvs ,RhinoDoc myDoc)
        {
            List<Guid> addedCurves = new List<Guid>();
            foreach(Curve c in crvs)
            {
                addedCurves.Add(myDoc.Objects.AddCurve(c));
            }
            Rhino.RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();
            ObjRef selectedCurve;
            int selection = -1;
            bool success = false;
            while (success == false)
            {
                var command = RhinoGet.GetOneObject("Please select a curve", true, ObjectType.Curve, out selectedCurve);
                Guid guid_r = selectedCurve.ObjectId;

                if (command == Rhino.Commands.Result.Success)
                {
                    for(int i=0;i<addedCurves.Count();i++)
                    {
                        if(addedCurves[i]==guid_r)
                        {
                            selection=i;
                            success = true;
                            break;
                        }
                    }
                }
            }
            foreach (Guid g in addedCurves)
            {
                myDoc.Objects.Delete(g, true);
            }
            return selection;
        }
        /// <summary>
        /// Start an axis selecting process and return the selected axis. 0 for failed selection, 1 for x,2 for y and 3 for z
        /// </summary>
        /// <param name="center">The starting point for 3 arrows</param>
        /// <param name="arrowLength">The length of arrow. Usually just use the default value</param>
        /// <param name="myDoc"></param>
        /// <returns></returns>
        
        
    }
    /// <summary>
    /// This class initiates an XYZ axis selection operation when one of it's instance is constructed. The selection result is a public member called selectAxis
    /// </summary>
    public class XYZselection
    {
        public int selectedAxis = 0;
        Guid xArrowID, yArrowID, zArrowID;
        ObjectAttributes redAttribute,greenAttribute,blueAttribute;
        RhinoDoc myDoc;
        /// <summary>
        /// Call an XYZ axis selection operation
        /// </summary>
        /// <param name="center">The center point of 3 axis</param>
        /// <param name="currDoc">The currently in use rhino doc</param>
        /// <param name="arrowLength">the length of arrow. Usually just use default value</param>
        public XYZselection(Point3d center, RhinoDoc currDoc, ObjectAttributes _redAttribute, ObjectAttributes _greenAttribute, ObjectAttributes _blueAttribute ,double arrowLength = 30)
        {
            myDoc = currDoc;
            redAttribute = _redAttribute;
            greenAttribute = _greenAttribute;
            blueAttribute = _greenAttribute;
            GenerateXYZArrow(center, arrowLength, myDoc);
            Rhino.Input.Custom.GetPoint gp = new Rhino.Input.Custom.GetPoint();
            gp.SetCommandPrompt("Select one translation axis. Press enter to confirm and continue.");
            gp.AcceptNothing(true);
            gp.MouseDown += Gp_MouseDown;
            gp.MouseMove += Gp_MouseMove;
            Rhino.Input.GetResult r1;
            do
            {
                r1 = gp.Get(true);

            } while (r1 != Rhino.Input.GetResult.Nothing);
            myDoc.Objects.Delete(xArrowID, true);
            myDoc.Objects.Delete(yArrowID, true);
            myDoc.Objects.Delete(zArrowID, true);
        }
        private void GenerateXYZArrow(Point3d center, double length, RhinoDoc myDoc)
        {
            double axisRadius = 1;
            Point3d XEndPt = center + Vector3d.XAxis * length;
            Point3d YEndPt = center + Vector3d.YAxis * length;
            Point3d ZEndPt = center + Vector3d.ZAxis * length;

            Line xLn = new Line(center, XEndPt);
            Curve xCrv = xLn.ToNurbsCurve();
            Brep xAxisBrep = Brep.CreatePipe(xCrv, axisRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Plane xArrowPln = new Plane(XEndPt + Vector3d.XAxis * 5, (-1) * Vector3d.XAxis);
            Cone xAxisArrowTipCone = new Cone(xArrowPln, 5, 2 * axisRadius);
            Brep xAxisArrowTipBrep = xAxisArrowTipCone.ToBrep(true);
            Brep xAxisArrow = Brep.CreateBooleanUnion(new List<Brep> { xAxisBrep, xAxisArrowTipBrep }, myDoc.ModelAbsoluteTolerance)[0];

            Line yLn = new Line(center, YEndPt);
            Curve yCrv = yLn.ToNurbsCurve();
            Brep yAxisBrep = Brep.CreatePipe(yCrv, axisRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Plane yArrowPln = new Plane(YEndPt + Vector3d.YAxis * 5, (-1) * Vector3d.YAxis);
            Cone yAxisArrowTipCone = new Cone(yArrowPln, 5, 2 * axisRadius);
            Brep yAxisArrowTipBrep = yAxisArrowTipCone.ToBrep(true);
            Brep yAxisArrow = Brep.CreateBooleanUnion(new List<Brep> { yAxisBrep, yAxisArrowTipBrep }, myDoc.ModelAbsoluteTolerance)[0];

            Line zLn = new Line(center, ZEndPt);
            Curve zCrv = zLn.ToNurbsCurve();
            Brep zAxisBrep = Brep.CreatePipe(zCrv, axisRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Plane zArrowPln = new Plane(ZEndPt + Vector3d.ZAxis * 5, (-1) * Vector3d.ZAxis);
            Cone zAxisArrowTipCone = new Cone(zArrowPln, 5, 2 * axisRadius);
            Brep zAxisArrowTipBrep = zAxisArrowTipCone.ToBrep(true);
            Brep zAxisArrow = Brep.CreateBooleanUnion(new List<Brep> { zAxisBrep, zAxisArrowTipBrep }, myDoc.ModelAbsoluteTolerance)[0];

            xArrowID = myDoc.Objects.AddBrep(xAxisArrow, redAttribute);
            yArrowID = myDoc.Objects.AddBrep(yAxisArrow, greenAttribute);
            zArrowID = myDoc.Objects.AddBrep(zAxisArrow, blueAttribute);
            myDoc.Views.Redraw();
        }
        private void Gp_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep xBrep = (Brep)myDoc.Objects.Find(xArrowID).Geometry;
            Brep yBrep = (Brep)myDoc.Objects.Find(yArrowID).Geometry;
            Brep zBrep = (Brep)myDoc.Objects.Find(zArrowID).Geometry;

            double x_dis = xBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double y_dis = yBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double z_dis = zBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if (x_dis <= y_dis && x_dis <= z_dis)
            {
                myDoc.Objects.UnselectAll();
                myDoc.Objects.Select(xArrowID);
            }
            else if (y_dis <= x_dis && y_dis <= z_dis)
            {
                myDoc.Objects.UnselectAll();
                myDoc.Objects.Select(yArrowID);
            }
            else if (z_dis <= y_dis && z_dis <= x_dis)
            {
                myDoc.Objects.UnselectAll();
                myDoc.Objects.Select(zArrowID);
            }
            else
            {
                myDoc.Objects.UnselectAll();
            }
        }

        private void Gp_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep xBrep = (Brep)myDoc.Objects.Find(xArrowID).Geometry;
            Brep yBrep = (Brep)myDoc.Objects.Find(yArrowID).Geometry;
            Brep zBrep = (Brep)myDoc.Objects.Find(zArrowID).Geometry;

            double x_dis = xBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double y_dis = yBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double z_dis = zBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if (x_dis <= y_dis && x_dis <= z_dis)
            {
                selectedAxis = 1;
            }
            else if (y_dis <= x_dis && y_dis <= z_dis)
            {
                selectedAxis = 2;
            }
            else if (z_dis <= y_dis && z_dis <= x_dis)
            {
                selectedAxis = 3;
            }
            else
            {
                selectedAxis = -1;
            }
        }

    }
    /// <summary>
    /// This class initiates a portion operation (portion a model with 2 parallel planes into 3 parts) when one instance is constructed. The portion result and several key variables are listed as public.
    /// </summary>
    public class PortionSelection
    {
        Brep model;
        RhinoDoc myDoc;
        Guid guide1, guide2,selected;
        public double t1, t2;
        int selectedAxis = 0;
        public Vector3d axis;
        public Curve skeleton;
        Vector3d skeletonVec;
        Plane pl1, pl2;
        PlaneSurface s1, s2;
        /// <summary>
        /// This constructor initiates a portion operation (portion a model with 2 parallel planes into 3 parts).
        /// </summary>
        /// <param name="m">Model as brep to be portioned</param>
        /// <param name="axisIndex">The index of main axis, which is going to be used as the normal vector of cutting planes. 1 for Xaxis, 2 for Y and 3 for Z.</param>
        /// <param name="currDoc">The currently in use rhino doc</param>
        public PortionSelection(Brep m,int axisIndex,RhinoDoc currDoc)
        {
            t1 = 0;
            t2 = 1;
            model = m;
            selectedAxis = axisIndex;
            myDoc = currDoc;
            GeneratePlanes();
            GetPoint gp2 = new GetPoint();
            gp2.SetCommandPrompt("Click and drag the partition plane to adjust their position. Press enter to confirm and continue.");
            gp2.MouseDown += Gp_MouseDown;
            gp2.MouseMove += Gp_MouseMove;
            gp2.AcceptNothing(true);
            GetResult r2;
            do
            {
                r2 = gp2.Get(true);

            } while (r2 != GetResult.Nothing);
            //RhinoDoc.ActiveDoc.Objects.Delete(ArrowCurve, true);
            myDoc.Objects.Delete(guide1, true);
            myDoc.Objects.Delete(guide2, true);
            myDoc.Views.Redraw();
        }
        private void GeneratePlanes()
        {
            //Delete these before generating new ones
            myDoc.Objects.Delete(guide1, true);
            myDoc.Objects.Delete(guide2, true);

            switch (selectedAxis)
            {
                case 1: axis = Vector3d.XAxis; break;
                case 2: axis = Vector3d.YAxis; break;
                case 3: axis = Vector3d.ZAxis; break;
                default: break;
            }
            if (axis != null)
            {
                BoxLike b = new BoxLike(model, axis);
                BoundingBox box = b.Bbox;
                box.Transform(b.RotateBack);
                /*Point3d start = box.PointAt(0, 0.5, 0.5);
                Point3d end = box.PointAt(1, 0.5, 0.5);*/ //this doesn't work!

                skeleton = b.Skeleton;
                skeleton.Transform(b.RotateBack);
                skeletonVec = new Vector3d(skeleton.PointAtEnd) - new Vector3d(skeleton.PointAtStart);
                pl1 = new Plane(skeleton.PointAtNormalizedLength(t1), axis);
                pl2 = new Plane(skeleton.PointAtNormalizedLength(t2), axis);

                Interval plnXInterval;
                Interval plnYInterval;
                Interval plnZInterval;
                if (selectedAxis == 1)
                {
                    // x axis is selected
                    plnYInterval = new Interval(-(box.Max.Y - box.Min.Y) * 0.6, (box.Max.Y - box.Min.Y) * 0.6);
                    plnZInterval = new Interval(-(box.Max.Z - box.Min.Z) * 0.6, (box.Max.Z - box.Min.Z) * 0.6);
                    s1 = new PlaneSurface(pl1, plnYInterval, plnZInterval);
                    s2 = new PlaneSurface(pl2, plnYInterval, plnZInterval);
                }
                else if (selectedAxis == 2)
                {
                    // y axis is selected
                    plnXInterval = new Interval(-(box.Max.X - box.Min.X) * 0.6, (box.Max.X - box.Min.X) * 0.6);
                    plnZInterval = new Interval(-(box.Max.Z - box.Min.Z) * 0.6, (box.Max.Z - box.Min.Z) * 0.6);
                    s1 = new PlaneSurface(pl1, plnXInterval, plnZInterval);
                    s2 = new PlaneSurface(pl2, plnXInterval, plnZInterval);
                }
                else if (selectedAxis == 3)
                {
                    // z axis is selected
                    plnXInterval = new Interval(-(box.Max.X - box.Min.X) * 0.6, (box.Max.X - box.Min.X) * 0.6);
                    plnYInterval = new Interval(-(box.Max.Y - box.Min.Y) * 0.6, (box.Max.Y - box.Min.Y) * 0.6);
                    s1 = new PlaneSurface(pl1, plnXInterval, plnYInterval);
                    s2 = new PlaneSurface(pl2, plnXInterval, plnYInterval);
                }

                guide1 = myDoc.Objects.Add(s1);
                guide2 = myDoc.Objects.Add(s2);
                myDoc.Views.Redraw();
            }
        }
        private void Gp_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (selected != Guid.Empty)
                selected = Guid.Empty;
            else
            {
                var p = e.Point;
                double dis1 = Math.Abs(pl1.DistanceTo(p)), dis2 = Math.Abs(pl2.DistanceTo(p));

                List<double> distances = new List<double> { dis1, dis2 };
                double min = distances.Min();
                if (min > 5) return;
                else if (min == dis1)
                {
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(guide1);
                    selected = guide1;
                }
                else if (min == dis2)
                {
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(guide2);
                    selected = guide2;
                }
            }

        }

        private void Gp_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (selected == Guid.Empty)
                return;

            double t = 0, tn = 0;
            skeleton.ClosestPoint(e.Point, out t);
            skeleton.NormalizedLengthParameter(t, out tn);
            if (tn < 0)
                tn = 0;
            if (tn > 1)
                tn = 1;
            if (selected == guide1)
            {
                //calculate where is the mouse and change t
                if (Math.Abs(t1 - tn) > 0.01)
                {
                    //move and update t1
                    Transform m = Transform.Translation(skeletonVec * (tn - t1));
                    guide1 = myDoc.Objects.Transform(guide1, m, true);
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(guide1);
                    pl1.Transform(m);
                    t1 = tn;
                    myDoc.Views.Redraw();
                }

            }
            if (selected == guide2)
            {
                //calculate where is the mouse and change t
                if (Math.Abs(t2 - tn) > 0.01)
                {
                    //move and update t2
                    Transform m = Transform.Translation(skeletonVec * (tn - t2));
                    guide2 = myDoc.Objects.Transform(guide2, m, true);
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(guide2);
                    pl2.Transform(m);
                    t2 = tn;
                    myDoc.Views.Redraw();
                }
            }
        }
    }
    /// <summary>
    /// This class initiates a side selection operation when one of it's instance is constructed. The selection result is one side along the main axis. 
    /// </summary>
    public class SideSelection
    {
        ObjectAttributes blueAttribute;

        RhinoDoc myDoc;
        Curve skeleton;
        Guid motionCtrlPtID1;
        Guid motionCtrlPtID2;
        public Point3d motionCtrlPointSelected;
        /// <summary>
        /// This class initiates a side selection operation when one of it's instance is constructed. The selection result is one side along the main axis
        /// </summary>
        /// <param name="model_skeleton"></param>
        /// <param name="currDoc"></param>
        public SideSelection(Curve model_skeleton, RhinoDoc currDoc, ObjectAttributes _blueAttribute)
        {
            blueAttribute = _blueAttribute;
            myDoc = currDoc;
            skeleton = model_skeleton;
            Rhino.Input.Custom.GetPoint gp3 = new Rhino.Input.Custom.GetPoint();
            gp3.SetCommandPrompt("Select one side to add the motion control.");
            gp3.MouseDown += Gp_MouseDown;
            gp3.MouseMove += Gp_MouseMove;
            //gp3.AcceptNothing(true);
            GenerateMotionControlIndicator();
            Rhino.Input.GetResult r3;
            r3 = gp3.Get(true);

            //do
            //{
            //    r3 = gp3.Get(true);
            //} while (r3 != Rhino.Input.GetResult.Nothing);

            myDoc.Objects.Hide(motionCtrlPtID1, true);
            myDoc.Objects.Hide(motionCtrlPtID2, true);
        }
        void GenerateMotionControlIndicator()
        {
            Point3d lPt = new Point3d();
            Point3d rPt = new Point3d();

            Point3d skePt1 = skeleton.PointAt(0);
            Point3d skePt2 = skeleton.PointAt(1);

            lPt = skePt1;
            rPt = skePt2;

            motionCtrlPtID1 = myDoc.Objects.AddSphere(new Sphere(lPt, 3), blueAttribute);
            motionCtrlPtID2 = myDoc.Objects.AddSphere(new Sphere(rPt, 3), blueAttribute);

            myDoc.Views.Redraw();

        }
        private void Gp_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep lPtBrep = (Brep)myDoc.Objects.Find(motionCtrlPtID1).Geometry;
            Brep rPtBrep = (Brep)myDoc.Objects.Find(motionCtrlPtID2).Geometry;

            double l_dis = lPtBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double r_dis = rPtBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if (l_dis <= 30 || r_dis <= 30)
            {
                if (l_dis <= r_dis)
                {
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(motionCtrlPtID1);
                }
                else
                {
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(motionCtrlPtID2);
                }
            }
            else
            {
                myDoc.Objects.UnselectAll();
            }
        }

        private void Gp_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep lPtBrep = (Brep)myDoc.Objects.Find(motionCtrlPtID1).Geometry;
            Brep rPtBrep = (Brep)myDoc.Objects.Find(motionCtrlPtID2).Geometry;

            double l_dis = lPtBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double r_dis = rPtBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if (l_dis <= 30 || r_dis <= 30)
            {
                if (l_dis <= r_dis)
                {
                    motionCtrlPointSelected = ((Brep)myDoc.Objects.Find(motionCtrlPtID1).Geometry).GetBoundingBox(true).Center;
                }
                else
                {
                    motionCtrlPointSelected = ((Brep)myDoc.Objects.Find(motionCtrlPtID2).Geometry).GetBoundingBox(true).Center;
                }
            }
        }
    }
    /// <summary>
    /// This class initiates a circular direction selection operation when one of it's instance is constructed. The selection result is the direction of end effector. 
    /// </summary>
    public class EndEffectorDirectionSelection
    {
        ObjectAttributes redAttribute;
        public Point3d eeCenPt;
        Curve skeleton;
        RhinoDoc myDoc;
        Guid eeCircleID;
        Point3d eeCircleDotPt;
        public Vector3d kineticUnitDir;
        public Vector3d axelDir;
        Vector3d direction;
        double t1, t2;
        List<Brep> brepCut;
        /// <summary>
        /// This class initiates a circular direction selection operation when one of it's instance is constructed. The selection result is the direction of end effector.
        /// </summary>
        /// <param name="_brepCut">Model as 3 pieces</param>
        /// <param name="model_skeleton">Skeleton of model</param>
        /// <param name="skeleton_direction">Vector from skeleton start to end</param>
        /// <param name="motionCtrlPointSelected">The side of motion control as a point</param>
        /// <param name="_t1"></param>
        /// <param name="_t2"></param>
        /// <param name="currDoc"></param>
        public EndEffectorDirectionSelection(List<Brep> _brepCut,Curve model_skeleton, Vector3d skeleton_direction,Point3d motionCtrlPointSelected,double _t1,double _t2,RhinoDoc currDoc, ObjectAttributes _redAttribute)
        {
            redAttribute = _redAttribute;
            // generate the circle
            skeleton = model_skeleton;
            direction = skeleton_direction;
            myDoc = currDoc;
            Point3d startPt = skeleton.PointAtNormalizedLength(0);
            Point3d endPt = skeleton.PointAtNormalizedLength(1);
            t1 = _t1;
            t2 = _t2;
            brepCut = _brepCut;

            if (t1 > t2)
            {
                endPt = skeleton.PointAtNormalizedLength(t1);
                startPt = skeleton.PointAtNormalizedLength(t2);
            }
            else
            {
                startPt = skeleton.PointAtNormalizedLength(t1);
                endPt = skeleton.PointAtNormalizedLength(t2);
            }

            if (motionCtrlPointSelected.DistanceTo(startPt) <= motionCtrlPointSelected.DistanceTo(endPt))
            {
                eeCenPt = endPt;
            }
            else
            {
                eeCenPt = startPt;
            }

            Curve eeCircleCrv = new Circle(new Plane(eeCenPt, direction), 50).ToNurbsCurve();
            eeCircleID = myDoc.Objects.AddCurve(eeCircleCrv, redAttribute);
            myDoc.Views.Redraw();

            Rhino.Input.Custom.GetPoint gp4 = new Rhino.Input.Custom.GetPoint();
            gp4.SetCommandPrompt("First, select one position to decide the kinetic unit orientation.");
            gp4.MouseDown += Gp4_MouseDown;
            gp4.MouseMove += Gp4_MouseMove;
            gp4.DynamicDraw += Gp4_DynamicDraw;
            //gp4.AcceptNothing(true);
            Rhino.Input.GetResult r4;
            r4 = gp4.Get(true);
            //do
            //{
            //    r4 = gp4.Get(true);
            //} while (r4 != Rhino.Input.GetResult.Nothing);

            myDoc.Objects.Delete(eeCircleID, true);
        }
        private void Gp4_DynamicDraw(object sender, Rhino.Input.Custom.GetPointDrawEventArgs e)
        {
            e.Display.DrawSphere(new Sphere(eeCircleDotPt, 5), Color.FromArgb(16, 150, 206));
        }

        private void Gp4_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Curve eeCircleCrv = (Curve)myDoc.Objects.Find(eeCircleID).Geometry;
            double t;
            eeCircleCrv.ClosestPoint(currPos, out t);
            eeCircleDotPt = eeCircleCrv.PointAt(t);
        }

        private void Gp4_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Curve eeCircleCrv = (Curve)myDoc.Objects.Find(eeCircleID).Geometry;
            double t;
            eeCircleCrv.ClosestPoint(currPos, out t);
            eeCircleDotPt = eeCircleCrv.PointAt(t);
            kineticUnitDir = eeCircleDotPt - eeCenPt;
            kineticUnitDir.Unitize();
            axelDir = kineticUnitDir;
            Transform rot = Transform.Rotation(Math.PI / 2, direction, eeCenPt);
            axelDir.Transform(rot);

            //myDoc.Objects.AddBrep(brepCut[1], orangeAttribute);
            //myDoc.Views.Redraw();

            //myDoc.Objects.AddCurve(skeleton, redAttribute);
            //myDoc.Views.Redraw();

            //Point3d tempPt = skeleton.PointAtEnd;
            //Curve directionCrv = new Line(tempPt, tempPt + direction).ToNurbsCurve();
            //myDoc.Objects.AddCurve(directionCrv, redAttribute);
            //myDoc.Views.Redraw();

            //Curve axelDirectionCrv = new Line(tempPt, tempPt + axelDir).ToNurbsCurve();
            //myDoc.Objects.AddCurve(axelDirectionCrv, redAttribute);
            //myDoc.Views.Redraw();

            //Curve kineticUnitDirectionCrv = new Line(tempPt, tempPt + kineticUnitDir).ToNurbsCurve();
            //myDoc.Objects.AddCurve(kineticUnitDirectionCrv, redAttribute);
            //myDoc.Views.Redraw();

            Curve selectedSegment = null;
            if (t1 > t2)
            {
                selectedSegment = new Line(skeleton.PointAtNormalizedLength(t2), skeleton.PointAtNormalizedLength(t1)).ToNurbsCurve();
            }
            else
            {
                selectedSegment = new Line(skeleton.PointAtNormalizedLength(t1), skeleton.PointAtNormalizedLength(t2)).ToNurbsCurve();
            }

            GetInnerCavitySpaceForDir(brepCut[1], selectedSegment, direction, axelDir, kineticUnitDir);
        }
        void GetInnerCavitySpaceForDir(Brep b, Curve c, Vector3d dir, Vector3d targetDir1, Vector3d targetDir2)
        {
            double target1Space = 0;
            double target2Space = 0;
            double step = 1.0 / 100.0;

            for (int i = 0; i <= 100; i++)
            {
                Point3d cen = c.PointAtNormalizedLength(i * step);
                Plane cenPln = new Plane(cen, dir);

                Curve[] outCrvs;
                Point3d[] outPts;

                Rhino.Geometry.Intersect.Intersection.BrepPlane(b, cenPln, myDoc.ModelAbsoluteTolerance, out outCrvs, out outPts);
                if (outCrvs != null && outCrvs.Count() > 0)
                {
                    Curve intersectCrv = outCrvs[0];

                    double area = 0;

                    double increamental = 0.05;
                    double target1Dis = 0;

                    Curve target1IntersectionCrv1 = new Line(cen, cen + targetDir1 * int.MaxValue).ToNurbsCurve();
                    Curve target1IntersectionCrv2 = new Line(cen, cen - targetDir1 * int.MaxValue).ToNurbsCurve();
                    Point3d intersectPt1 = new Point3d();
                    Point3d intersectPt2 = new Point3d();

                    var events1 = Rhino.Geometry.Intersect.Intersection.CurveCurve(intersectCrv, target1IntersectionCrv1, myDoc.ModelAbsoluteTolerance, myDoc.ModelAbsoluteTolerance);
                    if (events1 != null && events1.Count > 0)
                    {
                        intersectPt1 = events1[0].PointA;
                    }
                    var events2 = Rhino.Geometry.Intersect.Intersection.CurveCurve(intersectCrv, target1IntersectionCrv2, myDoc.ModelAbsoluteTolerance, myDoc.ModelAbsoluteTolerance);
                    if (events2 != null && events2.Count > 0)
                    {
                        intersectPt2 = events2[0].PointA;
                    }

                    if (intersectPt1.DistanceTo(cen) <= intersectPt2.DistanceTo(cen))
                    {
                        target1Dis = intersectPt1.DistanceTo(cen);
                    }
                    else
                    {
                        target1Dis = intersectPt2.DistanceTo(cen);
                    }

                    for (double p = 0; p <= target1Dis; p = p + increamental)
                    {
                        Point3d currPt = cen + targetDir1 * p;

                        Curve target2IntersectionCrv1 = new Line(currPt, currPt + targetDir2 * int.MaxValue).ToNurbsCurve();
                        Curve target2IntersectionCrv2 = new Line(currPt, currPt - targetDir2 * int.MaxValue).ToNurbsCurve();
                        Point3d intersectTarget2Pt1 = new Point3d();
                        Point3d intersectTarget2Pt2 = new Point3d();
                        double target2Dis = 0;

                        var events3 = Rhino.Geometry.Intersect.Intersection.CurveCurve(intersectCrv, target2IntersectionCrv1, myDoc.ModelAbsoluteTolerance, myDoc.ModelAbsoluteTolerance);
                        if (events3 != null && events3.Count > 0)
                        {
                            intersectTarget2Pt1 = events3[0].PointA;
                        }
                        var events4 = Rhino.Geometry.Intersect.Intersection.CurveCurve(intersectCrv, target2IntersectionCrv2, myDoc.ModelAbsoluteTolerance, myDoc.ModelAbsoluteTolerance);
                        if (events4 != null && events4.Count > 0)
                        {
                            intersectTarget2Pt2 = events4[0].PointA;
                        }

                        if (intersectTarget2Pt1.DistanceTo(currPt) <= intersectTarget2Pt2.DistanceTo(currPt))
                        {
                            target2Dis = intersectTarget2Pt1.DistanceTo(currPt);
                        }
                        else
                        {
                            target2Dis = intersectTarget2Pt2.DistanceTo(currPt);
                        }

                        double currArea = targetDir1 * targetDir2;
                        if (currArea >= area)
                        {
                            area = currArea;
                            target1Space = 2 * p;
                            target2Space = 2 * target2Dis;
                        }

                    }
                }
                else
                {
                    continue;
                }
            }
        }

    }
    public class PointOnLineSelection
    {
        RhinoDoc myDoc;
        Guid eeLineID;
        public Point3d eeLineDotPt;
        public PointOnLineSelection(Curve eeLineCrv,RhinoDoc currDoc,ObjectAttributes redAttribute)
        {
            myDoc = currDoc;
            eeLineID = myDoc.Objects.AddCurve(eeLineCrv, redAttribute);
            myDoc.Views.Redraw();

            Rhino.Input.Custom.GetPoint gp5 = new Rhino.Input.Custom.GetPoint();
            gp5.SetCommandPrompt("Second, select one position to decide the position of the end-effector.");
            gp5.MouseDown += Gp5_MouseDown;
            gp5.MouseMove += Gp5_MouseMove;
            gp5.DynamicDraw += Gp5_DynamicDraw;
            //gp5.AcceptNothing(true);
            Rhino.Input.GetResult r5;
            r5 = gp5.Get(true);

            //do
            //{
            //    r5 = gp5.Get(true);
            //} while (r5 != Rhino.Input.GetResult.Nothing);

            myDoc.Objects.Delete(eeLineID, true);
        }
        private void Gp5_DynamicDraw(object sender, Rhino.Input.Custom.GetPointDrawEventArgs e)
        {
            e.Display.DrawSphere(new Sphere(eeLineDotPt, 5), Color.FromArgb(16, 150, 206));
        }

        private void Gp5_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Curve eeLineCrv = (Curve)myDoc.Objects.Find(eeLineID).Geometry;
            double t;
            eeLineCrv.ClosestPoint(currPos, out t);
            eeLineDotPt = eeLineCrv.PointAt(t);
        }

        private void Gp5_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Curve eeLineCrv = (Curve)myDoc.Objects.Find(eeLineID).Geometry;
            double t;
            eeLineCrv.ClosestPoint(currPos, out t);
            eeLineDotPt = eeLineCrv.PointAt(t);
        }
    }


}
