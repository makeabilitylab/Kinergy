using System;
using System.Collections.Generic;
using System.IO;
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
using Kinergy.Utilities;
using Kinergy.Geom;
using Kinergy.Relationship;
using Kinergy.KineticUnit;
using Kinergy;
using System.Diagnostics;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace Kinergy.KineticUnit
{
    public class InstantTranslation : KineticUnit
    {
        //The initial inputs
        private Brep model;
        private bool curved = false;
        private double energy;//energy is above 0 and less than 10,indicating the total energy stored in fully compressed spring
        private double distance;//distance is between 0.2 and 0.8,indicating the ratio of spring travel out of length;
        private Vector3d direction = Vector3d.Unset;
        private bool addLock;
        RhinoDoc myDoc;

        //The later selected parameters
        private Point3d springPosition = Point3d.Unset;
        private double springT, springStart, springEnd;
        private int lockDirection = 0;//0:unset;1:lock at left;2:lock at right;
        private Point3d lockPosition = Point3d.Unset;
        private double lockT;

        //The calculated geometry and parameters
        Vector3d skeletonVector;//start to end;
        //private Transform xrotate;
        //private Transform xrotateBack;
        private double springRadius = 0;
        private double wireRadius = 0;
        private double springLength = 0;
        private int roundNum = 0;
        private Interval skeletonAvailableRange;

        private Curve skeleton = null;
        private List<Shape> modelCut;
        private List<Lock> locks;
        private Helix spring;
        private List<Point3d> LockPositionCandidates=new List<Point3d>();
        private List<double> LockTCandidates = new List<double>();
        public Helix Spring { get => spring; private set => spring = value; }
        public bool Curved { get => curved; set => curved = value; }
        public Curve Skeleton { get => skeleton; protected set => skeleton = value; }
        public Brep Model { get => model; set => model = value; }
        public List<Shape> ModelCut { get => modelCut; set => modelCut = value; }

        /// <summary> Default constructor without basic input parameter </summary>
        /// <returns> Returns empty instance</returns>

        public InstantTranslation(Brep Model, bool Curved ,Vector3d Direction, double Energy, double Distance)
        {
            model = Model;
            energy = Energy;
            distance = Distance;
            direction = Direction;
            curved = Curved;
            //xrotate = Transform.Rotation(direction, Vector3d.XAxis, Point3d.Origin);//Deprecated. Too ambiguous and indirect.
            //xrotateBack = Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin);
            modelCut = new List<Shape>();
            myDoc = RhinoDoc.ActiveDoc;
            locks = new List<Lock>();
        }
        
        /// <summary> Call this method to  </summary>
        /// <returns> Returns bool value showing whether all processes go well</returns>
        public bool Process()
        {
            string path3 = System.IO.Directory.GetCurrentDirectory();
            RhinoApp.WriteLine(path3);
            RhinoApp.WriteLine(FileOperation.FindComponentFolderDirectory());

            if (CalculateStraightSkeleton(0,1,model) == false)
            { throw new Exception("Unable to process this model,please provide valid model and vector"); }

            if (SetSpringPosition() == false)
            { throw new Exception("Failed to set spring position."); }

            if (CutModelForSpring() == false)
            { throw new Exception("Failed to cut model."); }

            if (ConstructSpring() == false)
            { throw new Exception("Failed to build spring."); }

            if (addLock)
            {
                if (SetLockDirection() == false)
                { throw new Exception("Failed to select lock direction."); }

                if (SetLockPosition() == false)
                { throw new Exception("Failed to select lock position."); }

                if (lockPosition == Point3d.Unset)
                { return true; }

                if (CutModelForLock() == false)
                { throw new Exception("Failed to select cut holes for lock."); }

                //if (ConstructLock(1) == false)
                //{ throw new Exception("Failed to build lock structure."); }
            }
            return true;
        }
        public bool SetSpringPosition()
        {
            List<Point3d> points = new List<Point3d>();
            for (int i = 0; i <= 20; i++)
            {
                points.Add(skeleton.PointAtNormalizedLength(skeletonAvailableRange.Min + skeletonAvailableRange.Length * i / 20));

            }

            Point3d selectedPosition = points[UserSelection.UserSelectPointInRhino(points, myDoc)];
            springPosition = selectedPosition;
            double T = 0;
            skeleton.ClosestPoint(springPosition, out T);
            springT = skeleton.Domain.NormalizedParameterAt(T);
            springStart = springT - springLength / 2 / skeleton.GetLength();
            springEnd = springT + springLength / 2 / skeleton.GetLength();

            if (springPosition != Point3d.Unset)
            { return true; }
            else
            { return false; }
        }
        public bool SetSpringPosition(Point3d pos)
        {
            springPosition = pos;
            double T = 0;
            skeleton.ClosestPoint(springPosition, out T,0.5);
            springT = skeleton.Domain.NormalizedParameterAt(T);
            // springLength= GetSectionRadius(springT) / 7.5 * 25;
            springStart = springT - springLength / 2 / skeleton.GetLength();
            springEnd = springT + springLength / 2 / skeleton.GetLength();

            return true;
        }
        public List<Point3d> GetSpringPositionCandidates()
        {
            List<Point3d> points = new List<Point3d>();
            if(curved)
            {
                for (int i = 0; i <= 20; i++)
                {
                    Vector3d d = GeometryMethods.AverageTangent(skeleton, skeletonAvailableRange.Min + skeletonAvailableRange.Length * i / 20);
                    if (Math.Abs(d * direction / d.Length / direction.Length) > 0.8)
                    { points.Add(skeleton.PointAtNormalizedLength(skeletonAvailableRange.Min + skeletonAvailableRange.Length * i / 20)); }
                }
            }
            else
            {
                for (int i = 0; i <= 20; i++)
                {
                    points.Add(skeleton.PointAtNormalizedLength(skeletonAvailableRange.Min + skeletonAvailableRange.Length * i / 20));
                }
            }
            
            return points;
        }
        public bool SetLockDirection()
        {
            var xrotate1 = Transform.Rotation(Vector3d.XAxis, -Vector3d.XAxis, Point3d.Origin);
            var move = Transform.Translation(new Vector3d(springLength / 2, springLength, 0));
            Curve A1 = Arrow.ConstructArrow();
            Curve A2 = Arrow.ConstructArrow();
            A1.Transform(move);
            A2.Transform(move);
            A1.Transform(xrotate1);
            //move arrows to spring position
            Transform toSpringCoordinate = Transform.Multiply(Transform.Translation(new Vector3d(springPosition)), Transform.Rotation(Vector3d.XAxis, skeletonVector, springPosition));
            A1.Transform(toSpringCoordinate);
            A2.Transform(toSpringCoordinate);
            List<Curve> crvs = new List<Curve>();
            crvs.Add(A1);
            crvs.Add(A2);
            RhinoApp.WriteLine("Please select the direction of spring Lock");
            lockDirection = UserSelection.UserSelectCurveInRhino(crvs, myDoc) + 1;

            return true;
        }
        public bool SetLockDirection(Arrow d)
        {
            Point3d p1 = skeleton.PointAtNormalizedLength(0);
            Point3d p2 = skeleton.PointAtNormalizedLength(1);
            direction  = new Vector3d(p2) - new Vector3d(p1);
            if (direction * d.Direction > 0.5)
            {
                lockDirection = 1;
            }
            else
            {
                lockDirection = 2;
            }
            return true;
        }
        public bool SetLockPosition()
        {

            Vector3d v = skeletonVector;
            double start = 0, end = 0;
            Brep B;
            if (lockDirection == 1)//the lock structure start from the left and stick into the right part
            {
                start = springEnd + 5 / skeleton.GetLength();
                end = 1 - 5 / skeleton.GetLength();
                B = modelCut[1].Model;
                //In this case, the moving end of spring should be reversed.
            }
            else if (lockDirection == 2)//the lock structure start from the right and stick into the left part
            {
                end = springStart - 5 / skeleton.GetLength();
                start = 5 / skeleton.GetLength();
                B = modelCut[0].Model;
            }
            else { return false; }
            double span = end - start, span_len = span * skeleton.GetLength();
            List<Point3d> candidates = new List<Point3d>();
            for (int i = 0; i < span_len / 2; i++)
            {
                Point3d p = skeleton.PointAtNormalizedLength(start + i * 2 / skeleton.GetLength());
                Plane plane = new Plane(p, v);
                Curve[] c;
                Point3d[] pt;
                Rhino.Geometry.Intersect.Intersection.BrepPlane(B, plane, 0.0000001, out c, out pt);
                for (int j = 0; j < 10; j++)
                {
                    candidates.Add(c[0].PointAtNormalizedLength(j * 0.1));
                }
            }
            if (candidates.Count == 0)
            {
                RhinoApp.WriteLine("No space for lock!");
                return true;
            }

            int selectedIndex = UserSelection.UserSelectPointInRhino(candidates, myDoc);
            lockPosition = candidates[selectedIndex];
            skeleton.ClosestPoint(candidates[selectedIndex], out lockT);
            if (lockPosition == Point3d.Unset)
            { return false; }
            return true;
        }
        public List<Point3d> GetLockPositionCandidates()
        {
            double start = 0, end = 0;
            Point3d p1 = Point3d.Unset;
            Vector3d vstart=Vector3d.Unset;
            Brep B;
            if (lockDirection == 1)//the lock structure start from the left and stick into the right part
            {
                start = springEnd + 5 / skeleton.GetLength();
                end = 1 - 5 / skeleton.GetLength();
                B = modelCut[1].Model;
                p1 = skeleton.PointAtNormalizedLength(springEnd);
                vstart = GeometryMethods.AverageTangent(skeleton, springEnd);
            }
            else //the lock structure start from the right and stick into the left part
            {
                start = springStart - 5 / skeleton.GetLength();
                end = 5 / skeleton.GetLength();
                B = modelCut[0].Model;
                p1 = skeleton.PointAtNormalizedLength(springStart);
                vstart = GeometryMethods.AverageTangent(skeleton, springStart);
            }
            double span = end - start, span_len =Math.Abs( span * skeleton.GetLength());
            int count =(int)Math.Round( span_len / 2);
            List<Point3d> candidates = new List<Point3d>();
            for (int i = 0; i < count; i++)
            {
                Point3d p = skeleton.PointAtNormalizedLength(start + span*i/count);
                Vector3d v = GeometryMethods.AverageTangent(skeleton, start + span * i / count);
                if(Math.Abs(v*vstart/v.Length/vstart.Length)<0.85)//Make sure lock never get too bended
                { break; }
                Plane plane = new Plane(p, v);
                Curve[] c;
                Point3d[] pt;
                Rhino.Geometry.Intersect.Intersection.BrepPlane(B, plane,myDoc.ModelAbsoluteTolerance, out c, out pt);
                for (int j = 0; j < 10; j++)
                {
                    candidates.Add(c[0].PointAtNormalizedLength(j * 0.1));
                    LockPositionCandidates.Add(c[0].PointAtNormalizedLength(j * 0.1));
                    LockTCandidates.Add(start + span * i / count);
                }
            }
            if (candidates.Count == 0)
            {
                RhinoApp.WriteLine("No space for lock!");
            }
            return candidates;

        }
        public void SetLockPosition(Point3d p)
        {
            lockPosition = p;
            for (int i= 0;i < LockPositionCandidates.Count();i++)
            {
                Point3d pt = LockPositionCandidates[i];
                if(pt.X==p.X && pt.Y==p.Y && pt.Z==p.Z)
                {
                    lockT = LockTCandidates[i];
                    break;
                }
            }
            //double t;
            //skeleton.ClosestPoint(p, out t);
            //lockT= skeleton.Domain.NormalizedParameterAt(t);
        }
        public List<Arrow> GetLockDirectionCandidates()
        {
            if(curved==false)
            {
                Vector3d v1 = direction;
                Vector3d v2 = -v1;
                Point3d p1 = springPosition + v1 * (springLength / 2 + 5)/v1.Length;
                Point3d p2 = springPosition - v1 * (springLength / 2 + 5)/v1.Length;
                Arrow a1 = new Arrow(v1, p1);
                Arrow a2 = new Arrow(v2, p2);
                return new List<Arrow> { a1, a2 };
            }
            else
            {
                Point3d p1 = skeleton.PointAtNormalizedLength(0);
                Point3d p2 = skeleton.PointAtNormalizedLength(1);
                Vector3d v1 = new Vector3d(p2)-new Vector3d(p1);
                Vector3d v2 = -v1;
                Point3d pos1 = p2 + v1/v1.Length * (springLength / 2 + 5);
                Point3d pos2 = p1 - v1/v1.Length * (springLength / 2 + 5);
                Arrow a1 = new Arrow(v1, pos1);
                Arrow a2 = new Arrow(v2, pos2);
                return new List<Arrow> { a1, a2 };
            }
            
        }
        public bool CalculateStraightSkeleton(double t1, double t2, Brep selectedModel)
        {
            Transform xrotate = Transform.Rotation(direction, Vector3d.XAxis, Point3d.Origin);//Deprecated. Too ambiguous and indirect.
            Transform xrotateBack = Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin);
            //here skeleton is calculated using bbox. Spring parameters are now determined by shape and ratio of bbox. energy and distance havn't been adopted.
            model.Transform(xrotate);
            selectedModel.Transform(xrotate);
            BoundingBox box = model.GetBoundingBox(true);
            BoundingBox box_sel = selectedModel.GetBoundingBox(true);
            model.Transform(xrotateBack);
            selectedModel.Transform(xrotateBack);
            Point3d p1 = new Point3d(box.Min.X, (box.Min.Y + box.Max.Y) / 2, (box.Min.Z + box.Max.Z) / 2), p2 = new Point3d(box.Max.X, (box.Min.Y + box.Max.Y) / 2, (box.Min.Z + box.Max.Z) / 2);
            Line l = new Line(p1, p2);

            #region old spring length and diameter
            //springLength = springRadius / 7.5 * 25;
            //if (springLength > l.Length * 0.5)//the model is too short,so decrease spring radius
            //{
            //    springLength = l.Length * 0.5;
            //    springRadius = springLength * 7.5 / 25 * 0.9;
            //}
            #endregion

            /*
            if (springLength > l.Length * 0.5)//the model is too short,so decrease spring radius
            {
                springLength = l.Length * 0.7;
                springRadius = springLength * 7.5 / 25 * 0.9;
            }
            if (springLength > l.Length * 0.5)//the model is too short,so decrease spring radius
            {
                springLength = l.Length * 0.8;
                springRadius = springLength * 7.5 / 25 * 0.9;
            }*/
            

            skeleton = l.ToNurbsCurve();
            skeleton.Transform(xrotateBack);

            Point3d stPt = skeleton.PointAtNormalizedLength(t1);
            Point3d endPt = skeleton.PointAtNormalizedLength(t2);
            springLength = stPt.DistanceTo(endPt);

            //springRadius = Math.Min(box_sel.Max.Y - box_sel.Min.Y, box_sel.Max.Z - box_sel.Min.Z) * 0.9;
            //springRadius = Math.Min(model.ClosestPoint(stPt).DistanceTo(stPt), model.ClosestPoint(endPt).DistanceTo(endPt)) * 2;
            Vector3d planeNormal= new Vector3d(stPt - endPt);
            Plane firstPtPlane = new Plane(stPt, planeNormal);
            Plane secondPtPlane = new Plane(endPt, planeNormal);

            Curve[] intersectStart;
            Point3d[] intersectStartPts;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(model, firstPtPlane, myDoc.ModelAbsoluteTolerance, out intersectStart, out intersectStartPts);
            Curve strCrv = intersectStart[0];

            #region test by LH
            //myDoc.Objects.AddCurve(strCrv);
            //myDoc.Views.Redraw();
            #endregion

            Curve[] intersectEnd;
            Point3d[] intersectEndPts;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(model, secondPtPlane, myDoc.ModelAbsoluteTolerance, out intersectEnd, out intersectEndPts);
            Curve endCrv = intersectEnd[0];

            #region test by LH
            //myDoc.Objects.AddCurve(endCrv);
            //myDoc.Views.Redraw();
            #endregion

            double pos1, pos2;
            strCrv.ClosestPoint(stPt, out pos1);
            endCrv.ClosestPoint(endPt, out pos2);

            springRadius = Math.Min(strCrv.PointAt(pos1).DistanceTo(stPt), endCrv.PointAt(pos2).DistanceTo(endPt)) * 1.5;


            //wireRadius = springRadius / 7.5 * 1;
            wireRadius = 2.8;
            skeletonAvailableRange = new Interval((springLength / 2 + 5) / l.Length, 1 - (springLength / 2 + 5) / l.Length);


            string body = string.Format("The skeleton is from {0},{1},{2} to {3},{4},{5}", skeleton.PointAtStart.X, skeleton.PointAtStart.Y, skeleton.PointAtStart.Z, skeleton.PointAtEnd.X, skeleton.PointAtEnd.Y, skeleton.PointAtEnd.Z);
            RhinoApp.WriteLine(body);
            body = string.Format("The skeletonAvailableRange is from {0} to {1},length is {2}", skeletonAvailableRange.Min, skeletonAvailableRange.Max, skeletonAvailableRange.Length);
            RhinoApp.WriteLine(body);
            skeletonVector = new Vector3d(skeleton.PointAtEnd) - new Vector3d(skeleton.PointAtStart);
            if (skeleton.GetLength() > 0)
            {
                return true;

            }
            return false;
        }

        public bool CalculateCurvedSkeleton()
        {
            if (curved)
            {
                SkeletonGen();
                
                double springLength1 = GetSectionRadius(0.1) / 7.5 * 25;
                double springLength2 = GetSectionRadius(0.9) / 7.5 * 25;
                if (springLength1 > skeleton.GetLength() * 0.4)//the model is too short,so decrease spring radius
                {
                    springLength1 = skeleton.GetLength() * 0.4;
                }
                if (springLength2 > skeleton.GetLength() * 0.4)//the model is too short,so decrease spring radius
                {
                    springLength2 = skeleton.GetLength() * 0.4;
                }
                skeletonAvailableRange = new Interval((springLength1 / 2 + 5) / skeleton.GetLength(), 1 - (springLength2 / 2 + 5) / skeleton.GetLength());
                return true;
            }
            else
                return false;
        }
        public bool CutModelForSpring()
        {
            BoundingBox box = model.GetBoundingBox(true);

            //Vector3d tan1 = skeleton.TangentAt(skeleton.Domain.ParameterAt(springStart));
            Vector3d tan1 = Kinergy.Utilities.GeometryMethods.AverageTangent(skeleton, springStart);
            //Vector3d tan2 = skeleton.TangentAt(skeleton.Domain.ParameterAt(springEnd));
            Vector3d tan2 = Kinergy.Utilities.GeometryMethods.AverageTangent(skeleton, springEnd);
            Plane plane2 = new Rhino.Geometry.Plane(skeleton.PointAtNormalizedLength(springEnd), -tan2);
            Plane plane1 = new Rhino.Geometry.Plane(skeleton.PointAtNormalizedLength(springStart), tan1);
            plane1.ExtendThroughBox(box, out Interval s1, out Interval t1);
            plane2.ExtendThroughBox(box, out Interval s2, out Interval t2);

            //int redIndex = myDoc.Materials.Add();
            //Rhino.DocObjects.Material redMat = myDoc.Materials[redIndex];
            //redMat.DiffuseColor = System.Drawing.Color.Red;
            //redMat.SpecularColor = System.Drawing.Color.Red;
            //redMat.CommitChanges();
            //ObjectAttributes redAttribute;
            //redAttribute = new ObjectAttributes();
            //redAttribute.LayerIndex = 1;
            //redAttribute.MaterialIndex = redIndex;
            //redAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            //myDoc.Objects.AddBrep(model, redAttribute);
            //myDoc.Views.Redraw();

            Brep[] Cut_Brep1 = model.Trim(plane1, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep[] Cut_Brep2 = model.Trim(plane2, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

            Shape mc1 = new Shape(Cut_Brep1[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance), false, "model");
            Shape mc2 = new Shape(Cut_Brep2[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance), false, "model");
            modelCut.Add(mc1);
            modelCut.Add(mc2);
            entityList.Add(mc1);
            entityList.Add(mc2);
            return true;
        }
        public bool ConstructSpring()
        {
            if(curved==true)
            {
                Point3d startPoint = skeleton.PointAtNormalizedLength(springStart);
                Point3d endPoint = skeleton.PointAtNormalizedLength(springEnd);
                spring = new Helix(skeleton, springStart, springEnd, springRadius, wireRadius, roundNum, distance,energy);
            }
            else
            {
                Point3d startPoint = skeleton.PointAtNormalizedLength(springStart);
                Point3d endPoint = skeleton.PointAtNormalizedLength(springEnd);
                spring = new Helix(startPoint, endPoint, springRadius, wireRadius, roundNum, distance,energy);
                //spring = new Helix(skeleton, springStart, springEnd, springRadius, wireRadius, roundNum, distance, energy);
            }
            EntityList.Add(spring);
            if (spring.Model != null)
            {
                //Register constraints for spring
                _ = new Fixation(spring, modelCut[0]);
                _ = new Fixation(spring, modelCut[1]);
                return true;
            }
            else { return false; }
        }
        public bool CutModelForLock(int type=1)
        {
            if(type==1)
            {
                Point3d p_inside = skeleton.PointAtNormalizedLength(lockT);
                Vector3d lockBaseVector = new Vector3d(lockPosition) - new Vector3d(p_inside);
                double scale = spring.SpringRadius / 7.5;
                double box1_x = 8 * scale, box1_y = 4 * scale, box2_x = 6 * scale, box2_y = 6 * scale;//box1 starts from p1 and end at point param;box2 starts from selected position
                double box2_z = new Line(lockPosition, p_inside).Length * 2.1;
                Vector3d xAxis = Vector3d.XAxis;
                Vector3d yAxis = Vector3d.YAxis;
                Vector3d zAxis = Vector3d.ZAxis;
                Rhino.Geometry.Box box1, box2;//boxes for cutting
                Point3d p1 = skeleton.PointAtNormalizedLength(springStart);
                Point3d p2 = skeleton.PointAtNormalizedLength(springEnd);
                if (lockT < springStart)
                {
                    //make sure base planes of boxes are generated at right coordinates to avoid further transformation
                    Vector3d v1 = new Vector3d(p_inside) - new Vector3d(p1);
                    //Vector3d v1 = GeometryMethods.AverageTangent(skeleton, lockT);
                    //Plane basePlane1 = new Plane(p1, lockBaseVector, new Plane(p1, lockBaseVector, v1).Normal);
                    Plane basePlane1 = new Plane(p1, v1);
                    Vector3d targetX = lockBaseVector - lockBaseVector * basePlane1.Normal * basePlane1.Normal / basePlane1.Normal.Length;
                    Transform rotate = Transform.Rotation(basePlane1.XAxis, targetX, p1);
                    basePlane1.Transform(rotate);
                    Plane basePlane2 = new Plane(p_inside, v1, new Plane(p_inside, lockBaseVector, v1).Normal);
                    //Curve rec = GeometryMethods.Rectangle(p1, GeometryMethods.AverageTangent(skeleton, springStart),
                    //lockBaseVector, new Interval(-box1_x / 2, box1_x / 2), new Interval(-box1_y / 2, box1_y / 2));
                    //sweep
                    //Curve rail = skeleton.ToNurbsCurve(new Interval( lockT,springStart+0.1));//Check if other type of t is needed here
                    //Curve rail = skeleton.ToNurbsCurve(new Interval(skeleton.Domain.ParameterAt( lockT),skeleton.Domain.ParameterAt( springStart + 0.1)));
                    //Brep[] sweep_shape = Rhino.Geometry.Brep.CreateFromSweep(rail, rec, true, myDoc.ModelAbsoluteTolerance);
                    //Brep b = sweep_shape[0];
                    //if (BrepSolidOrientation.Inward == b.SolidOrientation)
                    //{ b.Flip(); }
                    box1 = new Rhino.Geometry.Box(basePlane1, new Interval(-box1_x / 2, box1_x / 2), new Interval(-box1_y / 2, box1_y / 2), new Interval(-v1.Length, v1.Length));
                    box2 = new Rhino.Geometry.Box(basePlane2, new Interval(-box2_x / 2, box2_x / 2), new Interval(-box2_y / 2, box2_y / 2), new Interval(-box2_z / 2, box2_z / 2));
                    BoundingBox bbox1 = modelCut[0].Model.GetBoundingBox(true);
                    BoundingBox bbox2 = box2.ToBrep().GetBoundingBox(true);
                    //BoundingBox bbox3 = b.GetBoundingBox(true);
                    modelCut[0].SetModel(Brep.CreateBooleanDifference(modelCut[0].Model, box1.ToBrep(), myDoc.ModelAbsoluteTolerance)[0]);
                    modelCut[0].SetModel(Brep.CreateBooleanDifference(modelCut[0].Model, box2.ToBrep(), myDoc.ModelAbsoluteTolerance)[0]);
                }
                else
                {
                    Vector3d v1 = new Vector3d(p_inside) - new Vector3d(p2);
                    //Vector3d v1 = -GeometryMethods.AverageTangent(skeleton, lockT);
                    //Plane basePlane1 = new Plane(p2, lockBaseVector, new Plane(p2, lockBaseVector, v1).Normal);
                    Plane basePlane1 = new Plane(p2, v1);
                    Vector3d targetX = lockBaseVector - lockBaseVector * basePlane1.Normal * basePlane1.Normal / basePlane1.Normal.Length;
                    Transform rotate = Transform.Rotation(basePlane1.XAxis, targetX, p2);
                    basePlane1.Transform(rotate);
                    Plane basePlane2 = new Plane(p_inside, v1, new Plane(p_inside, lockBaseVector, v1).Normal);
                    box1 = new Rhino.Geometry.Box(basePlane1, new Interval(-box1_x / 2, box1_x / 2), new Interval(-box1_y / 2, box1_y / 2), new Interval(-v1.Length, v1.Length));
                    //Curve rec =GeometryMethods.Rectangle(p2,GeometryMethods.AverageTangent(skeleton, springEnd),
                    //lockBaseVector, new Interval(-box1_x / 2, box1_x / 2), new Interval(-box1_y / 2, box1_y / 2));
                    //sweep
                    //Curve rail = skeleton.ToNurbsCurve(new Interval(springEnd-0.1,lockT));//Check if other type of t is needed here
                    //Curve rail = skeleton.ToNurbsCurve(new Interval(skeleton.Domain.ParameterAt(springEnd - 0.1), skeleton.Domain.ParameterAt(lockT)));
                    //Brep[] sweep_shape = Rhino.Geometry.Brep.CreateFromSweep(rail, rec, true, myDoc.ModelAbsoluteTolerance);
                    //Brep b = sweep_shape[0];
                    //if (BrepSolidOrientation.Inward == b.SolidOrientation)
                    //{ b.Flip(); }
                    box2 = new Rhino.Geometry.Box(basePlane2, new Interval(-box2_x / 2, box2_x / 2), new Interval(-box2_y, box2_y), new Interval(-box2_z / 2, box2_z / 2));
                    BoundingBox bbox1 = modelCut[1].Model.GetBoundingBox(true);
                    BoundingBox bbox2 = box2.ToBrep().GetBoundingBox(true);
                    //BoundingBox bbox3 = b.GetBoundingBox(true);
                    modelCut[1].SetModel(Brep.CreateBooleanDifference(modelCut[1].Model, box1.ToBrep(), myDoc.ModelAbsoluteTolerance)[0]);
                    modelCut[1].SetModel(Brep.CreateBooleanDifference(modelCut[1].Model, box2.ToBrep(), myDoc.ModelAbsoluteTolerance)[0]);
                    //modelCut[1] = new Shape(Brep.CreateBooleanDifference(modelCut[1].Model, box1.ToBrep(), 0.000001)[0]);
                    // modelCut[1] = new Shape( Brep.CreateBooleanDifference(modelCut[1].Model, box2.ToBrep(), 0.000001)[0]);
                }
            }
            else if(type==2)
            {
                //The easier version, just use a little triangle shape to cut model at lock position
                Point3d p_inside = skeleton.PointAtNormalizedLength(lockT);
                Vector3d lockBaseVector = new Vector3d(lockPosition) - new Vector3d(p_inside);
                springRadius = spring.SpringRadius;
                double scale = springRadius / 7.5;
                double lock_radius = 1.5 * scale;
                double lock_length;
                //directory for test
                RhinoApp.WriteLine(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockHeadInstantExtension.3dm");
                Brep LockHead = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockHeadInstantTranslation2.3dm");
                //Brep LockBase = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockBaseInstantExtension.3dm");
                Cylinder rod = Cylinder.Unset;
                if (lockT < springStart)
                {
                    lock_length = skeleton.GetLength() * (springEnd - springStart);
                    lock_length += skeleton.GetLength() * (springStart - lockT);
                    lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;
                    Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength() - 0.001)) - new Vector3d(skeleton.PointAtNormalizedLength(springEnd + 0.03));
                    
                    //Brep[] sweep_shape = Rhino.Geometry.Brep.CreateFromSweep(rail, c.ToNurbsCurve(), true, myDoc.ModelAbsoluteTolerance);
                    //rod = sweep_shape[0];
                    //Move and rotate two models
                    Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength())) + lockBaseVector);
                    LockHead.Transform(move1);
                    Transform rotate1 = Transform.Rotation(Vector3d.YAxis,
                        GeometryMethods.AverageTangent(skeleton, springEnd - lock_length / skeleton.GetLength()),
                        skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
                    LockHead.Transform(rotate1);
                    Vector3d new_Xaxis = Vector3d.XAxis;
                    new_Xaxis.Transform(rotate1);
                    Transform rotate2 = Transform.Rotation(new_Xaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
                    LockHead.Transform(rotate2);
                    modelCut[0].SetModel(Brep.CreateBooleanDifference(modelCut[0].Model, LockHead, myDoc.ModelAbsoluteTolerance)[0]);
                }
                else
                {
                    lock_length = skeleton.GetLength() * (springEnd - springStart);
                    lock_length += skeleton.GetLength() * (lockT - springEnd);
                    lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;
                    Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + 0.001)) - new Vector3d(skeleton.PointAtNormalizedLength(springStart - 0.03));
                    
                    Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength())) + lockBaseVector);
                    LockHead.Transform(move1);
                    Transform rotate1 = Transform.Rotation(Vector3d.YAxis,
                        -GeometryMethods.AverageTangent(skeleton, springStart + lock_length / skeleton.GetLength()),
                        skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
                    LockHead.Transform(rotate1);
                    Vector3d new_Xaxis = Vector3d.XAxis;
                    new_Xaxis.Transform(rotate1);
                    Transform rotate2 = Transform.Rotation(new_Xaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
                    LockHead.Transform(rotate2);
                    modelCut[1].SetModel(Brep.CreateBooleanDifference(modelCut[1].Model, LockHead, myDoc.ModelAbsoluteTolerance)[0]);
                }

            }
            return true;
        }
        public bool ConstructLock(int type, GH_Document gh_d)
        {
            #region New version: buckle-like lock

            Point3d p_inside = skeleton.PointAtNormalizedLength(lockT);
            double lock_length;

            Point3d stPt = skeleton.PointAtNormalizedLength(springEnd + 0.01);
            Point3d endPt = skeleton.PointAtNormalizedLength(springStart - 0.01);

            Vector3d planeNormal = new Vector3d(stPt - endPt);
            Plane stPtPlane = new Plane(stPt, planeNormal);
            Plane endPtPlane = new Plane(endPt, planeNormal);

            Curve[] intersectStart;
            Point3d[] intersectStartPts;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(model, stPtPlane, myDoc.ModelAbsoluteTolerance, out intersectStart, out intersectStartPts);
            Curve strCrv = intersectStart[0];

            Curve[] intersectEnd;
            Point3d[] intersectEndPts;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(model, endPtPlane, myDoc.ModelAbsoluteTolerance, out intersectEnd, out intersectEndPts);
            Curve endCrv = intersectEnd[0];

            double pos1, pos2;
            strCrv.ClosestPoint(stPt, out pos1);
            endCrv.ClosestPoint(endPt, out pos2);

            double offset = Math.Max(strCrv.PointAt(pos1).DistanceTo(stPt), endCrv.PointAt(pos2).DistanceTo(endPt)) + 1.5;


            Vector3d lockBaseVector = new Vector3d(lockPosition) - new Vector3d(p_inside);
            lockBaseVector = lockBaseVector / lockBaseVector.Length * offset;

            string path = "";
            string path_base = "";
            if (gh_d.IsFilePathDefined)
            {
                path = gh_d.FilePath;
                path_base = gh_d.FilePath;

                int idx = path.IndexOf("KinergyMainInterface.gh");
                path = path.Substring(0, idx) + @"KinergyApp\KinergyApp\Resources\lockhead2.3dm";
                path_base = path.Substring(0, idx) + @"KinergyApp\KinergyApp\Resources\bucket.3dm";

            }

            RhinoApp.WriteLine(FileOperation.FindCurrentFolderResourceDirectory() + "\\lockhead2.3dm");
            //Brep LockHead = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() +
            //                    "\\LockHeadInstantTranslation2.3dm");

            Brep LockHead = FileOperation.SingleBrepFromResourceFileDirectory(path);

            RhinoApp.WriteLine(FileOperation.FindCurrentFolderResourceDirectory() + "\\bucket.3dm");

            Brep LockBase = FileOperation.SingleBrepFromResourceFileDirectory(path_base);

            // The origin of the imported lock head
            Point3d lockHeadCenOrig = new Point3d(0, 0, 0);
            Point3d lockBaseCenOrig = new Point3d(0, 0, 0);

            // create sweep function
            var sweep = new Rhino.Geometry.SweepOneRail();
            sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            sweep.ClosedSweep = false;
            sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            Brep hookBarBrep = new Brep();
            Brep firstHookBaseBrep = new Brep();
            Brep secondHookBaseBrep = new Brep();
            Brep secondHookBaseCavBrep = new Brep();
            Brep lockbaseBrep = new Brep();
            Brep secondHookBaseCavPtBrep = new Brep();

            Brep secondHookBaseTakeOutBrep = new Brep();
            Brep hookbarBrep = new Brep();

            Brep partForCutModelBoolean = new Brep();
            Brep partForCutModelBooleanMirror = new Brep();

            if (lockT < springStart)
            {

                lock_length = skeleton.GetLength() * (springEnd - springStart);
                lock_length += skeleton.GetLength() * (springStart - lockT);
                lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;

                Point3d endPointHook = skeleton.PointAtNormalizedLength(springEnd + 0.01);
                Point3d endPointSlot = skeleton.PointAtNormalizedLength(springStart - 0.01);

                Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength() - 0.001)) -
                    new Vector3d(endPointHook);
                Plane firstPtPlane = new Plane(endPointHook, normal);     // the plane at the end of the skeleton
                Plane secondPtPlane = new Plane(endPointSlot, normal);  // the plane at the start of the skeleton

                #region construct the hook bar for lock

                Vector3d f_xp = 4 / 2 * firstPtPlane.XAxis;
                Vector3d f_xn = -4 / 2 * firstPtPlane.XAxis;
                Vector3d f_yp = 1.2 / 2 * firstPtPlane.YAxis;
                Vector3d f_yn = -1.2 / 2 * firstPtPlane.YAxis;

                Point3d[] hookBarPts = new Point3d[5];
                hookBarPts[0] = endPointHook + f_xp + f_yp;
                hookBarPts[1] = endPointHook + f_xn + f_yp;
                hookBarPts[2] = endPointHook + f_xn + f_yn;
                hookBarPts[3] = endPointHook + f_xp + f_yn;
                hookBarPts[4] = endPointHook + f_xp + f_yp;
                Curve hookBarRect = new Polyline(hookBarPts).ToNurbsCurve();

                Transform rectAreaRotate = Transform.Rotation(firstPtPlane.YAxis, lockBaseVector, endPointHook);
                hookBarRect.Transform(rectAreaRotate);

                Point3d sweepFirstPt = endPointHook;
                Point3d sweepSecondPt = skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength() - 0.001) - lockBaseVector / lockBaseVector.Length * 1.2/2;
                Point3d[] railPts = new Point3d[2];
                railPts[0] = sweepFirstPt;
                railPts[1] = sweepSecondPt;
                Curve rail = new Polyline(railPts).ToNurbsCurve();

                hookBarBrep = sweep.PerformSweep(rail, hookBarRect)[0];
                hookBarBrep = hookBarBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                
                Transform hookBarBrepMove = Transform.Translation(lockBaseVector);
                hookBarBrep.Transform(hookBarBrepMove);

                #endregion

                #region test by LH
                //myDoc.Objects.AddBrep(hookBarBrep);
                //myDoc.Views.Redraw();
                #endregion

                #region construct the hook base at the end of the skeleton

                Vector3d fb_xp = 6.33 / 2 * firstPtPlane.XAxis;
                Vector3d fb_xn = -6.33 / 2 * firstPtPlane.XAxis;
                Vector3d fb_yp = 6 / 2 * firstPtPlane.YAxis;
                Vector3d fb_yn = -6 / 2 * firstPtPlane.YAxis;

                Point3d[] firstHookBasePts = new Point3d[5];
                firstHookBasePts[0] = endPointHook + fb_xp + fb_yp;
                firstHookBasePts[1] = endPointHook + fb_xn + fb_yp;
                firstHookBasePts[2] = endPointHook + fb_xn + fb_yn;
                firstHookBasePts[3] = endPointHook + fb_xp + fb_yn;
                firstHookBasePts[4] = endPointHook + fb_xp + fb_yp;
                Curve firstHookBaseRect = new Polyline(firstHookBasePts).ToNurbsCurve();

                firstHookBaseRect.Transform(rectAreaRotate);

                Point3d sweepFirstBaseFirstPt = endPointHook;
                Vector3d extVec = new Vector3d(skeleton.PointAtNormalizedLength(springEnd) - skeleton.PointAtNormalizedLength(springStart));
                Point3d sweepFirstBaseSecondPt = sweepFirstBaseFirstPt + extVec / extVec.Length * 2;
                Point3d[] firstBasePts = new Point3d[2];
                firstBasePts[0] = sweepFirstBaseFirstPt;
                firstBasePts[1] = sweepFirstBaseSecondPt;
                Curve firstBaseRail = new Polyline(firstBasePts).ToNurbsCurve();

                firstHookBaseBrep = sweep.PerformSweep(firstBaseRail, firstHookBaseRect)[0];
                firstHookBaseBrep = firstHookBaseBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                firstHookBaseBrep.Transform(hookBarBrepMove);
                firstHookBaseBrep.Flip();

                #endregion

                #region test by LH
                //myDoc.Objects.AddBrep(firstHookBaseBrep);
                //myDoc.Views.Redraw();
                #endregion

                #region transform the lock head

                Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()) - lockHeadCenOrig));
                LockHead.Transform(move1);
                Vector3d new_Xaxis = Vector3d.XAxis;
                Vector3d new_Yaxis = Vector3d.YAxis;
                new_Xaxis.Transform(move1);
                new_Yaxis.Transform(move1);

                Vector3d skeXAxis = skeleton.PointAtNormalizedLength(springEnd) - skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength());
                Transform rotate1 = Transform.Rotation(Vector3d.XAxis, skeXAxis, skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate1);
                new_Xaxis.Transform(rotate1);
                new_Yaxis.Transform(rotate1);

                Transform rotate2 = Transform.Rotation(new_Yaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate2);
                new_Xaxis.Transform(rotate2);
                new_Yaxis.Transform(rotate2);

                Transform move2 = Transform.Translation(lockBaseVector);
                LockHead.Transform(move2);
                new_Xaxis.Transform(move2);
                new_Yaxis.Transform(move2);

                #endregion

                #region test by LH
                //myDoc.Objects.AddBrep(LockHead);
                //myDoc.Views.Redraw();
                #endregion

                #region construct the hood base at the start of the skeleton
                //Vector3d sb_xp = 6.33 / 2 * secondPtPlane.XAxis;
                //Vector3d sb_xn = -6.33 / 2 * secondPtPlane.XAxis;
                //Vector3d sb_yp = 6.6 * secondPtPlane.YAxis;
                //Vector3d sb_yn = -10 / 2 * secondPtPlane.YAxis;

                //Point3d[] secondHookBasePts = new Point3d[5];
                //secondHookBasePts[0] = endPointSlot + sb_xp + sb_yp;
                //secondHookBasePts[1] = endPointSlot + sb_xn + sb_yp;
                //secondHookBasePts[2] = endPointSlot + sb_xn + sb_yn;
                //secondHookBasePts[3] = endPointSlot + sb_xp + sb_yn;
                //secondHookBasePts[4] = endPointSlot + sb_xp + sb_yp;
                //Curve secondHookBaseRect = new Polyline(secondHookBasePts).ToNurbsCurve();

                //secondHookBaseRect.Transform(rectAreaRotate);

                //Point3d sweepSecondBaseFirstPt = endPointSlot;
                //Vector3d extVec1 = new Vector3d(skeleton.PointAtNormalizedLength(springStart) - skeleton.PointAtNormalizedLength(springEnd));
                //Point3d sweepSecondBaseSecondPt = sweepSecondBaseFirstPt + extVec1 / extVec1.Length * 13.6;
                //Point3d[] secondBasePts = new Point3d[2];
                //secondBasePts[0] = sweepSecondBaseFirstPt;
                //secondBasePts[1] = sweepSecondBaseSecondPt;
                //Curve secondBaseRail = new Polyline(secondBasePts).ToNurbsCurve();

                //secondHookBaseBrep = sweep.PerformSweep(secondBaseRail, secondHookBaseRect)[0];
                //secondHookBaseBrep = secondHookBaseBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                //secondHookBaseBrep.Transform(hookBarBrepMove);

                #region test by LH
                //myDoc.Objects.AddBrep(secondHookBaseBrep);
                //myDoc.Views.Redraw();
                #endregion

                // create the cavity part1 for the lock head
                Vector3d ca_xp = 6/ 2 * secondPtPlane.XAxis;
                Vector3d ca_xn = -6/ 2 * secondPtPlane.XAxis;
                Vector3d ca_yp = 2.6 * secondPtPlane.YAxis;
                Vector3d ca_yn = -10 / 2 * secondPtPlane.YAxis;

                Point3d[] secondHookBaseCavPts = new Point3d[5];
                secondHookBaseCavPts[0] = endPointSlot + ca_xp + ca_yp;
                secondHookBaseCavPts[1] = endPointSlot + ca_xn + ca_yp;
                secondHookBaseCavPts[2] = endPointSlot + ca_xn + ca_yn;
                secondHookBaseCavPts[3] = endPointSlot + ca_xp + ca_yn;
                secondHookBaseCavPts[4] = endPointSlot + ca_xp + ca_yp;
                Curve secondHookBaseCavRect = new Polyline(secondHookBaseCavPts).ToNurbsCurve();

                secondHookBaseCavRect.Transform(rectAreaRotate);

                Vector3d extVec1 = new Vector3d(skeleton.PointAtNormalizedLength(springStart) - skeleton.PointAtNormalizedLength(springEnd));
                Point3d sweepSecondBaseCavFirstPt = endPointSlot;
                Point3d sweepSecondBaseCavSecondPt = sweepSecondBaseCavFirstPt + extVec1 / extVec1.Length * 12;
                Point3d[] secondBaseCavPts = new Point3d[2];
                secondBaseCavPts[0] = sweepSecondBaseCavFirstPt;
                secondBaseCavPts[1] = sweepSecondBaseCavSecondPt;
                Curve secondBaseCavRail = new Polyline(secondBaseCavPts).ToNurbsCurve();

                secondHookBaseCavBrep = sweep.PerformSweep(secondBaseCavRail, secondHookBaseCavRect)[0];
                secondHookBaseCavBrep = secondHookBaseCavBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                secondHookBaseCavBrep.Transform(hookBarBrepMove);

                // Add an extension for Boolean different from the main body
                Brep partForCutModelBooleanInitial = secondHookBaseCavBrep.DuplicateBrep();

                Point3d portForBooleanExtensionFirstPt = endPointSlot;
                Point3d portForBooleanExtensionSecondPt = portForBooleanExtensionFirstPt - extVec1 / extVec1.Length * 40;
                Point3d[] partForBooleanExtensionPts = new Point3d[2];
                partForBooleanExtensionPts[0] = portForBooleanExtensionFirstPt;
                partForBooleanExtensionPts[1] = portForBooleanExtensionSecondPt;
                Curve partForBooleanExtensionRail = new Polyline(partForBooleanExtensionPts).ToNurbsCurve();

                Brep partForBooleanExtensionBrep = sweep.PerformSweep(partForBooleanExtensionRail, secondHookBaseCavRect)[0];
                partForBooleanExtensionBrep = partForBooleanExtensionBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                partForBooleanExtensionBrep.Transform(hookBarBrepMove);
                partForBooleanExtensionBrep.Flip();

                var brepsForBooleanExtension = Brep.CreateBooleanUnion(new List<Brep> { partForCutModelBooleanInitial, partForBooleanExtensionBrep },
                                myDoc.ModelAbsoluteTolerance);

                if (brepsForBooleanExtension == null)
                {
                    partForBooleanExtensionBrep.Flip();
                    brepsForBooleanExtension = Brep.CreateBooleanUnion(new List<Brep> { partForCutModelBooleanInitial, partForBooleanExtensionBrep },
                                myDoc.ModelAbsoluteTolerance);
                }
                partForCutModelBoolean = brepsForBooleanExtension[0];

                partForCutModelBooleanMirror = partForCutModelBoolean.DuplicateBrep();
                Transform mirrorTrans = Transform.Mirror(p_inside, lockBaseVector);
                partForCutModelBooleanMirror.Transform(mirrorTrans);
                partForCutModelBooleanMirror.Flip();

                #region test by LH
                //myDoc.Objects.AddBrep(secondHookBaseCavBrep);
                //myDoc.Views.Redraw();
                //myDoc.Objects.AddBrep(partForCutModelBoolean);
                //myDoc.Views.Redraw();
                #endregion

                // create the cavity part2 for the lock head
                //Vector3d ca_pt_xp = 4 / 2 * secondPtPlane.XAxis;
                //Vector3d ca_pt_xn = -4 / 2 * secondPtPlane.XAxis;
                //Vector3d ca_pt_yp = 7 * secondPtPlane.YAxis;
                //Vector3d ca_pt_yn = -7.5 / 2 * secondPtPlane.YAxis;

                //Point3d[] secondHookBaseCavPtPts = new Point3d[5];
                //secondHookBaseCavPtPts[0] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xp + ca_pt_yp;
                //secondHookBaseCavPtPts[1] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xn + ca_pt_yp;
                //secondHookBaseCavPtPts[2] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xn + ca_pt_yn;
                //secondHookBaseCavPtPts[3] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xp + ca_pt_yn;
                //secondHookBaseCavPtPts[4] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xp + ca_pt_yp;
                //Curve secondHookBaseCavPtRect = new Polyline(secondHookBaseCavPtPts).ToNurbsCurve();

                //secondHookBaseCavPtRect.Transform(rectAreaRotate);

                //Point3d sweepSecondBaseCavPtFirstPt = endPointSlot + extVec1 / extVec1.Length * 2.6;
                //Point3d sweepSecondBaseCavPtSecondPt = sweepSecondBaseCavPtFirstPt + extVec1 / extVec1.Length * 9;
                //Point3d[] secondBaseCavPtPts = new Point3d[2];
                //secondBaseCavPtPts[0] = sweepSecondBaseCavPtFirstPt;
                //secondBaseCavPtPts[1] = sweepSecondBaseCavPtSecondPt;
                //Curve secondBaseCavPtRail = new Polyline(secondBaseCavPtPts).ToNurbsCurve();

                //secondHookBaseCavPtBrep = sweep.PerformSweep(secondBaseCavPtRail, secondHookBaseCavPtRect)[0];
                //secondHookBaseCavPtBrep = secondHookBaseCavPtBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                //secondHookBaseCavPtBrep.Transform(hookBarBrepMove);

                #region test by LH
                //myDoc.Objects.AddBrep(secondHookBaseCavPtBrep);
                //myDoc.Views.Redraw();
                #endregion

                //var allCavities = Brep.CreateBooleanUnion(new List<Brep> { secondHookBaseCavBrep, secondHookBaseCavPtBrep },
                //                myDoc.ModelRelativeTolerance);
                //Brep allCavity = allCavities[0];

                //var lockbaseBreps = Brep.CreateBooleanDifference(secondHookBaseBrep, allCavity, myDoc.ModelAbsoluteTolerance);
                //if (lockbaseBreps == null)
                //{
                //    secondHookBaseCavPtBrep.Flip();
                //    lockbaseBreps = Brep.CreateBooleanDifference(secondHookBaseBrep, allCavity, myDoc.ModelAbsoluteTolerance);
                //}
                //lockbaseBrep = lockbaseBreps[0];


                //var secondHookBaseTakeOutBreps = Brep.CreateBooleanDifference(secondHookBaseBrep, secondHookBaseCavBrep, myDoc.ModelAbsoluteTolerance);
                //if (secondHookBaseTakeOutBreps == null)
                //{
                //    secondHookBaseCavBrep.Flip();
                //    secondHookBaseTakeOutBreps = Brep.CreateBooleanDifference(secondHookBaseBrep, secondHookBaseCavBrep, myDoc.ModelAbsoluteTolerance);
                //}
                //secondHookBaseTakeOutBrep = secondHookBaseTakeOutBreps[0];

                //#region test by LH
                ////myDoc.Objects.AddBrep(secondHookBaseTakeOutBrep);
                ////myDoc.Views.Redraw();
                //#endregion

                //var lockbaseBreps = Brep.CreateBooleanDifference(secondHookBaseTakeOutBrep, secondHookBaseCavPtBrep, myDoc.ModelAbsoluteTolerance);
                //if (lockbaseBreps == null)
                //{
                //    secondHookBaseCavPtBrep.Flip();
                //    lockbaseBreps = Brep.CreateBooleanDifference(secondHookBaseTakeOutBrep, secondHookBaseCavPtBrep, myDoc.ModelAbsoluteTolerance);
                //}
                //lockbaseBrep = lockbaseBreps[0];

                #region transform lock base from bucket.3dm
                Vector3d se = skeleton.PointAtNormalizedLength(springEnd) - skeleton.PointAtNormalizedLength(springStart);
                Transform bs_move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart) - lockBaseCenOrig));
                LockBase.Transform(bs_move1);
                Vector3d bs_new_Xaxis = Vector3d.XAxis;
                Vector3d bs_new_Yaxis = Vector3d.YAxis;
                bs_new_Xaxis.Transform(bs_move1);
                bs_new_Yaxis.Transform(bs_move1);

                Vector3d bs_skeXAxis = skeleton.PointAtNormalizedLength(springEnd) - skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength());
                Transform bs_rotate1 = Transform.Rotation(Vector3d.XAxis, bs_skeXAxis, skeleton.PointAtNormalizedLength(springEnd));
                LockBase.Transform(bs_rotate1);
                bs_new_Xaxis.Transform(bs_rotate1);
                bs_new_Yaxis.Transform(bs_rotate1);

                Transform bs_rotate2 = Transform.Rotation(bs_new_Yaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springEnd));
                LockBase.Transform(bs_rotate2);
                bs_new_Xaxis.Transform(bs_rotate2);
                bs_new_Yaxis.Transform(bs_rotate2);

                Transform bs_move2 = Transform.Translation(lockBaseVector);
                LockBase.Transform(bs_move2);
                bs_new_Xaxis.Transform(bs_move2);
                bs_new_Yaxis.Transform(bs_move2);

                lockbaseBrep = LockBase;

                #endregion

                #endregion

                #region test by LH
                //myDoc.Objects.AddBrep(lockbaseBrep);
                //myDoc.Views.Redraw();
                #endregion

            }
            else
            {
                lock_length = skeleton.GetLength() * (springEnd - springStart);
                lock_length += skeleton.GetLength() * (lockT - springEnd);
                lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;

                Point3d endPointSlot = skeleton.PointAtNormalizedLength(springEnd + 0.01);
                Point3d endPointHook = skeleton.PointAtNormalizedLength(springStart - 0.01);

                Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + 0.001)) - new Vector3d(endPointHook);

                Plane firstPtPlane = new Plane(endPointHook, normal);     // the plane at the end of the skeleton
                Plane secondPtPlane = new Plane(endPointSlot, normal);  // the plane at the start of the skeleton

                #region construct the hook bar for lock

                Vector3d f_xp = 4 / 2 * firstPtPlane.XAxis;
                Vector3d f_xn = -4 / 2 * firstPtPlane.XAxis;
                Vector3d f_yp = 1.2 / 2 * firstPtPlane.YAxis;
                Vector3d f_yn = -1.2 / 2 * firstPtPlane.YAxis;

                Point3d[] hookBarPts = new Point3d[5];
                hookBarPts[0] = endPointHook + f_xp + f_yp;
                hookBarPts[1] = endPointHook + f_xn + f_yp;
                hookBarPts[2] = endPointHook + f_xn + f_yn;
                hookBarPts[3] = endPointHook + f_xp + f_yn;
                hookBarPts[4] = endPointHook + f_xp + f_yp;
                Curve hookBarRect = new Polyline(hookBarPts).ToNurbsCurve();

                Transform rectAreaRotate = Transform.Rotation(firstPtPlane.YAxis, lockBaseVector, endPointHook);
                hookBarRect.Transform(rectAreaRotate);

                Point3d sweepFirstPt = endPointHook;
                Point3d sweepSecondPt = skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + 0.001) - lockBaseVector / lockBaseVector.Length * 1.2/2;
                Point3d[] railPts = new Point3d[2];
                railPts[0] = sweepFirstPt;
                railPts[1] = sweepSecondPt;
                Curve rail = new Polyline(railPts).ToNurbsCurve();

                hookBarBrep = sweep.PerformSweep(rail, hookBarRect)[0];
                hookBarBrep = hookBarBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                Transform hookBarBrepMove = Transform.Translation(lockBaseVector);
                hookBarBrep.Transform(hookBarBrepMove);

                #endregion

                #region test by LH
                //myDoc.Objects.AddBrep(hookBarBrep);
                //myDoc.Views.Redraw();
                #endregion

                #region construct the hook base at the end of the skeleton

                Vector3d fb_xp = 6.33 / 2 * firstPtPlane.XAxis;
                Vector3d fb_xn = -6.33 / 2 * firstPtPlane.XAxis;
                Vector3d fb_yp = 6 / 2 * firstPtPlane.YAxis;
                Vector3d fb_yn = -6 / 2 * firstPtPlane.YAxis;

                Point3d[] firstHookBasePts = new Point3d[5];
                firstHookBasePts[0] = endPointHook + fb_xp + fb_yp;
                firstHookBasePts[1] = endPointHook + fb_xn + fb_yp;
                firstHookBasePts[2] = endPointHook + fb_xn + fb_yn;
                firstHookBasePts[3] = endPointHook + fb_xp + fb_yn;
                firstHookBasePts[4] = endPointHook + fb_xp + fb_yp;
                Curve firstHookBaseRect = new Polyline(firstHookBasePts).ToNurbsCurve();

                firstHookBaseRect.Transform(rectAreaRotate);

                Point3d sweepFirstBaseFirstPt = endPointHook;
                Vector3d extVec = new Vector3d(skeleton.PointAtNormalizedLength(springStart) - skeleton.PointAtNormalizedLength(springEnd));
                Point3d sweepFirstBaseSecondPt = sweepFirstBaseFirstPt + extVec / extVec.Length * 2;
                Point3d[] firstBasePts = new Point3d[2];
                firstBasePts[0] = sweepFirstBaseFirstPt;
                firstBasePts[1] = sweepFirstBaseSecondPt;
                Curve firstBaseRail = new Polyline(firstBasePts).ToNurbsCurve();

                firstHookBaseBrep = sweep.PerformSweep(firstBaseRail, firstHookBaseRect)[0];
                firstHookBaseBrep = firstHookBaseBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                firstHookBaseBrep.Transform(hookBarBrepMove);
                firstHookBaseBrep.Flip();

                #endregion

                #region test by LH
                //myDoc.Objects.AddBrep(firstHookBaseBrep);
                //myDoc.Views.Redraw();
                #endregion

                #region transform the lock head

                Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()) - lockHeadCenOrig));
                LockHead.Transform(move1);
                Vector3d new_Xaxis = Vector3d.XAxis;
                Vector3d new_Yaxis = Vector3d.YAxis;
                new_Xaxis.Transform(move1);
                new_Yaxis.Transform(move1);

                Vector3d skeXAxis = skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()) - skeleton.PointAtNormalizedLength(springStart);
                Transform rotate1 = Transform.Rotation(Vector3d.XAxis, (-1) * skeXAxis, skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate1);
                new_Xaxis.Transform(rotate1);
                new_Yaxis.Transform(rotate1);

                Transform rotate2 = Transform.Rotation(new_Yaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate2);
                new_Xaxis.Transform(rotate2);
                new_Yaxis.Transform(rotate2);

                Transform move2 = Transform.Translation(lockBaseVector);
                LockHead.Transform(move2);
                new_Xaxis.Transform(move2);
                new_Yaxis.Transform(move2);

                #endregion

                #region test by LH
                //myDoc.Objects.AddBrep(LockHead);
                //myDoc.Views.Redraw();
                #endregion

                #region construct the hood base at the end of the skeleton
                //Vector3d sb_xp = 6.33 / 2 * secondPtPlane.XAxis;
                //Vector3d sb_xn = -6.33 / 2 * secondPtPlane.XAxis;
                //Vector3d sb_yp = 6.6 * secondPtPlane.YAxis;
                //Vector3d sb_yn = -10 / 2 * secondPtPlane.YAxis;

                //Point3d[] secondHookBasePts = new Point3d[5];
                //secondHookBasePts[0] = endPointSlot + sb_xp + sb_yp;
                //secondHookBasePts[1] = endPointSlot + sb_xn + sb_yp;
                //secondHookBasePts[2] = endPointSlot + sb_xn + sb_yn;
                //secondHookBasePts[3] = endPointSlot + sb_xp + sb_yn;
                //secondHookBasePts[4] = endPointSlot + sb_xp + sb_yp;
                //Curve secondHookBaseRect = new Polyline(secondHookBasePts).ToNurbsCurve();

                //secondHookBaseRect.Transform(rectAreaRotate);

                //Point3d sweepSecondBaseFirstPt = endPointSlot;
                //Vector3d extVec1 = new Vector3d(skeleton.PointAtNormalizedLength(springEnd) - skeleton.PointAtNormalizedLength(springStart));
                //Point3d sweepSecondBaseSecondPt = sweepSecondBaseFirstPt + extVec1 / extVec1.Length * 13.6;
                //Point3d[] secondBasePts = new Point3d[2];
                //secondBasePts[0] = sweepSecondBaseFirstPt;
                //secondBasePts[1] = sweepSecondBaseSecondPt;
                //Curve secondBaseRail = new Polyline(secondBasePts).ToNurbsCurve();

                //secondHookBaseBrep = sweep.PerformSweep(secondBaseRail, secondHookBaseRect)[0];
                //secondHookBaseBrep = secondHookBaseBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                //secondHookBaseBrep.Transform(hookBarBrepMove);

                // create the cavity part1 for the lock head

                Vector3d extVec1 = new Vector3d(skeleton.PointAtNormalizedLength(springEnd) - skeleton.PointAtNormalizedLength(springStart));
                Vector3d ca_xp = 6 / 2 * secondPtPlane.XAxis;
                Vector3d ca_xn = -6 / 2 * secondPtPlane.XAxis;
                Vector3d ca_yp = 2.6 * secondPtPlane.YAxis;
                Vector3d ca_yn = -10 / 2 * secondPtPlane.YAxis;

                Point3d[] secondHookBaseCavPts = new Point3d[5];
                secondHookBaseCavPts[0] = endPointSlot + ca_xp + ca_yp;
                secondHookBaseCavPts[1] = endPointSlot + ca_xn + ca_yp;
                secondHookBaseCavPts[2] = endPointSlot + ca_xn + ca_yn;
                secondHookBaseCavPts[3] = endPointSlot + ca_xp + ca_yn;
                secondHookBaseCavPts[4] = endPointSlot + ca_xp + ca_yp;
                Curve secondHookBaseCavRect = new Polyline(secondHookBaseCavPts).ToNurbsCurve();

                secondHookBaseCavRect.Transform(rectAreaRotate);

                Point3d sweepSecondBaseCavFirstPt = endPointSlot;
                Point3d sweepSecondBaseCavSecondPt = sweepSecondBaseCavFirstPt + extVec1 / extVec1.Length * 12;
                Point3d[] secondBaseCavPts = new Point3d[2];
                secondBaseCavPts[0] = sweepSecondBaseCavFirstPt;
                secondBaseCavPts[1] = sweepSecondBaseCavSecondPt;
                Curve secondBaseCavRail = new Polyline(secondBaseCavPts).ToNurbsCurve();

                secondHookBaseCavBrep = sweep.PerformSweep(secondBaseCavRail, secondHookBaseCavRect)[0];
                secondHookBaseCavBrep = secondHookBaseCavBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                secondHookBaseCavBrep.Transform(hookBarBrepMove);

                // Add an extension for Boolean different from the main body
                Brep partForCutModelBooleanInitial = secondHookBaseCavBrep.DuplicateBrep();

                Point3d portForBooleanExtensionFirstPt = endPointSlot;
                Point3d portForBooleanExtensionSecondPt = portForBooleanExtensionFirstPt - extVec1 / extVec1.Length * 50;
                Point3d[] partForBooleanExtensionPts = new Point3d[2];
                partForBooleanExtensionPts[0] = portForBooleanExtensionFirstPt;
                partForBooleanExtensionPts[1] = portForBooleanExtensionSecondPt;
                Curve partForBooleanExtensionRail = new Polyline(partForBooleanExtensionPts).ToNurbsCurve();

                Brep partForBooleanExtensionBrep = sweep.PerformSweep(partForBooleanExtensionRail, secondHookBaseCavRect)[0];
                partForBooleanExtensionBrep = partForBooleanExtensionBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                partForBooleanExtensionBrep.Transform(hookBarBrepMove);
                partForBooleanExtensionBrep.Flip();

                var brepsForBooleanExtension = Brep.CreateBooleanUnion(new List<Brep> { partForCutModelBooleanInitial, partForBooleanExtensionBrep },
                                myDoc.ModelAbsoluteTolerance);

                if (brepsForBooleanExtension == null)
                {
                    partForBooleanExtensionBrep.Flip();
                    brepsForBooleanExtension = Brep.CreateBooleanUnion(new List<Brep> { partForCutModelBooleanInitial, partForBooleanExtensionBrep },
                                myDoc.ModelAbsoluteTolerance);
                }
                partForCutModelBoolean = brepsForBooleanExtension[0];

                partForCutModelBooleanMirror = partForCutModelBoolean.DuplicateBrep();
                Transform mirrorTrans = Transform.Mirror(p_inside, lockBaseVector);
                partForCutModelBooleanMirror.Transform(mirrorTrans);
                partForCutModelBooleanMirror.Flip();

                #region test by LH
                //myDoc.Objects.AddBrep(secondHookBaseCavBrep);
                //myDoc.Views.Redraw();
                #endregion

                // create the cavity part2 for the lock head
                //Vector3d ca_pt_xp = 4 / 2 * secondPtPlane.XAxis;
                //Vector3d ca_pt_xn = -4 / 2 * secondPtPlane.XAxis;
                //Vector3d ca_pt_yp = 7 * secondPtPlane.YAxis;
                //Vector3d ca_pt_yn = -7.5 / 2 * secondPtPlane.YAxis;

                //Point3d[] secondHookBaseCavPtPts = new Point3d[5];
                //secondHookBaseCavPtPts[0] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xp + ca_pt_yp;
                //secondHookBaseCavPtPts[1] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xn + ca_pt_yp;
                //secondHookBaseCavPtPts[2] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xn + ca_pt_yn;
                //secondHookBaseCavPtPts[3] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xp + ca_pt_yn;
                //secondHookBaseCavPtPts[4] = endPointSlot + extVec1 / extVec1.Length * 2.6 + ca_pt_xp + ca_pt_yp;
                //Curve secondHookBaseCavPtRect = new Polyline(secondHookBaseCavPtPts).ToNurbsCurve();

                //secondHookBaseCavPtRect.Transform(rectAreaRotate);

                //Point3d sweepSecondBaseCavPtFirstPt = endPointSlot + extVec1 / extVec1.Length * 2.6;
                //Point3d sweepSecondBaseCavPtSecondPt = sweepSecondBaseCavPtFirstPt + extVec1 / extVec1.Length * 9;
                //Point3d[] secondBaseCavPtPts = new Point3d[2];
                //secondBaseCavPtPts[0] = sweepSecondBaseCavPtFirstPt;
                //secondBaseCavPtPts[1] = sweepSecondBaseCavPtSecondPt;
                //Curve secondBaseCavPtRail = new Polyline(secondBaseCavPtPts).ToNurbsCurve();

                //secondHookBaseCavPtBrep = sweep.PerformSweep(secondBaseCavPtRail, secondHookBaseCavPtRect)[0];
                //secondHookBaseCavPtBrep = secondHookBaseCavPtBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                //secondHookBaseCavPtBrep.Transform(hookBarBrepMove);

                #region test by LH
                //myDoc.Objects.AddBrep(secondHookBaseCavPtBrep);
                //myDoc.Views.Redraw();
                #endregion

                //var secondHookBaseTakeOutBreps = Brep.CreateBooleanDifference(secondHookBaseBrep, secondHookBaseCavBrep, myDoc.ModelAbsoluteTolerance);
                //if (secondHookBaseTakeOutBreps == null)
                //{
                //    secondHookBaseCavBrep.Flip();
                //    secondHookBaseTakeOutBreps = Brep.CreateBooleanDifference(secondHookBaseBrep, secondHookBaseCavBrep, myDoc.ModelAbsoluteTolerance);
                //}
                //secondHookBaseTakeOutBrep = secondHookBaseTakeOutBreps[0];


                //var lockbaseBreps = Brep.CreateBooleanDifference(secondHookBaseTakeOutBrep, secondHookBaseCavPtBrep, myDoc.ModelAbsoluteTolerance);
                //if (lockbaseBreps == null)
                //{
                //    secondHookBaseCavPtBrep.Flip();
                //    lockbaseBreps = Brep.CreateBooleanDifference(secondHookBaseTakeOutBrep, secondHookBaseCavPtBrep, myDoc.ModelAbsoluteTolerance);
                //}
                //lockbaseBrep = lockbaseBreps[0];

                #region transform lock base from bucket.3dm
                Vector3d se = skeleton.PointAtNormalizedLength(springStart) - skeleton.PointAtNormalizedLength(springEnd);
                Transform bs_move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springEnd) - lockBaseCenOrig));
                LockBase.Transform(bs_move1);
                Vector3d bs_new_Xaxis = Vector3d.XAxis;
                Vector3d bs_new_Yaxis = Vector3d.YAxis;
                bs_new_Xaxis.Transform(bs_move1);
                bs_new_Yaxis.Transform(bs_move1);

                Vector3d bs_skeXAxis = skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()) - skeleton.PointAtNormalizedLength(springStart);
                Transform bs_rotate1 = Transform.Rotation(Vector3d.XAxis, (-1) * bs_skeXAxis, skeleton.PointAtNormalizedLength(springEnd));
                LockBase.Transform(bs_rotate1);
                bs_new_Xaxis.Transform(bs_rotate1);
                bs_new_Yaxis.Transform(bs_rotate1);

                Transform bs_rotate2 = Transform.Rotation(bs_new_Yaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springEnd));
                LockBase.Transform(bs_rotate2);
                bs_new_Xaxis.Transform(bs_rotate2);
                bs_new_Yaxis.Transform(bs_rotate2);

                Transform bs_move2 = Transform.Translation(lockBaseVector);
                LockBase.Transform(bs_move2);
                bs_new_Xaxis.Transform(bs_move2);
                bs_new_Yaxis.Transform(bs_move2);

                lockbaseBrep = LockBase;
                #endregion

                #endregion

                #region test by LH
                //myDoc.Objects.AddBrep(lockbaseBrep);
                //myDoc.Views.Redraw();
                #endregion
            }


            var LockHeadBreps = Brep.CreateBooleanUnion(new List<Brep> { firstHookBaseBrep, hookBarBrep, LockHead }, 
                                myDoc.ModelAbsoluteTolerance);

            //myDoc.Objects.AddBrep(LockHeadBreps0[0]);
            //myDoc.Objects.AddBrep(LockHead);
            //myDoc.Views.Redraw();

            if (LockHeadBreps != null)
            {
                LockHead = LockHeadBreps[0];

                Brep LockHeadMirror = LockHead.DuplicateBrep();
                Brep LockBaseMirror = lockbaseBrep.DuplicateBrep();

                Transform mirrorTrans = Transform.Mirror(p_inside, lockBaseVector);
                LockHeadMirror.Transform(mirrorTrans);
                LockBaseMirror.Transform(mirrorTrans);


                Lock lockHead1 = new Lock(LockHead, true, false);
                Lock lockBase1 = new Lock(lockbaseBrep, false, false);
                Lock lockHead2 = new Lock(LockHeadMirror, true, false);
                Lock lockBase2 = new Lock(LockBaseMirror, false, false);

                lockHead1.RegisterOtherPart(lockBase1);
                lockHead2.RegisterOtherPart(lockBase2);

                if (lockT < springStart)
                {
                    //Add fixation between lockhead and modelCut[1], lockbase and modelCut[0]
                    _ = new Fixation(lockHead1, modelCut[1]);
                    _ = new Fixation(lockHead2, modelCut[1]);

                    var baseBreps = Brep.CreateBooleanDifference(modelCut[0].GetModelinWorldCoordinate(), partForCutModelBoolean, myDoc.ModelAbsoluteTolerance);
                    if (baseBreps == null)
                    {
                        partForCutModelBoolean.Flip();
                        baseBreps = Brep.CreateBooleanDifference(modelCut[0].GetModelinWorldCoordinate(), partForCutModelBoolean, myDoc.ModelAbsoluteTolerance);
                    }
                    Brep modelCutBaseUpdated = baseBreps[0];

                    var baseBrepsMirror = Brep.CreateBooleanDifference(modelCutBaseUpdated, partForCutModelBooleanMirror, myDoc.ModelAbsoluteTolerance);
                    if (baseBrepsMirror == null)
                    {
                        partForCutModelBooleanMirror.Flip();
                        baseBrepsMirror = Brep.CreateBooleanDifference(modelCutBaseUpdated, partForCutModelBooleanMirror, myDoc.ModelAbsoluteTolerance);
                    }

                    Brep modelCutBaseFinal = baseBrepsMirror[0];

                    modelCut[0].SetModel(modelCutBaseFinal);

                    _ = new Fixation(lockBase1, modelCut[0]);
                    _ = new Fixation(lockBase2, modelCut[0]);
                }
                else
                {
                    _ = new Fixation(lockHead1, modelCut[0]);//Here lies the problem
                    _ = new Fixation(lockHead2, modelCut[0]);

                    var baseBreps = Brep.CreateBooleanDifference(modelCut[1].GetModelinWorldCoordinate(), partForCutModelBoolean, myDoc.ModelAbsoluteTolerance);
                    if (baseBreps == null)
                    {
                        partForCutModelBoolean.Flip();
                        baseBreps = Brep.CreateBooleanDifference(modelCut[1].GetModelinWorldCoordinate(), partForCutModelBoolean, myDoc.ModelAbsoluteTolerance);
                    }
                    Brep modelCutBaseUpdated = baseBreps[0];

                    var baseBrepsMirror = Brep.CreateBooleanDifference(modelCutBaseUpdated, partForCutModelBooleanMirror, myDoc.ModelAbsoluteTolerance);
                    if (baseBrepsMirror == null)
                    {
                        partForCutModelBooleanMirror.Flip();
                        baseBrepsMirror = Brep.CreateBooleanDifference(modelCutBaseUpdated, partForCutModelBooleanMirror, myDoc.ModelAbsoluteTolerance);
                    }

                    Brep modelCutBaseFinal = baseBrepsMirror[0];

                    modelCut[1].SetModel(modelCutBaseFinal);

                    _ = new Fixation(lockBase1, modelCut[1]);
                    _ = new Fixation(lockBase2, modelCut[1]);
                }
                entityList.Add(lockHead1);
                entityList.Add(lockHead2);
                entityList.Add(lockBase1);
                entityList.Add(lockBase2);

                locks.Add(lockHead1);
                locks.Add(lockHead2);
                locks.Add(lockBase1);
                locks.Add(lockBase2);
            }

            #endregion

            #region Old version
            //double extension = 3.4;

            //Point3d p_inside = skeleton.PointAtNormalizedLength(lockT);
            //Vector3d lockBaseVector = new Vector3d(lockPosition) - new Vector3d(p_inside);
            //double lockOffset = lockPosition.DistanceTo(p_inside) + extension;
            //lockBaseVector = lockBaseVector * lockOffset / lockBaseVector.Length;

            //#region test by LH
            ////Line test_L1 = new Line(p_inside, lockBaseVector);
            ////Curve test_Cr1 = test_L1.ToNurbsCurve();
            ////myDoc.Objects.AddCurve(test_Cr1);
            ////myDoc.Views.Redraw();
            //#endregion

            //springRadius = spring.SpringRadius;
            //double scale = springRadius / 7.5;
            //double lock_radius = 1.5 * scale;

            //double lock_length;

            ////directory for test
            //RhinoApp.WriteLine(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockHeadInstantTranslation2.3dm");
            //Brep LockHead = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() +
            //                    "\\LockHeadInstantTranslation2.3dm");

            ////Point3d lockHeadCenOrig = LockHead.GetBoundingBox(true).Center;
            //Point3d lockHeadCenOrig = new Point3d(0, 0, 0);
            //Brep hookBrep = new Brep();
            //Brep hookbarBrep = new Brep();

            ////Brep LockBase = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + 
            ////                  "\\LockBaseInstantExtension.3dm");
            //if (lockT < springStart)
            //{
            //    lock_length = skeleton.GetLength() * (springEnd - springStart);
            //    lock_length += skeleton.GetLength() * (springStart - lockT);
            //    lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;
            //    Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength() - 0.001)) -
            //        new Vector3d(skeleton.PointAtNormalizedLength(springEnd + 0.03));

            //    #region test by LH
            //    //Line test_L2 = new Line(skeleton.PointAtNormalizedLength(springEnd+0.03), normal);
            //    //Curve test_Cr2 = test_L2.ToNurbsCurve();
            //    //myDoc.Objects.AddCurve(test_Cr2);
            //    //myDoc.Views.Redraw();
            //    #endregion

            //    #region Create the hook bar

            //    Plane skePtPlane = new Plane(skeleton.PointAtNormalizedLength(springEnd + 0.03) + lockBaseVector, normal);
            //    Vector3d xp = 2.11 / 2 * skePtPlane.XAxis;
            //    Vector3d xn = -2.11 / 2 * skePtPlane.XAxis;
            //    Vector3d yp = 2 / 2 * skePtPlane.YAxis;
            //    Vector3d yn = -2 / 2 * skePtPlane.YAxis;

            //    Point3d[] hookPts = new Point3d[5];
            //    hookPts[0] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xp + yp;
            //    hookPts[1] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xn + yp;
            //    hookPts[2] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xn + yn;
            //    hookPts[3] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xp + yn;
            //    hookPts[4] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xp + yp;
            //    Curve hookRect = new Polyline(hookPts).ToNurbsCurve();

            //    Transform rectAreaRotate = Transform.Rotation(skePtPlane.YAxis, lockBaseVector,
            //                skeleton.PointAtNormalizedLength(springEnd + 0.03));
            //    hookRect.Transform(rectAreaRotate);

            //    // create sweep function
            //    var sweep = new Rhino.Geometry.SweepOneRail();
            //    sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            //    sweep.ClosedSweep = false;
            //    sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            //    Point3d sweepFirstPt = skeleton.PointAtNormalizedLength(springEnd + 0.03);
            //    Point3d sweepSecondPt = skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength() - 0.001);
            //    Point3d[] railPts = new Point3d[2];
            //    railPts[0] = sweepFirstPt;
            //    railPts[1] = sweepSecondPt;
            //    Curve rail = new Polyline(railPts).ToNurbsCurve();

            //    hookBrep = sweep.PerformSweep(rail, hookRect)[0];
            //    hookBrep = hookBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //    Transform hookBrepMove = Transform.Translation(lockBaseVector);
            //    hookBrep.Transform(hookBrepMove);

            //    // Create the link between the selected 3D body and the hook bar
            //    Point3d[] linkPts = new Point3d[5];
            //    Vector3d link_yp = skePtPlane.YAxis * (lockBaseVector.Length + 2 / 2);
            //    linkPts[0] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xp;
            //    linkPts[1] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xp + link_yp;
            //    linkPts[2] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xn + link_yp;
            //    linkPts[3] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xn;
            //    linkPts[4] = skeleton.PointAtNormalizedLength(springEnd + 0.03) + xp;
            //    Curve linkRect = new Polyline(linkPts).ToNurbsCurve();
            //    linkRect.Transform(rectAreaRotate);

            //    Point3d sweepLinkFirstPt = skeleton.PointAtNormalizedLength(springEnd + 0.03);
            //    Vector3d extVec = new Vector3d(skeleton.PointAtNormalizedLength(springEnd) - skeleton.PointAtNormalizedLength(springStart));
            //    Point3d sweepLinkSecondPt = sweepLinkFirstPt + extVec / extVec.Length * 2;
            //    Point3d[] linkRailPts = new Point3d[2];
            //    linkRailPts[0] = sweepLinkFirstPt;
            //    linkRailPts[1] = sweepLinkSecondPt;
            //    Curve linkRail = new Polyline(linkRailPts).ToNurbsCurve();

            //    Brep linkBrep = new Brep();
            //    linkBrep = sweep.PerformSweep(linkRail, linkRect)[0];
            //    linkBrep = linkBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            //    linkBrep.Flip();

            //    #region test by LH
            //    //myDoc.Objects.AddBrep(linkBrep);
            //    //myDoc.Views.Redraw();
            //    //myDoc.Objects.AddBrep(hookBrep);
            //    //myDoc.Views.Redraw();
            //    #endregion

            //    var hookbarBreps = Brep.CreateBooleanUnion(new List<Brep> { linkBrep, hookBrep }, myDoc.ModelAbsoluteTolerance);
            //    if (hookbarBreps.Count() == 0)
            //    {
            //        hookBrep.Flip();
            //        hookbarBreps = Brep.CreateBooleanUnion(new List<Brep> { linkBrep, hookBrep }, myDoc.ModelAbsoluteTolerance);
            //    }
            //    hookbarBrep = hookbarBreps[0];

            //    #region test by LH
            //    //myDoc.Objects.AddBrep(hookbarBrep);
            //    //myDoc.Views.Redraw();
            //    #endregion
            //    #endregion

            //    // Move the imported LockHead to the endpoint of the lock trajectory on the skeleton
            //    Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()) - lockHeadCenOrig));
            //    LockHead.Transform(move1);
            //    Vector3d new_Xaxis = Vector3d.XAxis;
            //    Vector3d new_Yaxis = Vector3d.YAxis;
            //    new_Xaxis.Transform(move1);
            //    new_Yaxis.Transform(move1);

            //    Vector3d skeXAxis = skeleton.PointAtNormalizedLength(springEnd) - skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength());
            //    Transform rotate1 = Transform.Rotation(Vector3d.XAxis, skeXAxis, skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
            //    LockHead.Transform(rotate1);
            //    new_Xaxis.Transform(rotate1);
            //    new_Yaxis.Transform(rotate1);

            //    Transform rotate2 = Transform.Rotation(new_Yaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
            //    LockHead.Transform(rotate2);
            //    new_Xaxis.Transform(rotate2);
            //    new_Yaxis.Transform(rotate2);

            //    Transform move2 = Transform.Translation(lockBaseVector);
            //    LockHead.Transform(move2);
            //    new_Xaxis.Transform(move2);
            //    new_Yaxis.Transform(move2);
            //}
            //else
            //{

            //    lock_length = skeleton.GetLength() * (springEnd - springStart);
            //    lock_length += skeleton.GetLength() * (lockT - springEnd);
            //    lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;
            //    Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + 0.001)) - new Vector3d(skeleton.PointAtNormalizedLength(springStart - 0.03));

            //    #region Create the hook bar

            //    Plane skePtPlane = new Plane(skeleton.PointAtNormalizedLength(springStart - 0.03) + lockBaseVector, normal);
            //    Vector3d xp = 2.11 / 2 * skePtPlane.XAxis;
            //    Vector3d xn = -2.11 / 2 * skePtPlane.XAxis;
            //    Vector3d yp = 2 / 2 * skePtPlane.YAxis;
            //    Vector3d yn = -2 / 2 * skePtPlane.YAxis;

            //    Point3d[] hookPts = new Point3d[5];
            //    hookPts[0] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xp + yp;
            //    hookPts[1] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xn + yp;
            //    hookPts[2] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xn + yn;
            //    hookPts[3] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xp + yn;
            //    hookPts[4] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xp + yp;
            //    Curve hookRect = new Polyline(hookPts).ToNurbsCurve();

            //    Transform rectAreaRotate = Transform.Rotation(skePtPlane.YAxis, lockBaseVector,
            //                skeleton.PointAtNormalizedLength(springStart - 0.03));
            //    hookRect.Transform(rectAreaRotate);

            //    // create sweep function
            //    var sweep = new Rhino.Geometry.SweepOneRail();
            //    sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            //    sweep.ClosedSweep = false;
            //    sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            //    Point3d sweepFirstPt = skeleton.PointAtNormalizedLength(springStart - 0.03);
            //    Point3d sweepSecondPt = skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + 0.001);
            //    Point3d[] railPts = new Point3d[2];
            //    railPts[0] = sweepFirstPt;
            //    railPts[1] = sweepSecondPt;
            //    Curve rail = new Polyline(railPts).ToNurbsCurve();

            //    hookBrep = sweep.PerformSweep(rail, hookRect)[0];
            //    hookBrep = hookBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //    Transform hookBrepMove = Transform.Translation(lockBaseVector);
            //    hookBrep.Transform(hookBrepMove);

            //    // Create the link between the selected 3D body and the hook bar
            //    // Create the link between the selected 3D body and the hook bar
            //    Point3d[] linkPts = new Point3d[5];
            //    Vector3d link_yp = skePtPlane.YAxis * (lockBaseVector.Length + 2 / 2);
            //    linkPts[0] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xp;
            //    linkPts[1] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xp + link_yp;
            //    linkPts[2] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xn + link_yp;
            //    linkPts[3] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xn;
            //    linkPts[4] = skeleton.PointAtNormalizedLength(springStart - 0.03) + xp;
            //    Curve linkRect = new Polyline(linkPts).ToNurbsCurve();
            //    linkRect.Transform(rectAreaRotate);

            //    Point3d sweepLinkFirstPt = skeleton.PointAtNormalizedLength(springStart - 0.03);
            //    Vector3d extVec = new Vector3d(skeleton.PointAtNormalizedLength(springStart) - skeleton.PointAtNormalizedLength(springEnd));
            //    Point3d sweepLinkSecondPt = sweepLinkFirstPt + extVec / extVec.Length * 2;
            //    Point3d[] linkRailPts = new Point3d[2];
            //    linkRailPts[0] = sweepLinkFirstPt;
            //    linkRailPts[1] = sweepLinkSecondPt;
            //    Curve linkRail = new Polyline(linkRailPts).ToNurbsCurve();

            //    Brep linkBrep = new Brep();
            //    linkBrep = sweep.PerformSweep(linkRail, linkRect)[0];
            //    linkBrep = linkBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            //    linkBrep.Flip();

            //    var hookbarBreps = Brep.CreateBooleanUnion(new List<Brep> { linkBrep, hookBrep }, myDoc.ModelAbsoluteTolerance);
            //    if (hookbarBreps.Count() == 0)
            //    {
            //        hookBrep.Flip();
            //        hookbarBreps = Brep.CreateBooleanUnion(new List<Brep> { linkBrep, hookBrep }, myDoc.ModelAbsoluteTolerance);
            //    }
            //    hookbarBrep = hookbarBreps[0];
            //    #endregion


            //    //Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength())) + lockBaseVector);
            //    //LockHead.Transform(move1);
            //    //Transform rotate1 = Transform.Rotation(Vector3d.YAxis,
            //    //    -GeometryMethods.AverageTangent(skeleton, springStart + lock_length / skeleton.GetLength()),
            //    //    skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
            //    //LockHead.Transform(rotate1);
            //    //Vector3d new_Xaxis = Vector3d.XAxis;
            //    //new_Xaxis.Transform(rotate1);
            //    //Transform rotate2 = Transform.Rotation(new_Xaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
            //    //LockHead.Transform(rotate2);

            //    // Move the imported LockHead to the endpoint of the lock trajectory on the skeleton
            //    Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()) - lockHeadCenOrig));
            //    LockHead.Transform(move1);
            //    Vector3d new_Xaxis = Vector3d.XAxis;
            //    Vector3d new_Yaxis = Vector3d.YAxis;
            //    new_Xaxis.Transform(move1);
            //    new_Yaxis.Transform(move1);

            //    Vector3d skeXAxis = skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()) - skeleton.PointAtNormalizedLength(springStart);
            //    Transform rotate1 = Transform.Rotation(Vector3d.XAxis, (-1) * skeXAxis, skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
            //    LockHead.Transform(rotate1);
            //    new_Xaxis.Transform(rotate1);
            //    new_Yaxis.Transform(rotate1);

            //    Transform rotate2 = Transform.Rotation(new_Yaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
            //    LockHead.Transform(rotate2);
            //    new_Xaxis.Transform(rotate2);
            //    new_Yaxis.Transform(rotate2);

            //    Transform move2 = Transform.Translation(lockBaseVector);
            //    LockHead.Transform(move2);
            //    new_Xaxis.Transform(move2);
            //    new_Yaxis.Transform(move2);

            //    #region test by LH
            //    //myDoc.Objects.AddBrep(LockHead);
            //    //myDoc.Views.Redraw();
            //    #endregion
            //}

            //#region test by LH
            //Guid lockID3 = myDoc.Objects.AddBrep(hookbarBrep);
            //myDoc.Views.Redraw();
            //Guid lockID4 = myDoc.Objects.AddBrep(LockHead);
            //myDoc.Views.Redraw();
            //#endregion

            //var LockHeadBreps = Brep.CreateBooleanUnion(new List<Brep> { LockHead, hookbarBrep }, myDoc.ModelAbsoluteTolerance);
            //if (LockHeadBreps.Count() == 0)
            //{
            //    // flip hookBrep
            //    hookbarBrep.Flip();
            //    LockHeadBreps = Brep.CreateBooleanUnion(new List<Brep> { LockHead, hookbarBrep }, myDoc.ModelAbsoluteTolerance);
            //}

            //if (LockHeadBreps.Count() != 0)
            //{
            //    LockHead = LockHeadBreps[0];
            //    Lock lockHead = new Lock(LockHead, true, false);
            //    Lock lockBase = new Lock(null, false, false);
            //    lockHead.RegisterOtherPart(lockBase);
            //    if (lockT < springStart)
            //    {
            //        //Add fixation between lockhead and modelCut[1], lockbase and modelCut[0]
            //        _ = new Fixation(lockHead, modelCut[1]);
            //        _ = new Fixation(lockBase, modelCut[0]);
            //    }
            //    else
            //    {
            //        _ = new Fixation(lockHead, modelCut[0]);//Here lies the problem
            //        _ = new Fixation(lockBase, modelCut[1]);
            //    }
            //    entityList.Add(lockHead);
            //    entityList.Add(lockBase);

            //    locks.Add(lockHead);
            //    locks.Add(lockBase);
            //}

            #endregion

            //if (type==1)
            //{
            //    Point3d p_inside = skeleton.PointAtNormalizedLength(lockT);
            //    Vector3d lockBaseVector = new Vector3d(lockPosition) - new Vector3d(p_inside);
            //    springRadius = spring.SpringRadius;
            //    double scale = springRadius / 7.5;
            //    double lock_radius = 1.5 * scale;

            //    double lock_length;

            //    //directory for test
            //    RhinoApp.WriteLine(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockHeadInstantExtension.3dm");
            //    Brep LockHead = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockHeadInstantExtension.3dm");
            //    Brep LockBase = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockBaseInstantExtension.3dm");
            //    Point3d LockBaseReleasePosition = FileOperation.SinglePointFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockBaseInstantExtension.3dm");
            //    //directory for actual use
            //    //RhinoApp.WriteLine(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockHead.3dm");
            //    //Brep LockHead = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockHeadInstantExtension.3dm");
            //    //Brep LockBase = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockBaseInstantExtension.3dm");
            //    //Point3d LockBaseReleasePosition = FileOperation.SinglePointFromResourceFileDirectory(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockBaseInstantExtension.3dm");
            //    //Recent trial of using resources manager
            //    /*RhinoApp.WriteLine("Using Resources Manager to load 3dm files");
            //    Brep LockHead = FileOperation.SingleBrepFromResourceFile(Properties.Resources.LockHeadInstantExtension);
            //    Brep LockBase = FileOperation.SingleBrepFromResourceFile(Properties.Resources.LockBaseInstantExtension);
            //    Point3d LockBaseReleasePosition = FileOperation.SinglePointFromResourceFile(Properties.Resources.LockBaseInstantExtension);*/

            //    Cylinder rod;
            //    Transform scaler = Transform.Scale(new Point3d(0, 0, 0), scale);
            //    LockHead.Transform(scaler);
            //    LockBase.Transform(scaler);
            //    LockBaseReleasePosition.Transform(scaler);
            //    if (lockT < springStart)
            //    {
            //        //Generate the rod of lock structure
            //        lock_length = skeleton.GetLength() * (springEnd - springStart);
            //        lock_length += skeleton.GetLength() * (springStart - lockT);
            //        lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;
            //        Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength() - 0.001)) - new Vector3d(skeleton.PointAtNormalizedLength(springEnd + 0.03));
            //        Circle c = new Circle(new Plane(skeleton.PointAtNormalizedLength(springEnd + 0.03), normal), lock_radius);
            //        //Curve rail= skeleton.ToNurbsCurve(new Interval(springEnd-lock_length/skeleton.GetLength(), springEnd));//Check if other type of t is needed here
            //        //Curve rail = skeleton.ToNurbsCurve(new Interval(skeleton.Domain.ParameterAt(springEnd - lock_length / skeleton.GetLength()), skeleton.Domain.ParameterAt(springEnd)));
            //        rod = new Cylinder(c, normal.Length);
            //        //Brep[] sweep_shape = Rhino.Geometry.Brep.CreateFromSweep(rail, c.ToNurbsCurve(), true, myDoc.ModelAbsoluteTolerance);
            //        //rod = sweep_shape[0];
            //        //Move and rotate two models
            //        Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength())));
            //        LockHead.Transform(move1);
            //        LockBase.Transform(move1);
            //        LockBaseReleasePosition.Transform(move1);
            //        Transform rotate1 = Transform.Rotation(Vector3d.YAxis,
            //            GeometryMethods.AverageTangent(skeleton, springEnd - lock_length / skeleton.GetLength()),
            //            skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
            //        LockHead.Transform(rotate1);
            //        LockBase.Transform(rotate1);
            //        LockBaseReleasePosition.Transform(rotate1);
            //        Vector3d new_Xaxis = Vector3d.XAxis;
            //        Vector3d new_Yaxis = Vector3d.YAxis;
            //        new_Xaxis.Transform(rotate1);
            //        new_Yaxis.Transform(rotate1);
            //        Transform rotate2 = Transform.Rotation(new_Xaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
            //        LockHead.Transform(rotate2);
            //        LockBase.Transform(rotate2);
            //        LockBaseReleasePosition.Transform(rotate2);
            //        new_Yaxis.Transform(rotate2);
            //        //Transform move2 = Transform.Translation(-skeletonVector * (springEnd - springStart) * distance);
            //        Transform move2 = Transform.Translation(
            //            new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength() - (springEnd - springStart) * distance))
            //            - new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength())));
            //        LockBase.Transform(move2);
            //        LockBaseReleasePosition.Transform(move2);
            //        Transform rotate3 = Transform.Rotation(new_Yaxis, GeometryMethods.AverageTangent
            //            (skeleton, springEnd - lock_length / skeleton.GetLength()), skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
            //        LockHead.Transform(rotate3);
            //        Transform rotate4 = Transform.Rotation(Kinergy.Utilities.GeometryMethods.AverageTangent(skeleton, springEnd - lock_length / skeleton.GetLength()),
            //            Kinergy.Utilities.GeometryMethods.AverageTangent(skeleton, springEnd - lock_length / skeleton.GetLength() - (springEnd - springStart) * distance),
            //            skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength() - (springEnd - springStart) * distance));
            //        LockBase.Transform(rotate4);
            //        LockBaseReleasePosition.Transform(rotate4);
            //        new_Yaxis.Transform(rotate4);
            //        Transform rotate5 = Transform.Rotation(new_Yaxis, GeometryMethods.AverageTangent(skeleton, lockT), skeleton.PointAtNormalizedLength(lockT));
            //        LockBase.Transform(rotate5);
            //        LockBaseReleasePosition.Transform(rotate5);
            //        //spring.Reverse();
            //    }
            //    else
            //    {
            //        lock_length = skeleton.GetLength() * (springEnd - springStart);
            //        lock_length += skeleton.GetLength() * (lockT - springEnd);
            //        lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;
            //        //Circle c = new Circle(new Plane(skeleton.PointAtNormalizedLength(springStart), skeletonVector), lock_radius);
            //        Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + 0.001)) - new Vector3d(skeleton.PointAtNormalizedLength(springStart - 0.03));
            //        Circle c = new Circle(new Plane(skeleton.PointAtNormalizedLength(springStart - 0.03), normal), lock_radius);
            //        rod = new Cylinder(c, normal.Length);
            //        //Curve rail = skeleton.ToNurbsCurve(new Interval(skeleton.Domain.ParameterAt( springStart),skeleton.Domain.ParameterAt( springStart + lock_length / skeleton.GetLength())));
            //        //Brep[] sweep_shape = Rhino.Geometry.Brep.CreateFromSweep(rail, c.ToNurbsCurve(), true, myDoc.ModelAbsoluteTolerance);
            //        //rod = sweep_shape[0];
            //        //Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart) + skeletonVector * lock_length / skeletonVector.Length));
            //        Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength())));
            //        LockHead.Transform(move1);
            //        LockBase.Transform(move1);
            //        LockBaseReleasePosition.Transform(move1);
            //        Transform rotate1 = Transform.Rotation(Vector3d.YAxis,
            //            -GeometryMethods.AverageTangent(skeleton, springStart + lock_length / skeleton.GetLength()),
            //            skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
            //        LockHead.Transform(rotate1);
            //        LockBase.Transform(rotate1);
            //        LockBaseReleasePosition.Transform(rotate1);
            //        Vector3d new_Xaxis = Vector3d.XAxis;
            //        Vector3d new_Yaxis = Vector3d.YAxis;
            //        new_Xaxis.Transform(rotate1);
            //        new_Yaxis.Transform(rotate1);
            //        Transform rotate2 = Transform.Rotation(new_Xaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
            //        LockHead.Transform(rotate2);
            //        LockBase.Transform(rotate2);
            //        LockBaseReleasePosition.Transform(rotate2);
            //        new_Yaxis.Transform(rotate2);
            //        //Transform move2 = Transform.Translation(skeletonVector * (springEnd - springStart) * distance);
            //        Transform move2 = Transform.Translation(
            //            new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + (springEnd - springStart) * distance))
            //            - new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength())));
            //        LockBase.Transform(move2);
            //        LockBaseReleasePosition.Transform(move2);
            //        Transform rotate3 = Transform.Rotation(new_Yaxis, -GeometryMethods.AverageTangent
            //            (skeleton, springStart + lock_length / skeleton.GetLength()), skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
            //        LockHead.Transform(rotate3);
            //        Transform rotate4 = Transform.Rotation(GeometryMethods.AverageTangent(skeleton, springStart + lock_length / skeleton.GetLength()),
            //            Kinergy.Utilities.GeometryMethods.AverageTangent(skeleton, springStart + lock_length / skeleton.GetLength() + (springEnd - springStart) * distance),
            //            skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + (springEnd - springStart) * distance));
            //        LockBase.Transform(rotate4);
            //        LockBaseReleasePosition.Transform(rotate4);
            //        new_Yaxis.Transform(rotate4);
            //        Transform rotate5 = Transform.Rotation(new_Yaxis, -GeometryMethods.AverageTangent(skeleton, lockT), skeleton.PointAtNormalizedLength(lockT));
            //        LockBase.Transform(rotate5);
            //        LockBaseReleasePosition.Transform(rotate5);
            //        spring.Reverse();

            //    }
            //    LockHead = Brep.CreateBooleanUnion(new List<Brep> { LockHead, rod.ToBrep(true, true) }, myDoc.ModelAbsoluteTolerance)[0];
            //    Lock lockHead = new Lock(LockHead, true, false);
            //    Lock lockBase = new Lock(LockBase, false, LockBaseReleasePosition, false);
            //    lockHead.RegisterOtherPart(lockBase);
            //    if (lockT < springStart)
            //    {
            //        //Add fixation between lockhead and modelCut[1], lockbase and modelCut[0]
            //        _ = new Fixation(lockHead, modelCut[1]);
            //        _ = new Fixation(lockBase, modelCut[0]);
            //    }
            //    else
            //    {
            //        _ = new Fixation(lockHead, modelCut[0]);//Here lies the problem
            //        _ = new Fixation(lockBase, modelCut[1]);
            //    }
            //    entityList.Add(lockHead);
            //    entityList.Add(lockBase);

            //    locks.Add(lockHead);
            //    locks.Add(lockBase);

            //}
            //else if(type==2)
            //{

            //}

            return true;
        }
        private double GetSectionRadius(double t)
        {
            Curve[] crvs;
            Plane midPlane = new Plane(skeleton.PointAtNormalizedLength(t), Kinergy.Utilities.GeometryMethods.AverageTangent(skeleton,t));
            Rhino.Geometry.Intersect.Intersection.BrepPlane(model, midPlane, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out crvs, out _);
            BoundingBox bbox = crvs[0].GetBoundingBox(true);
            double Radius = bbox.Diagonal.Length / 2/Math.Sqrt(2);
            return Radius*0.7;
        }
        public override bool LoadKineticUnit()
        {
            Movement compression;
            if (lockT < springStart)
            { compression = new Movement(spring, 3, -springLength * distance); }
            else
            { compression = new Movement(spring, 3, springLength * distance);  }
            spring.SetMovement(compression);
            compression.Activate();
            locks[0].SetLocked();
            Loaded = true;
            return true;
        }
        public override bool Trigger()
        {
            return locks[0].Activate();//Create point and wait for selection
        }
        public override bool TriggerWithoutInteraction()
        {
            return locks[0].ActivateWithoutInteraction();//Just release locks, no need to wait for selection.
        }
        public override Movement Simulate(double interval = 20, double precision = 0.01)
        {
            Movement m = null;
            m = spring.Activate(interval);
            m.Activate();
            return m;
        }
        private bool SkeletonGen()
        {
            // Limitations: if there are sharp cornners existing on the geometry, the generated medial axis is not accurate. 
            //              In other word, we should fillet the edges of the model if possible.
            // Convert all objects in Rhino to mesh and save as stl files in the current directory

            string dir = @"C:\KinergyTest\";
            //string dir = "\\Mac/Home/Desktop/kinetic_tool/Kinergy-master/Kinergy/Code/MotionSolver/InstantExtension/InstantExtension/bin";
            #region Bake and export the brep model as stl file
            Guid mybrepguid =Guid.Empty;
            string layername = "new_layer";
            // layer to bake the objects to
            InstTranslation.Utilities.create_layer(layername);
            //create a directory to store the stl files
            InstTranslation.Utilities.create_dir(dir);
            var doc = Rhino.RhinoDoc.ActiveDoc;

            //declare the objects attributes
            ObjectAttributes attr = new Rhino.DocObjects.ObjectAttributes();
            //set attributes
            var l = doc.Layers.FindName(layername);
            attr.LayerIndex = l.Index;

            Guid id = Guid.Empty;

            //bake the brep
            if (model.ObjectType == ObjectType.Brep)
            {
                id = doc.Objects.AddBrep(model, attr);
            }
            // add the breps to the guid
            if (id.ToString().Length > 0) mybrepguid = id;

            // select the breps in Rhino to successfully export them
            doc.Objects.Select(mybrepguid, true);
            //where to save
            string oldSTLFile = dir + @"/temp_stl.stl";
            if (File.Exists(oldSTLFile)) File.Delete(oldSTLFile);
            //and export them
            Rhino.RhinoApp.RunScript("-_Export\n\"" + oldSTLFile+"\"\n _Enter\n _Enter", true);

            //delete the breps after exporting them
            doc.Objects.Delete(mybrepguid, true);
            /*ObjRef objSel_ref;
            Guid sufObjId = Guid.Empty;
            var rc = RhinoGet.GetOneObject("Select surface or polysurface to mesh", false, ObjectType.Brep, out objSel_ref);
            if (rc == Rhino.Commands.Result.Success)
            {
                String str1 = "_ExportFileAs=_Binary ";
                String str2 = "_ExportUnfinishedObjects=_Yes ";
                String str3 = "_UseSimpleDialog=_No ";
                String str4 = "_UseSimpleParameters=_Yes ";

                String str5 = "_Enter _DetailedOptions ";
                String str6 = "_JaggedSeams=_No ";
                String str7 = "_PackTextures=_No ";
                String str8 = "_Refine=_Yes ";
                String str9 = "_SimplePlane=_Yes ";
                String str10 = "_Weld=_No ";
                String str11 = "_AdvancedOptions ";
                String str12 = "_Angle=15 ";
                String str13 = "_AspectRatio=0 ";
                String str14 = "_Distance=0.01 ";
                String str15 = "_Grid=16 ";
                String str16 = "_MaxEdgeLength=0 ";
                String str17 = "_MinEdgeLength=0.0001 ";
                String str18 = "_Enter _Enter";

                String str = str1 + str2 + str3 + str4 + str18;
                //String str = str1 + str18;
                //String str = str1 + str2 + str3 + str4 + str5 + str6 + str7 + str8 + str9 + str10 + str11 + str12 +
                //str13 + str14 + str15 + str16 + str17 + str18;
                //String str = str18;

                var stlScript = string.Format("-_Export "+oldSTLFile+str);// _ - Export \\\Mac / Home / Desktop / kinetic tool / Kinergy - master / Kinergy / Code / MotionSolver / InstantExtension / InstantExtension / bin / temp_stl.stl
                success=RhinoApp.RunScript(stlScript, true);
                model = objSel_ref.Brep();
            }
            else
            { return false; }*/
            #endregion

            List<Curve> cvs = new List<Curve>();
            Curve joined = null;

            // clean old files
            string oldFile1 = dir + @"/temp_off_skeleton.txt";
            string oldFile2 = dir + @"/temp_off.off";
            string oldFile3 = dir + @"/temp_off_convert.off";
            string oldFile4 = dir + @"/temp_off_skeleton.off";

            if (File.Exists(oldFile1)) File.Delete(oldFile1);
            if (File.Exists(oldFile2)) File.Delete(oldFile2);
            if (File.Exists(oldFile3)) File.Delete(oldFile3);
            if (File.Exists(oldFile4)) File.Delete(oldFile4);

            //var brep_mesh = Mesh.CreateFromBrep(model, MeshingParameters.FastRenderMesh)[0];

            #region Using meshlab server to convert the mesh into off file
            Process meshCompiler = new Process();
            ProcessStartInfo meshStartInfo = new ProcessStartInfo();
            meshStartInfo.CreateNoWindow = true;
            meshStartInfo.UseShellExecute = false;

            meshStartInfo.FileName = @"meshlabserver/meshlabserver.exe";

            // Note: unifying duplicated vertices is necessary
            meshStartInfo.Arguments = @" -i " + dir + @"/temp_stl.stl -o " + dir + @"/temp_off.off -s " + @"meshlabserver/clean.mlx";

            meshCompiler.StartInfo = meshStartInfo;
            meshCompiler.Start();
            meshCompiler.WaitForExit();
            #endregion

            #region call the medial axis generation cmd
            Process matCompiler = new Process();
            ProcessStartInfo startInfo = new ProcessStartInfo();
            startInfo.CreateNoWindow = true;
            //startInfo.CreateNoWindow = false;
            startInfo.UseShellExecute = false;
            startInfo.FileName = @"skeletonization/skeletonization.exe";

            startInfo.Arguments = dir + @"/temp_off.off --debug";

            matCompiler.StartInfo = startInfo;
            matCompiler.Start();
            matCompiler.WaitForExit();
            //Process.Start(startInfo);


            string curFile = dir + @"/temp_off_skeleton.txt";
            int ctrlPtNum = 0;
            //System.Threading.Thread.Sleep(10000);
            List<Point3d> maPoints = new List<Point3d>();
            string line;

            //Pass the file path and file name to the StreamReader constructor
            StreamReader sr = new StreamReader(curFile);

            //Read the first line of text
            line = sr.ReadLine();
            maPoints.Clear();

            do
            {
                // if there is only one number skip this line,
                // otherwise store those points
                string[] dots = line.Split('\t');
                if (dots.Length == 1 && maPoints.Count != 0)
                {

                    //foreach (Point3d p in maPoints)
                    //{
                    //    myDoc.Objects.AddPoint(p);
                    //}

                    Curve ma = Rhino.Geometry.Curve.CreateControlPointCurve(maPoints, 9);
                    cvs.Add(ma);
                    maPoints.Clear();
                }
                else if (dots.Length == 3)
                {
                    Point3d tempPt = new Point3d();
                    tempPt.X = Convert.ToDouble(dots[0]);
                    tempPt.Y = Convert.ToDouble(dots[1]);
                    tempPt.Z = Convert.ToDouble(dots[2]);
                    maPoints.Add(tempPt);
                    ctrlPtNum++;
                }

                line = sr.ReadLine();
            } while (line != null);
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            if (maPoints.Count != 0)
            {

                //foreach (Point3d p in maPoints)
                //{
                //    myDoc.Objects.AddPoint(p);
                //}

                Curve ma = Curve.CreateControlPointCurve(maPoints, 9);
                cvs.Add(ma);
                joined = Curve.JoinCurves(cvs)[0];

            }
            //close the file
            sr.Close();
            skeleton = joined;
            #endregion
            if (joined.GetLength() > 0)
            { return true; }
            return false;
        }

    }
}




