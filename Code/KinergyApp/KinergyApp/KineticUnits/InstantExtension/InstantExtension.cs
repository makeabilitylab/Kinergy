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
using Kinergy.Relationships;
using Kinergy.KineticUnit;
using Kinergy;
using System.Diagnostics;

namespace Kinergy.KineticUnit
{
    public class InstantExtension : KineticUnit
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

        /// <summary> Default constructor without basic input parameter </summary>
        /// <returns> Returns empty instance</returns>

        public InstantExtension(Brep Model, bool Curved ,Vector3d Direction, double Energy, double Distance)
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

            if (CalculateStraightSkeleton() == false)
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

                if (ConstructLock() == false)
                { throw new Exception("Failed to build lock structure."); }
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
            springLength= GetSectionRadius(springT) / 7.5 * 25;
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
        public bool CalculateStraightSkeleton()
        {
            Transform xrotate = Transform.Rotation(direction, Vector3d.XAxis, Point3d.Origin);//Deprecated. Too ambiguous and indirect.
            Transform xrotateBack = Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin);
            //here skeleton is calculated using bbox. Spring parameters are now determined by shape and ratio of bbox. energy and distance havn't been adopted.
            model.Transform(xrotate);
            BoundingBox box = model.GetBoundingBox(true);
            model.Transform(xrotateBack);
            Point3d p1 = new Point3d(box.Min.X, (box.Min.Y + box.Max.Y) / 2, (box.Min.Z + box.Max.Z) / 2), p2 = new Point3d(box.Max.X, (box.Min.Y + box.Max.Y) / 2, (box.Min.Z + box.Max.Z) / 2);
            Line l = new Line(p1, p2);
            springRadius = Math.Min(box.Max.Y - box.Min.Y, box.Max.Z - box.Min.Z) * 0.9;
            springLength = springRadius / 7.5 * 25;
            if (springLength > l.Length * 0.5)//the model is too short,so decrease spring radius
            {
                springLength = l.Length * 0.5;
                springRadius = springLength * 7.5 / 25 * 0.9;
            }
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
            wireRadius = springRadius / 7.5 * 1;
            skeletonAvailableRange = new Interval((springLength / 2 + 5) / l.Length, 1 - (springLength / 2 + 5) / l.Length);

            skeleton = l.ToNurbsCurve();
            skeleton.Transform(xrotateBack);
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
            Brep[] Cut_Brep1 = model.Trim(plane1, 0.0001);
            Brep[] Cut_Brep2 = model.Trim(plane2, 0.0001);

            Shape mc1 = new Shape(Cut_Brep1[0].CapPlanarHoles(0.00001), false, "model");
            Shape mc2 = new Shape(Cut_Brep2[0].CapPlanarHoles(0.00001), false, "model");
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
        public bool CutModelForLock()
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
                Vector3d targetX = lockBaseVector - lockBaseVector * basePlane1.Normal*basePlane1.Normal/basePlane1.Normal.Length;
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
                modelCut[0].SetModel(Brep.CreateBooleanDifference(modelCut[0].Model, box2.ToBrep(),myDoc.ModelAbsoluteTolerance)[0]);
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
                box1 = new Rhino.Geometry.Box(basePlane1, new Interval(-box1_x / 2, box1_x / 2),new Interval(-box1_y / 2, box1_y / 2),new Interval(-v1.Length, v1.Length));
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

            return true;
        }
        public bool ConstructLock()
        {
            Point3d p_inside = skeleton.PointAtNormalizedLength(lockT);
            Vector3d lockBaseVector = new Vector3d(lockPosition) - new Vector3d(p_inside);
            springRadius = spring.SpringRadius;
            double scale = springRadius / 7.5;
            double lock_radius = 1.5 * scale;

            double lock_length;

            //directory for test
            RhinoApp.WriteLine(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockHeadInstantExtension.3dm");
            Brep LockHead = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockHeadInstantExtension.3dm");
            Brep LockBase = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockBaseInstantExtension.3dm");
            Point3d LockBaseReleasePosition = FileOperation.SinglePointFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockBaseInstantExtension.3dm");
            //directory for actual use
            //RhinoApp.WriteLine(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockHead.3dm");
            //Brep LockHead = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockHeadInstantExtension.3dm");
            //Brep LockBase = FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockBaseInstantExtension.3dm");
            //Point3d LockBaseReleasePosition = FileOperation.SinglePointFromResourceFileDirectory(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockBaseInstantExtension.3dm");
            //Recent trial of using resources manager
            /*RhinoApp.WriteLine("Using Resources Manager to load 3dm files");
            Brep LockHead = FileOperation.SingleBrepFromResourceFile(Properties.Resources.LockHeadInstantExtension);
            Brep LockBase = FileOperation.SingleBrepFromResourceFile(Properties.Resources.LockBaseInstantExtension);
            Point3d LockBaseReleasePosition = FileOperation.SinglePointFromResourceFile(Properties.Resources.LockBaseInstantExtension);*/

            Cylinder rod ;
            Transform scaler = Transform.Scale(new Point3d(0, 0, 0), scale);
            LockHead.Transform(scaler);
            LockBase.Transform(scaler);
            LockBaseReleasePosition.Transform(scaler);
            if (lockT < springStart)
            {
                //Generate the rod of lock structure
                lock_length = skeleton.GetLength() * (springEnd - springStart);
                lock_length += skeleton.GetLength() * (springStart - lockT);
                lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;
                Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()-0.001)) - new Vector3d(skeleton.PointAtNormalizedLength(springEnd+0.03));
                Circle c = new Circle(new Plane(skeleton.PointAtNormalizedLength(springEnd+0.03),normal), lock_radius);
                //Curve rail= skeleton.ToNurbsCurve(new Interval(springEnd-lock_length/skeleton.GetLength(), springEnd));//Check if other type of t is needed here
                //Curve rail = skeleton.ToNurbsCurve(new Interval(skeleton.Domain.ParameterAt(springEnd - lock_length / skeleton.GetLength()), skeleton.Domain.ParameterAt(springEnd)));
                rod = new Cylinder(c, normal.Length);
                //Brep[] sweep_shape = Rhino.Geometry.Brep.CreateFromSweep(rail, c.ToNurbsCurve(), true, myDoc.ModelAbsoluteTolerance);
                //rod = sweep_shape[0];
                //Move and rotate two models
                Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength())));
                LockHead.Transform(move1);
                LockBase.Transform(move1);
                LockBaseReleasePosition.Transform(move1);
                Transform rotate1 = Transform.Rotation(Vector3d.YAxis, 
                    GeometryMethods.AverageTangent(skeleton, springEnd - lock_length / skeleton.GetLength()), 
                    skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate1);
                LockBase.Transform(rotate1);
                LockBaseReleasePosition.Transform(rotate1);
                Vector3d new_Xaxis = Vector3d.XAxis;
                Vector3d new_Yaxis = Vector3d.YAxis;
                new_Xaxis.Transform(rotate1);
                new_Yaxis.Transform(rotate1);
                Transform rotate2 = Transform.Rotation(new_Xaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate2);
                LockBase.Transform(rotate2);
                LockBaseReleasePosition.Transform(rotate2);
                new_Yaxis.Transform(rotate2);
                //Transform move2 = Transform.Translation(-skeletonVector * (springEnd - springStart) * distance);
                Transform move2 = Transform.Translation(
                    new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()- (springEnd - springStart) * distance))
                    -new Vector3d(skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength())));
                LockBase.Transform(move2);
                LockBaseReleasePosition.Transform(move2);
                Transform rotate3 = Transform.Rotation(new_Yaxis, GeometryMethods.AverageTangent
                    (skeleton, springEnd - lock_length / skeleton.GetLength()), skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate3);
                Transform rotate4 = Transform.Rotation(Kinergy.Utilities.GeometryMethods.AverageTangent(skeleton, springEnd - lock_length / skeleton.GetLength()),
                    Kinergy.Utilities.GeometryMethods.AverageTangent(skeleton, springEnd - lock_length / skeleton.GetLength()- (springEnd - springStart) * distance), 
                    skeleton.PointAtNormalizedLength(springEnd - lock_length / skeleton.GetLength()- (springEnd - springStart) * distance));
                LockBase.Transform(rotate4);
                LockBaseReleasePosition.Transform(rotate4);
                new_Yaxis.Transform(rotate4);
                Transform rotate5 = Transform.Rotation(new_Yaxis, GeometryMethods.AverageTangent(skeleton,lockT), skeleton.PointAtNormalizedLength(lockT));
                LockBase.Transform(rotate5);
                LockBaseReleasePosition.Transform(rotate5);
                //spring.Reverse();
            }
            else
            {
                lock_length = skeleton.GetLength() * (springEnd - springStart);
                lock_length += skeleton.GetLength() * (lockT - springEnd);
                lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;
                //Circle c = new Circle(new Plane(skeleton.PointAtNormalizedLength(springStart), skeletonVector), lock_radius);
                Vector3d normal = new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()+0.001)) - new Vector3d(skeleton.PointAtNormalizedLength(springStart-0.03));
                Circle c = new Circle(new Plane(skeleton.PointAtNormalizedLength(springStart-0.03), normal), lock_radius);
                rod = new Cylinder(c, normal.Length);
                //Curve rail = skeleton.ToNurbsCurve(new Interval(skeleton.Domain.ParameterAt( springStart),skeleton.Domain.ParameterAt( springStart + lock_length / skeleton.GetLength())));
                //Brep[] sweep_shape = Rhino.Geometry.Brep.CreateFromSweep(rail, c.ToNurbsCurve(), true, myDoc.ModelAbsoluteTolerance);
                //rod = sweep_shape[0];
                //Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart) + skeletonVector * lock_length / skeletonVector.Length));
                Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength())));
                LockHead.Transform(move1);
                LockBase.Transform(move1);
                LockBaseReleasePosition.Transform(move1);
                Transform rotate1 = Transform.Rotation(Vector3d.YAxis,
                    -GeometryMethods.AverageTangent(skeleton, springStart + lock_length / skeleton.GetLength()), 
                    skeleton.PointAtNormalizedLength(springStart +lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate1);
                LockBase.Transform(rotate1);
                LockBaseReleasePosition.Transform(rotate1);
                Vector3d new_Xaxis = Vector3d.XAxis;
                Vector3d new_Yaxis = Vector3d.YAxis;
                new_Xaxis.Transform(rotate1);
                new_Yaxis.Transform(rotate1);
                Transform rotate2 = Transform.Rotation(new_Xaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springStart+ lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate2);
                LockBase.Transform(rotate2);
                LockBaseReleasePosition.Transform(rotate2);
                new_Yaxis.Transform(rotate2);
                //Transform move2 = Transform.Translation(skeletonVector * (springEnd - springStart) * distance);
                Transform move2= Transform.Translation(
                    new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + (springEnd - springStart) * distance))
                    - new Vector3d(skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength())));
                LockBase.Transform(move2);
                LockBaseReleasePosition.Transform(move2);
                Transform rotate3 = Transform.Rotation(new_Yaxis, -GeometryMethods.AverageTangent
                    (skeleton, springStart + lock_length / skeleton.GetLength()), skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength()));
                LockHead.Transform(rotate3);
                Transform rotate4 = Transform.Rotation(GeometryMethods.AverageTangent(skeleton, springStart + lock_length / skeleton.GetLength()),
                    Kinergy.Utilities.GeometryMethods.AverageTangent(skeleton, springStart + lock_length / skeleton.GetLength() + (springEnd - springStart) * distance),
                    skeleton.PointAtNormalizedLength(springStart + lock_length / skeleton.GetLength() + (springEnd - springStart) * distance));
                LockBase.Transform(rotate4);
                LockBaseReleasePosition.Transform(rotate4);
                new_Yaxis.Transform(rotate4);
                Transform rotate5 = Transform.Rotation(new_Yaxis, -GeometryMethods.AverageTangent(skeleton, lockT), skeleton.PointAtNormalizedLength(lockT));
                LockBase.Transform(rotate5);
                LockBaseReleasePosition.Transform(rotate5);
                spring.Reverse();

            }
            LockHead = Brep.CreateBooleanUnion(new List<Brep> { LockHead, rod.ToBrep(true,true) }, myDoc.ModelAbsoluteTolerance)[0];
            Lock lockHead = new Lock(LockHead, true, false);
            Lock lockBase = new Lock(LockBase, false, LockBaseReleasePosition, false);
            lockHead.RegisterOtherPart(lockBase);
            if (lockT < springStart)
            {
                //Add fixation between lockhead and modelCut[1], lockbase and modelCut[0]
                _ = new Fixation(lockHead, modelCut[1]);
                _ = new Fixation(lockBase, modelCut[0]);
            }
            else
            {
                _ = new Fixation(lockHead, modelCut[0]);//Here lies the problem
                _ = new Fixation(lockBase, modelCut[1]);
            }
            entityList.Add(lockHead);
            entityList.Add(lockBase);

            locks.Add(lockHead);
            locks.Add(lockBase);

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
            InstExtension.Utilities.create_layer(layername);
            //create a directory to store the stl files
            InstExtension.Utilities.create_dir(dir);
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




