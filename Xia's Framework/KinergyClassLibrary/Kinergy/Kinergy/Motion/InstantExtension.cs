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
using Kinergy.Constraints;
namespace Kinergy
{
    namespace Motion
    {
        public class InstantExtension:Motion
        {
            //The initial inputs
            private Brep model;
            private double energy;//energy is above 0 and less than 10,indicating the total energy stored in fully compressed spring
            private double distance;//distance is between 0.2 and 0.8,indicating the ratio of spring travel out of length;
            private Vector3d direction=Vector3d.Unset;
            private bool addLock;
            RhinoDoc myDoc;

            //The later selected parameters
            private Point3d springPosition = Point3d.Unset;
            private double springT,springStart,springEnd;
            private int lockDirection = 0;//0:unset;1:lock at left;2:lock at right;
            private Point3d lockPosition = Point3d.Unset;
            private double lockT;

            //The calculated geometry and parameters
            Vector3d skeletonVector;//start to end;
            private Transform xrotate;
            private Transform xrotateBack;
            private double springRadius=0;
            private double wireRadius = 0;
            private double springLength = 0;
            private int roundNum=0;
            private Interval skeletonAvailableRange;

            private Curve skeleton = null;
            private List<Shape> modelCut;
            private List<Lock> locks;
            private Spring spring;

            public Spring Spring { get => spring;private set => spring = value; }

            /// <summary> Default constructor without basic input parameter </summary>
            /// <returns> Returns empty instance</returns>
            public InstantExtension(Brep Model,double Energy,double Distance,Vector3d Direction,bool AddLock)
            {
                model = Model;
                energy = Energy;
                distance = Distance;
                direction = Direction;
                addLock = AddLock;
                xrotate = Transform.Rotation(direction, Vector3d.XAxis, Point3d.Origin);
                xrotateBack = Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin);
                modelCut = new List<Shape>();
                myDoc = RhinoDoc.ActiveDoc;
                if (addLock)
                { locks = new List<Lock>(); }
                
            }
            public InstantExtension(Brep Model, Vector3d Direction,double Energy, double Distance)
            {
                model = Model;
                energy = Energy;
                distance = Distance;
                direction = Direction;
                
                xrotate = Transform.Rotation(direction, Vector3d.XAxis, Point3d.Origin);
                xrotateBack = Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin);
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

                if (CalculateSkeleton()==false)
                {throw new Exception("Unable to process this model,please provide valid model and vector");}

                if(SetSpringPosition()==false)
                {throw new Exception("Failed to set spring position.");}

                if(CutModelForSpring()==false)
                {throw new Exception("Failed to cut model.");}

                if (ConstructSpring() == false)
                {throw new Exception("Failed to build spring.");}

                if(addLock)
                {
                    if(SetLockDirection()==false)
                    {throw new Exception("Failed to select lock direction.");}

                    if (SetLockPosition() == false)
                    {throw new Exception("Failed to select lock position.");}

                    if (lockPosition==Point3d.Unset)
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
                    points.Add( skeleton.PointAtNormalizedLength(skeletonAvailableRange.Min + skeletonAvailableRange.Length * i / 20));
                    
                }
                
                Point3d selectedPosition = points[UserSelection.UserSelectPointInRhino(points, myDoc)];
                springPosition = selectedPosition;
                skeleton.ClosestPoint(springPosition,out springT);
                springStart = springT - springLength / 2 / skeleton.GetLength();
                springEnd= springT + springLength / 2 / skeleton.GetLength();

                if (springPosition != Point3d.Unset)
                { return true; }
                else 
                { return false; }
            }
            public bool SetSpringPosition(Point3d pos)
            {
                springPosition = pos;
                skeleton.ClosestPoint(springPosition, out springT);
                springStart = springT - springLength / 2 / skeleton.GetLength();
                springEnd = springT + springLength / 2 / skeleton.GetLength();
                return true;
            }
            public List<Point3d> GetSpringPositionCandidates()
            {
                List<Point3d> points = new List<Point3d>();
                for (int i = 0; i <= 20; i++)
                {
                    points.Add(skeleton.PointAtNormalizedLength(skeletonAvailableRange.Min + skeletonAvailableRange.Length * i / 20));

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
                lockDirection = UserSelection.UserSelectCurveInRhino(crvs, myDoc)+1;
                
                return true;
            }
            public bool SetLockDirection(Arrow d)
            {
                if(direction*d.Direction>0.5)
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
                else if(lockDirection == 2)//the lock structure start from the right and stick into the left part
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
                if(candidates.Count==0)
                {
                    RhinoApp.WriteLine("No space for lock!");
                    return true;
                }
                
                int selectedIndex = UserSelection.UserSelectPointInRhino(candidates, myDoc);
                lockPosition = candidates[selectedIndex];
                skeleton.ClosestPoint(candidates[selectedIndex], out lockT);
                if(lockPosition==Point3d.Unset)
                { return false; }
                return true;
            }
            public List<Point3d> GetLockPositionCandidates()
            {
                Vector3d v = skeletonVector;
                double start = 0, end = 0;
                Brep B;
                if (lockDirection == 1)//the lock structure start from the left and stick into the right part
                {
                    start = springEnd + 5 / skeleton.GetLength();
                    end = 1 - 5 / skeleton.GetLength();
                    B = modelCut[1].Model;
                }
                else //the lock structure start from the right and stick into the left part
                {
                    end = springStart - 5 / skeleton.GetLength();
                    start = 5 / skeleton.GetLength();
                    B = modelCut[0].Model;
                }
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
                }
                return candidates;

            }
            public void SetLockPosition(Point3d p)
            {
                lockPosition = p;
                skeleton.ClosestPoint(p, out lockT);
            }
            public List<Arrow> GetLockDirectionCandidates()
            {
                
                Vector3d v1 = direction;
                Vector3d v2 = -v1;
                Point3d p1 = springPosition + v1 * (springLength / 2 + 5);
                Point3d p2 = springPosition - v1 * (springLength / 2 + 5);
                Arrow a1 = new Arrow(v1, p1);
                Arrow a2 = new Arrow(v2, p2);
                return new List<Arrow> { a1, a2 };
            }
            public bool CalculateSkeleton()
            {
                //here skeleton is calculated using bbox. Spring parameters are now determined by shape and ratio of bbox. energy and distance havn't been adopted.
                model.Transform(xrotate);
                BoundingBox box = model.GetBoundingBox(true);
                model.Transform(xrotateBack);
                Point3d p1 = new Point3d(box.Min.X, (box.Min.Y + box.Max.Y) / 2, (box.Min.Z + box.Max.Z) / 2), p2 = new Point3d(box.Max.X, (box.Min.Y + box.Max.Y) / 2, (box.Min.Z + box.Max.Z) / 2);
                Line l = new Line(p1, p2);
                springRadius = Math.Min(box.Max.Y - box.Min.Y, box.Max.Z - box.Min.Z)*0.9;
                springLength = springRadius / 7.5 * 25;
                if(springLength>l.Length*0.5)//the model is too short,so decrease spring radius
                {
                    springLength = l.Length * 0.5;
                    springRadius = springLength * 7.5 / 25*0.9;
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
                skeletonAvailableRange = new Interval((springLength/2+5)/l.Length,1- (springLength/2 + 5) / l.Length);
                
                skeleton = l.ToNurbsCurve();
                skeleton.Transform(xrotateBack);
                string body = string.Format("The skeleton is from {0},{1},{2} to {3},{4},{5}", skeleton.PointAtStart.X, skeleton.PointAtStart.Y,skeleton.PointAtStart.Z, skeleton.PointAtEnd.X, skeleton.PointAtEnd.Y, skeleton.PointAtEnd.Z);
                    RhinoApp.WriteLine(body);
                body = string.Format("The skeletonAvailableRange is from {0} to {1},length is {2}", skeletonAvailableRange.Min,skeletonAvailableRange.Max,skeletonAvailableRange.Length);
                RhinoApp.WriteLine(body);
                skeletonVector = new Vector3d(skeleton.PointAtEnd) - new Vector3d(skeleton.PointAtStart);
                if (skeleton.GetLength()>0)
                { 
                    return true;
                    
                }
                return false;
            }
            public bool CutModelForSpring()
            {   
                BoundingBox box = model.GetBoundingBox(true);

                Vector3d tan1 = skeleton.TangentAt(springStart);
                Vector3d tan2 = skeleton.TangentAt(springEnd);
                Plane plane2 = new Rhino.Geometry.Plane(skeleton.PointAtNormalizedLength(springEnd), -tan2);
                Plane plane1 = new Rhino.Geometry.Plane(skeleton.PointAtNormalizedLength(springStart), tan1);
                plane1.ExtendThroughBox(box, out Interval s1, out Interval t1);
                plane2.ExtendThroughBox(box, out Interval s2, out Interval t2);
                Brep[] Cut_Brep1 = model.Trim(plane1, 0.0001);
                Brep[] Cut_Brep2 = model.Trim(plane2, 0.0001);
                
                Shape mc1 = new Shape(Cut_Brep1[0].CapPlanarHoles(0.00001),false,"model");
                Shape mc2 = new Shape(Cut_Brep2[0].CapPlanarHoles(0.00001), false, "model");
                modelCut.Add(mc1);
                modelCut.Add( mc2);
                entityList.Add(mc1);
                entityList.Add(mc2);
                return true;
            }
            public bool ConstructSpring()
            {
                Point3d startPoint = skeleton.PointAtNormalizedLength(springStart);
                Point3d endPoint = skeleton.PointAtNormalizedLength(springEnd);
                spring = new Geom.Spring(startPoint,endPoint,springRadius,wireRadius,roundNum,distance);
                EntityList.Add(spring);
                if(spring.Model!=null)
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
                double scale = springRadius / 7.5;
                double box1_x = 8 * scale, box1_y = 4 * scale, box2_x = 6 * scale, box2_y = 6 * scale;//box1 starts from p1 and end at point param;box2 starts from selected position
                double box2_z = new Line(lockPosition, p_inside).Length*2;
                Vector3d xAxis = Vector3d.XAxis;
                Vector3d yAxis = Vector3d.YAxis;
                Vector3d zAxis = Vector3d.ZAxis;
                Rhino.Geometry.Box box1, box2;//boxes for cutting
                Point3d p1 = skeleton.PointAtNormalizedLength(springStart);
                Point3d p2 = skeleton.PointAtNormalizedLength(springEnd);
                if (lockT <springStart)
                {
                    //make sure base planes of boxes are generated at right coordinates to avoid further transformation
                    
                    Plane basePlane1 = new Plane(p1,lockBaseVector,new Plane(p1,lockBaseVector,skeletonVector).Normal);
                    Plane basePlane2 = new Plane(p_inside,skeletonVector, new Plane(p_inside, lockBaseVector, skeletonVector).Normal);
                    box1 = new Rhino.Geometry.Box(basePlane1, new Interval(-box1_x / 2, box1_x / 2), 
                           new Interval(-box1_y / 2, box1_y / 2), 
                           new Interval(-skeleton.GetLength() * (springStart - lockT), skeleton.GetLength() * (springStart - lockT)));

                    box2 = new Rhino.Geometry.Box(basePlane2, new Interval(-box2_x / 2, box2_x / 2), new Interval(-box2_y / 2, box2_y/2), new Interval(-box2_z / 2, box2_z / 2));

                    modelCut[0].SetModel(Brep.CreateBooleanDifference(modelCut[0].Model, box1.ToBrep(), 0.000001)[0]);
                    modelCut[0].SetModel(Brep.CreateBooleanDifference(modelCut[0].Model, box2.ToBrep(), 0.000001)[0]);
                }
                else
                {
                    Plane basePlane1 = new Plane(p2, lockBaseVector, new Plane(p2, lockBaseVector, skeletonVector).Normal);
                    Plane basePlane2 = new Plane(p_inside, skeletonVector, new Plane(p_inside, lockBaseVector, skeletonVector).Normal);
                    box1 = new Rhino.Geometry.Box(basePlane1, new Interval(-box1_x / 2, box1_x / 2),
                           new Interval(-box1_y / 2, box1_y / 2),
                           new Interval(-skeleton.GetLength() * (lockT - springEnd), skeleton.GetLength() * (lockT - springEnd)));

                    box2 = new Rhino.Geometry.Box(basePlane2, new Interval(-box2_x / 2, box2_x / 2), new Interval(-box2_y, box2_y), new Interval(-box2_z / 2, box2_z / 2));

                    modelCut[1].SetModel(Brep.CreateBooleanDifference(modelCut[1].Model, box1.ToBrep(), 0.000001)[0]);
                    modelCut[1].SetModel(Brep.CreateBooleanDifference(modelCut[1].Model, box2.ToBrep(), 0.000001)[0]);
                    //modelCut[1] = new Shape(Brep.CreateBooleanDifference(modelCut[1].Model, box1.ToBrep(), 0.000001)[0]);
                   // modelCut[1] = new Shape( Brep.CreateBooleanDifference(modelCut[1].Model, box2.ToBrep(), 0.000001)[0]);
                }

                return true;
            }
            public bool ConstructLock()
            {
                Point3d p_inside = skeleton.PointAtNormalizedLength(lockT);
                Vector3d lockBaseVector = new Vector3d(lockPosition) - new Vector3d(p_inside);
                double scale = springRadius / 7.5;
                double lock_radius = 1.5 * scale;
                
                double lock_length;
                
                //directory for test
                RhinoApp.WriteLine(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockHeadInstantExtension.3dm");
                Brep LockHead = FileOperation.SingleBrepFromResourceFile(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockHeadInstantExtension.3dm");
                Brep LockBase = FileOperation.SingleBrepFromResourceFile(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockBaseInstantExtension.3dm");
                Point3d LockBaseReleasePosition = FileOperation.SinglePointFromResourceFile(FileOperation.FindCurrentFolderResourceDirectory() + "\\LockBaseInstantExtension.3dm");
                //directory for actual use
                //RhinoApp.WriteLine(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockHead.3dm");
                //Brep LockHead = FileOperation.SingleBrepFromResourceFile(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockHeadInstantExtension.3dm");
                //Brep LockBase = FileOperation.SingleBrepFromResourceFile(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockBaseInstantExtension.3dm");
                //Point3d LockBaseTrigger = FileOperation.SinglePointFromResourceFile(FileOperation.FindComponentFolderDirectory() + "\\Resources\\LockBaseInstantExtension.3dm");
                Cylinder rod = new Cylinder();
                Transform scaler = Transform.Scale(new Point3d(0, 0, 0), scale);
                LockHead.Transform(scaler);
                LockBase.Transform(scaler);
                LockBaseReleasePosition.Transform(scaler);
                if (lockT < springStart)
                {
                    //Generate the rod of lock structure
                    lock_length = skeleton.GetLength() * (springEnd - springStart);
                    lock_length +=skeleton.GetLength() * (springStart - lockT);
                    lock_length -= skeleton.GetLength() * (springEnd - springStart) * distance;
                    Circle c = new Circle(new Plane(skeleton.PointAtNormalizedLength(springEnd), -skeletonVector), lock_radius);
                    rod = new Cylinder(c, lock_length);
                    //Move and rotate two models
                    Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springEnd) - skeletonVector * lock_length / skeleton.GetLength()));
                    LockHead.Transform(move1);
                    LockBase.Transform(move1);
                    LockBaseReleasePosition.Transform(move1);
                    Transform rotate1 = Transform.Rotation(Vector3d.YAxis, skeletonVector, skeleton.PointAtNormalizedLength(springEnd) - skeletonVector * lock_length / skeleton.GetLength());
                    LockHead.Transform(rotate1);
                    LockBase.Transform(rotate1);
                    LockBaseReleasePosition.Transform(rotate1);
                    Vector3d new_Xaxis = Vector3d.XAxis;
                    new_Xaxis.Transform(rotate1);
                    Transform rotate2 = Transform.Rotation(new_Xaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springEnd) - skeletonVector * lock_length / skeletonVector.Length);
                    LockHead.Transform(rotate2);
                    LockBase.Transform(rotate2);
                    LockBaseReleasePosition.Transform(rotate2);
                    Transform move2 = Transform.Translation(-skeletonVector * (springEnd - springStart) * distance);
                    LockBase.Transform(move2);
                    LockBaseReleasePosition.Transform(move2);
                }
                else
                {
                    lock_length = skeleton.GetLength() * (springEnd - springStart);
                    lock_length += skeleton.GetLength() * (lockT - springEnd);
                    lock_length -= skeleton.GetLength() * (springEnd-springStart) *  distance;
                    Circle c = new Circle(new Plane(skeleton.PointAtNormalizedLength(springStart), skeletonVector), lock_radius);
                    rod = new Cylinder(c, lock_length);
                    Transform move1 = Transform.Translation(new Vector3d(skeleton.PointAtNormalizedLength(springStart) + skeletonVector * lock_length / skeletonVector .Length));
                    LockHead.Transform(move1);
                    LockBase.Transform(move1);
                    LockBaseReleasePosition.Transform(move1);
                    Transform rotate1 = Transform.Rotation(Vector3d.YAxis, -skeletonVector , skeleton.PointAtNormalizedLength(springStart) + skeletonVector * lock_length / skeletonVector.Length);
                    LockHead.Transform(rotate1);
                    LockBase.Transform(rotate1);
                    LockBaseReleasePosition.Transform(rotate1);
                    Vector3d new_Xaxis = Vector3d.XAxis;
                    new_Xaxis.Transform(rotate1);
                    Transform rotate2 = Transform.Rotation(new_Xaxis, lockBaseVector, skeleton.PointAtNormalizedLength(springStart) + skeletonVector * lock_length / skeletonVector.Length);
                    LockHead.Transform(rotate2);
                    LockBase.Transform(rotate2);
                    LockBaseReleasePosition.Transform(rotate2);
                    Transform move2 = Transform.Translation(skeletonVector * (springEnd - springStart) * distance);
                    LockBase.Transform(move2);
                    LockBaseReleasePosition.Transform(move2);

                    spring.Reverse();

                }
                LockHead = Brep.CreateBooleanUnion(new List<Brep> { LockHead, rod.ToBrep(true, true) }, myDoc.ModelAbsoluteTolerance)[0];
                Lock lockHead = new Lock(LockHead,true,false);
                Lock lockBase = new Lock(LockBase, false,LockBaseReleasePosition,false);
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
            public override bool LoadMotion()
            {
                Movement compression = new Movement(spring, 3, -springLength * distance);
                compression.Activate();
                locks[0].SetLocked();
                return true;
            }
            public override bool Trigger()
            {
                return locks[0].Activate();//Create point and wait for selection
            }
            public override Movement Simulate(double interval = 20, double precision=0.01)
            {
                Movement m=null;
                m=spring.Activate(interval);
                m.Activate();
                return m;
            }
        }
    }
    
}

