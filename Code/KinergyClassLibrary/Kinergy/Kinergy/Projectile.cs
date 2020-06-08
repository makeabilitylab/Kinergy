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

namespace Kinergy
{
    namespace Motion
    {
        public class Projectile
        {
            //The initial inputs
            private Brep arm;
            private Brep body;
            private double axisRadius=1.0;
            private double angle;//angle is between 0 to 180,indicating the angle of the arm to rotate;
            private double energy;//energy is above 0 and less than 10,indicating the total energy stored in fully compressed spring
            private Vector3d direction = Vector3d.Unset;
            private bool addLock;
            RhinoDoc myDoc;

            //The later selected parameters
            private Point3d controlPosition = Point3d.Unset;
            private Vector3d controlDirection = -Vector3d.YAxis;
            private Point3d triggerPosition = Point3d.Unset;

            //The calculated geometry and parameters
            private Point3d axisPoint =Point3d.Unset;//start to end;
            private Transform xrotate;
            private Transform xrotateBack;
            private BoundingBox bboxA;
            private BoundingBox bboxB;
            private double ControlPosMinY;
            private double ControlPosMaxY;
            private double boardThick = 1;
            private List<Brep> modelCut;
            private List<Brep> locks;
            private List<Brep> unit=new List<Brep>();

            /// <summary> Default constructor without basic input parameter </summary>
            /// <returns> Returns empty instance</returns>
            public Projectile(Brep Arm, Brep Body, double Angle, double Energy, Vector3d Direction, bool AddLock)
            {
                arm = Arm;
                body = Body;
                arm.Transform(xrotate);
                body.Transform(xrotate);
                bboxA = arm.GetBoundingBox(true);
                bboxB = body.GetBoundingBox(true);
                angle = Angle;
                energy = Energy;
                direction = Direction;
                addLock = AddLock;
                xrotate = Transform.Rotation(direction, Vector3d.XAxis, Point3d.Origin);
                xrotateBack = Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin);
                modelCut = new List<Brep>();
                myDoc = RhinoDoc.ActiveDoc;
                if (addLock)
                {
                    locks = new List<Brep>();
                }

            }
            /// <summary> Call this method to  </summary>
            /// <returns> Returns bool value showing whether all processes go well</returns>
            public bool Process()
            {
                string path3 = System.IO.Directory.GetCurrentDirectory();
                RhinoApp.WriteLine(path3);

                if (TestIntersection() == false)
                {
                    throw new Exception("The arm and the body are too close to each other.");
                }

                if (ConstructAxis() == false)
                {
                    throw new Exception("Failed to construct the rotation axis.");
                }

                if (SetControlPosition() == false)
                {
                    throw new Exception("Failed to set the position of the kinetic unit.");
                }

                if (ConstructSpring() == false)
                {
                    throw new Exception("Failed to construct the kinetic unit.");
                }


                if (addLock)
                {
                    if (SetTriggerPosition() == false)
                    {
                        throw new Exception("Failed to select trigger direction.");
                    }

                    if (ConstructLock() == false)
                    {
                        throw new Exception("Failed to build lock structure.");
                    }
                }
                return true;
            }
            public bool TestIntersection()
            {
                double shortestDistance = 5;
                Curve[] curves;
                Point3d[] points;
                bool intersect = Rhino.Geometry.Intersect.Intersection.BrepBrep(arm, body,shortestDistance,out curves,out points);
                if(intersect==false)
                {
                    double zMin = bboxA.Min.Z+axisRadius*2;
                    Plane plane = Plane.WorldXY;
                    plane.Translate(Vector3d.ZAxis*zMin);
                    Rhino.Geometry.Intersect.Intersection.BrepPlane(arm, plane, RhinoMath.SqrtEpsilon, out curves, out points);
                    Brep brep = Brep.CreatePlanarBreps(curves, RhinoMath.SqrtEpsilon)[0];
                    axisPoint = brep.GetBoundingBox(false).Center;
                    return true;
                }
                return false;
            }
            public bool ConstructAxis()
            {
                double yMax = bboxB.Max.Y;
                double yMin = bboxB.Min.Y;
                if (bboxA.Max.Y < yMax && yMin < bboxA.Min.Y)
                {
                    //Construct Axis
                    var xm = Transform.Translation((yMin - axisPoint.Y) * Vector3d.YAxis);
                    axisPoint.Transform(xm);
                    Brep axis = new Cylinder(new Circle(axisPoint, axisRadius), yMax - yMin).ToBrep(true, true);
                    unit.Add(axis);
                    //Construct end cylinders which prevent the axis from moving
                    double endRadius = 3;
                    double endHeight = 1;
                    Brep end = new Cylinder(new Circle(axisPoint, endRadius), endHeight).ToBrep(true, true);
                    unit.Add(end);
                    Brep endMirror = end.DuplicateBrep();
                    endMirror.Transform(Transform.Translation(Vector3d.YAxis * (yMax - yMin - endHeight)));
                    unit.Add(endMirror);
                    //Construct supports which connect axis to body
                    string path = System.IO.Directory.GetCurrentDirectory();
                    DirectoryInfo pathInfo = new DirectoryInfo(path);
                    string newPath = pathInfo.Parent.FullName;
                    Brep support = FileOperation.SingleBrepFromResourceFile(newPath + "\\Plug-ins\\Grasshopper\\Components\\KinergyResources\\Throwing_AxisSupport.3dm");
                    double gap = 2;     
                    var xt = Transform.Translation(new Point3d(axisPoint.X, axisPoint.Y + endHeight + gap, axisPoint.Z)-support.GetBoundingBox(true).Center);
                    support.Transform(xt);
                    BoundingBox bboxS = support.GetBoundingBox(true);
                    //ToUpdate: A method to adapt to more complex body shape
                    BoundingBox bboxSB = new BoundingBox(bboxS.Min.X, bboxS.Min.Y, bboxB.Max.Z, bboxS.Max.X, bboxS.Max.Y, bboxS.Min.Z);
                    Brep supportBottom = new Box(bboxSB).ToBrep();
                    Brep[] breps = { support, supportBottom };
                    support = Brep.CreateBooleanUnion(breps, RhinoMath.SqrtEpsilon)[0];
                    unit.Add(support);
                    Brep supportMirror = support.DuplicateBrep();
                    supportMirror.Transform(Transform.Translation(Vector3d.YAxis * (yMax - yMin - endHeight * 2 - gap * 2 - (bboxS.Max.Y - bboxS.Min.Y))));
                    unit.Add(supportMirror);
                    double unitSpace = 12;
                    ControlPosMinY = support.GetBoundingBox(false).Max.Y + unitSpace + gap;
                    ControlPosMaxY = supportMirror.GetBoundingBox(false).Min.Y - gap;
                    return true;
                }
                return false;
            }
            public bool SetControlPosition()
            {
                List<Point3d> points = new List<Point3d>();
                double gap = 5;
                while(ControlPosMinY < bboxA.Min.Y-1)
                {
                    Point3d point = new Point3d(axisPoint.X, ControlPosMinY, axisPoint.Z);
                    point.Transform(xrotateBack);
                    points.Add(point);
                    ControlPosMinY += gap;
                }
                while (bboxA.Max.Y+1 < ControlPosMaxY)
                {
                    Point3d point = new Point3d(axisPoint.X, ControlPosMaxY, axisPoint.Z);
                    point.Transform(xrotateBack);
                    points.Add(point);
                    ControlPosMaxY -= gap;
                }
                if (points.Count == 0)
                    return false;
                RhinoList<Guid> Id = myDoc.Objects.AddPoints(points);
                Point3d selectedPosition = Point3d.Unset;
                while (selectedPosition == Point3d.Unset)
                {
                    ObjRef obj_ref;
                    var rcommand = RhinoGet.GetOneObject("Set the position to add kinetic Unit", false, ObjectType.Point, out obj_ref);
                    if (rcommand == Rhino.Commands.Result.Success)
                    {
                        Guid guid = obj_ref.ObjectId;
                        foreach (Guid g in Id)
                        {
                            if (g == guid)
                            {
                                selectedPosition = obj_ref.Point().Location;
                                break;
                            }
                        }
                        if (selectedPosition == Point3d.Unset)
                        {
                            RhinoApp.WriteLine("The point is invalid.");
                        }
                    }
                }
                foreach (Guid g in Id)
                {
                    myDoc.Objects.Delete(g, true);
                }
                controlPosition = selectedPosition;
                controlPosition.Transform(xrotate);
                return true;
            }
            public bool ConstructSpring()
            {
                //Read control components from file and add to model
                string path = System.IO.Directory.GetCurrentDirectory();
                DirectoryInfo pathInfo = new DirectoryInfo(path);
                string newPath = pathInfo.Parent.FullName;
                List<Brep> controls = FileOperation.BrepsFromResourceFile(newPath + "\\Plug-ins\\Grasshopper\\Components\\KinergyResources\\Throwing_EnergyControl.3dm");
                //ToTest: Whether the plate's rank is 0
                Brep plate = controls[0];
                Point3d originPos = plate.GetBoundingBox(true).Center;
                var xt = Transform.Translation(controlPosition - originPos);
                var xr = Transform.Rotation(angle / Math.PI, originPos);
                for(int i=0;i<4;i++)
                {
                    if (addLock && i == 3)
                        continue;
                    else if (!addLock && i == 2)
                        continue;
                    controls[i].Transform(xr);
                    controls[i].Transform(xt);
                    unit.Add(controls[i]);
                }
                //If addlock: Add block box to the right place
                //ToTest: whether thr block box's rank is 4
                if(addLock)
                {
                    controls[4].Transform(xt);
                    locks.Add(controls[4]);
                }
                //Read connect spring components from file and add to model
                List<Brep> connections = FileOperation.BrepsFromResourceFile(newPath + "\\Plug-ins\\Grasshopper\\Components\\KinergyResources\\Throwing_SpringConnect.3dm");
                //Totest: whether ring's rank is 0
                Brep ring = connections[0];
                originPos= ring.GetBoundingBox(true).Center;
                //ToTest: whether the cylinder's rank is 1
                Point3d newPos = controls[1].GetBoundingBox(true).Center;
                xt = Transform.Translation(newPos - originPos);
                foreach(Brep connection in connections)
                {
                    connection.Transform(xt);
                }
                //Decide the spring paremeter using energy and construct spring
                //ToTest: whether the connection's rank is 1
                BoundingBox bboxC = connections[1].GetBoundingBox(true);
                Point3d connectPoint = new Point3d(bboxC.Max.X,bboxC.Center.Y,bboxC.Center.Z);
                double maxLength = bboxB.Max.X - connectPoint.X;
                double springR = 4 + energy;
                double pitch = 4.0;
                double minTurnCount = 3;
                double maxTurnCount = 5;
                if (maxLength / pitch < minTurnCount)
                    return false;
                maxTurnCount = (maxLength / pitch < maxTurnCount) ? maxLength / pitch : maxTurnCount;
                double turnCount = minTurnCount + energy * (maxTurnCount - minTurnCount);
                double thickR = 0.4 + energy * 0.5;
                Curve springCurve = NurbsCurve.CreateSpiral(Point3d.Origin, Vector3d.XAxis, new Point3d(0, 1, 0), pitch, turnCount, springR, springR);
                springCurve.Transform(Transform.Translation(connectPoint - springCurve.PointAtStart));
                Curve thickC = new Circle(Plane.WorldYZ, connectPoint, thickR).ToNurbsCurve();
                Brep spring = Brep.CreateFromSweep(springCurve, thickC, true, RhinoMath.SqrtEpsilon)[0];
                unit.Add(spring);
                //Add the support connection from spring end to body
                double supportThick = 1;
                BoundingBox bboxS = spring.GetBoundingBox(true);
                Point3d supportMax = new Point3d(bboxS.Max.X + supportThick, bboxS.Max.Y, bboxS.Max.Z);
                Point3d supportMin = new Point3d(bboxS.Max.X, bboxS.Min.Y, bboxB.Max.Z);
                Brep support = new Box(new BoundingBox(supportMin, supportMax)).ToBrep();
                unit.Add(support);
                //If AddLock: Add boardSupport to Model
                if (addLock)
                {
                    double boardconnectZ = locks[0].GetBoundingBox(true).Min.Z - boardThick / 2;
                    double boardconnectX = bboxS.Max.X - pitch;
                    //Todo: Add the cylinder and the support
                }
                return true;
            }
            public bool SetTriggerPosition()
            {
                //Select Trigger Position
                //Copy the block box to the right place
                return false;
                //这一步的目标是确定trigger的位置
            }
            public bool ConstructLock()
            {
                return false;
            }
            public List<Brep> GetModel()
            {
                return modelCut;
            }
            public List<Brep> GetSpring()
            {
                return unit;
            }
            public List<Brep> GetLock()
            {
                return locks;
            }
        }

    }
}
