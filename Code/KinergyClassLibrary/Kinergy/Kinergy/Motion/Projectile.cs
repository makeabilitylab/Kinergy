using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.IO;
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

namespace Kinergy
{
    namespace Motion
    {
        public class Projectile : Motion
        {
            //The initial inputs
            private Brep arm;
            private Brep body;
            private double angle;//angle is between 0 to 180,indicating the angle of the arm to rotate;
            private double energy;//energy is above 0 and less than 10,indicating the total energy stored in fully compressed spring
            private Vector3d direction = Vector3d.Unset;
            private bool addLock;
            RhinoDoc myDoc;
            private double axisRadius = 1.0;

            //The later selected parameters
            //private Point3d controlPosition = Point3d.Unset;
            //private Vector3d controlDirection = -Vector3d.YAxis;
            private Point3d triggerPos = Point3d.Unset;

            //The calculated geometry and parameters
            private Point3d axisPoint = Point3d.Unset;//start to end;
            private Transform xrotate;
            private Transform xrotateBack;
            private BoundingBox bboxA;
            private BoundingBox bboxB;
            private Point3d controlPos = Point3d.Unset;
            private Brep controlUnit = null;
            private Point3d springStartPoint = Point3d.Unset;
            private Point3d springEndPoint = Point3d.Unset;
            Point3d supportAxisPos = Point3d.Unset;
            private double ControlPosMinY;
            private double ControlPosMaxY;
            private double supportPosMinY;
            private double supportPosMaxY;

            //The final outputs
            private List<Brep> locks;
            private List<Brep> unit;

            /// <summary> Default constructor without basic input parameter </summary>
            /// <returns> Returns empty instance</returns>
            public Projectile(Brep Arm, Brep Body, Vector3d Direction, double Energy, double Angle, bool AddLock)
            {
                arm = Arm;
                body = Body;
                angle = Angle;
                energy = Energy;
                direction = Direction;
                addLock = AddLock;
                xrotate = Transform.Rotation(direction, Vector3d.XAxis, Point3d.Origin);
                xrotateBack = Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin);
                arm.Transform(xrotate);
                body.Transform(xrotate);
                bboxA = arm.GetBoundingBox(true);
                bboxB = body.GetBoundingBox(true);
                myDoc = RhinoDoc.ActiveDoc;
                unit = new List<Brep>();
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
                if (TestRotateIntersection() == false)
                {
                    throw new Exception("The rotation angle is invalid.");
                }
                if (ConstructControl() == false)
                {
                    throw new Exception("Failed to construct the control unit.");
                }
                if (ConstructSpring() == false)
                {
                    throw new Exception("Failed to construct the spring.");
                }
                if (addLock)
                {
                    if (ConstructBlock() == false)
                    {
                        throw new Exception("Failed to construct lock blocks.");
                    }

                    if (ConstructLockAxis() == false)
                    {
                        throw new Exception("Failed to construct lock axis.");
                    }

                    if (SetTriggerPosition() == false)
                    {
                        throw new Exception("Failed to select trigger direction.");
                    }

                    if (ConstructBlockSupport() == false)
                    {
                        throw new Exception("Failed to construct support of lock.");
                    }

                }
                if (RotateBack() == false)
                {
                    throw new Exception("Failed to rotate back kinergy unit.");
                }
                return true;
            }

            public bool TestIntersection()
            {
                double shortestDistance = 5;
                Curve[] curves;
                Point3d[] points;
                bool intersect = Rhino.Geometry.Intersect.Intersection.BrepBrep(arm, body, shortestDistance, out curves, out points);
                if (curves.Length == 0)
                {
                    double zMin = bboxA.Min.Z + axisRadius * 2;
                    Plane plane = Plane.WorldXY;
                    plane.Translate(Vector3d.ZAxis * zMin);
                    Rhino.Geometry.Intersect.Intersection.BrepPlane(arm, plane, RhinoMath.SqrtEpsilon, out curves, out points);
                    Brep brep = Brep.CreatePlanarBreps(curves, RhinoMath.SqrtEpsilon)[0];
                    axisPoint = brep.GetBoundingBox(true).Center;
                    return true;
                }
                return false;
            }

            private Brep generateIntersection(Plane plane, Brep brep)
            {
                Curve[] curves;
                Point3d[] points;
                Rhino.Geometry.Intersect.Intersection.BrepPlane(brep, plane, RhinoMath.SqrtEpsilon, out curves, out points);
                if (curves.Length == 0)
                    return null;
                Brep intersect = Brep.CreatePlanarBreps(curves, RhinoMath.SqrtEpsilon)[0];
                return intersect;
            }

            public bool ConstructAxis()
            {
                Plane intersectPlane = new Plane(axisPoint, Vector3d.XAxis);
                BoundingBox intersect = generateIntersection(intersectPlane, body).GetBoundingBox(true);
                double yMax = intersect.Max.Y - 1;
                double yMin = intersect.Min.Y + 1;
                if (bboxA.Max.Y > yMax || yMin > bboxA.Min.Y)
                    return false;
                //Construct Axis
                var xm = Transform.Translation((yMin - axisPoint.Y) * Vector3d.YAxis);
                Point3d axisMinEnd = new Point3d(axisPoint.X, yMin, axisPoint.Z);
                Brep axis = new Cylinder(new Circle(Plane.WorldZX, axisMinEnd, axisRadius), yMax - yMin).ToBrep(true, true);
                unit.Add(axis);
                //Construct end cylinders which prevent the axis from moving
                double endRadius = 3;
                double endHeight = 1;
                Brep end = new Cylinder(new Circle(Plane.WorldZX, axisMinEnd, endRadius), endHeight).ToBrep(true, true);
                unit.Add(end);
                Brep endMirror = end.DuplicateBrep();
                var xmirror = Transform.Mirror(new Point3d(axisPoint.X, (yMin + yMax) / 2, axisPoint.Z), Vector3d.YAxis);
                endMirror.Transform(xmirror);
                unit.Add(endMirror);
                //Construct supports which connect axis to body
                double gap = 2;
                double supportWidth = 8;
                double supportThick = 1;
                BoundingBox bboxS = new BoundingBox(axisMinEnd.X - supportWidth / 2, axisMinEnd.Y + gap, (bboxB.Min.Z + bboxB.Max.Z) / 2, axisMinEnd.X + supportWidth / 2, axisMinEnd.Y + gap + supportThick, axisMinEnd.Z + axisRadius + 2 * gap);
                Brep support = new Box(bboxS).ToBrep();
                double gapPrint = 0.8;
                Brep trim = new Cylinder(new Circle(Plane.WorldZX, axisMinEnd, axisRadius + gapPrint), yMax - yMin).ToBrep(true, true);
                support = Brep.CreateBooleanDifference(support, trim, RhinoMath.SqrtEpsilon)[0];
                support = Brep.CreateBooleanDifference(support, body, RhinoMath.SqrtEpsilon)[0];
                unit.Add(support);
                Brep supportMirror = support.DuplicateBrep();
                supportMirror.Transform(xmirror);
                unit.Add(supportMirror);
                ControlPosMinY = support.GetBoundingBox(true).Max.Y + gap;
                supportPosMinY = ControlPosMinY;
                ControlPosMaxY = supportMirror.GetBoundingBox(true).Min.Y - gap;
                supportPosMaxY = ControlPosMaxY;
                return true;
            }

            public bool TestRotateIntersection()
            {
                //ToTest
                var xr = Transform.Rotation(-angle / 180 * Math.PI, Vector3d.YAxis, axisPoint);
                Brep armCopy = arm.DuplicateBrep();
                armCopy.Transform(xr);
                Curve[] curves;
                Point3d[] points;
                Rhino.Geometry.Intersect.Intersection.BrepBrep(armCopy, body, RhinoMath.SqrtEpsilon, out curves, out points);
                if (curves.Length == 0) return true;
                return false;
            }

            public bool ConstructControl()
            {
                double unitSpace = 12;
                double gap = 2;
                if (ControlPosMinY + unitSpace < bboxA.Min.Y - gap)
                {
                    controlPos = new Point3d(axisPoint.X, ControlPosMinY, axisPoint.Z);
                }
                else if (bboxA.Max.Y + gap < ControlPosMaxY - unitSpace)
                {
                    controlPos = new Point3d(axisPoint.X, ControlPosMaxY - unitSpace, axisPoint.Z);
                }
                if (controlPos == Point3d.Unset)
                    return false;
                //string pathInfo = FileOperation.FindComponentFolderDirectory() + "\\Resources\\Throwing_EnergyControl.3dm";
                string pathInfo = "C:\\Program Files\\Rhino 6\\Plug-ins\\Grasshopper\\Components\\Resources\\Throwing_EnergyControl.3dm";
                RhinoApp.WriteLine(pathInfo);
                List<Brep> controls = FileOperation.BrepsFromResourceFile(pathInfo);
                //ToTest: Whether the plate's rank is 0
                Brep plate = controls[1];
                Point3d originPos = plate.GetBoundingBox(true).Center;
                var xt = Transform.Translation(controlPos - originPos);
                var xr = Transform.Rotation(angle / 180 * Math.PI, Vector3d.YAxis, originPos);
                if (addLock)
                {
                    controlUnit = controls[3].DuplicateBrep();
                    controlUnit.Transform(xt);
                }
                for (int i = 0; i < 4; i++)
                {
                    if (addLock && i == 0)
                        continue;
                    else if (!addLock && i == 3)
                        continue;
                    controls[i].Transform(xr);
                    controls[i].Transform(xt);
                    unit.Add(controls[i]);
                }
                pathInfo = "C:\\Program Files\\Rhino 6\\Plug-ins\\Grasshopper\\Components\\Resources\\Throwing_SpringConnect.3dm";
                RhinoApp.WriteLine(pathInfo);
                //Read connect spring components from file and add to model
                List<Brep> connections = FileOperation.BrepsFromResourceFile(pathInfo);
                //Totest: whether ring's rank is 0
                Brep ring = connections[0];
                originPos = ring.GetBoundingBox(true).Center;
                //ToTest: whether the cylinder's rank is 1
                Point3d newPos = controls[2].GetBoundingBox(true).Center;
                xt = Transform.Translation(newPos - originPos);
                foreach (Brep connection in connections)
                {
                    connection.Transform(xt);
                    unit.Add(connection);
                }
                BoundingBox bboxC = connections[1].GetBoundingBox(true);
                springStartPoint = new Point3d(bboxC.Min.X + 1, bboxC.Center.Y, bboxC.Center.Z);
                return true;
            }

            public bool ConstructSpring()
            {
                Plane intersectPlane = new Plane(springStartPoint, Vector3d.YAxis);
                BoundingBox intersect = generateIntersection(intersectPlane, body).GetBoundingBox(true);
                double xMin = intersect.Min.X - 1;
                //Decide the spring paremeter using energy and construct spring
                //ToTest: whether the connection's rank is 1
                double maxLength = springStartPoint.X - xMin;
                double springR = 2 + 2 * energy;
                double wireR = 0.3 + 0.2 * energy;
                double pitch = 3.0;
                int TurnCount = 3 + (int)angle / 60;
                double length = pitch * TurnCount;
                if (maxLength < length)
                    return false;
                springStartPoint.Z -= springR;
                Point3d springEndPoint = new Point3d(springStartPoint.X - length, springStartPoint.Y, springStartPoint.Z);
                double gap = 5;
                supportAxisPos.X = springEndPoint.X + gap;
                Spring spring = new Spring(springStartPoint, springEndPoint, springR, wireR, TurnCount);
                unit.Add(spring.Model);
                /*
                maxTurnCount = (maxLength / pitch < maxTurnCount) ? maxLength / pitch : maxTurnCount;
                double turnCount = minTurnCount + energy * (maxTurnCount - minTurnCount);
                double thickR = 0.4 + energy * 0.5;
                Curve springCurve = NurbsCurve.CreateSpiral(Point3d.Origin, Vector3d.XAxis, new Point3d(0, 1, 0), pitch, turnCount, springR, springR);
                springCurve.Transform(Transform.Translation(connectPoint - springCurve.PointAtStart));
                Curve thickC = new Circle(Plane.WorldYZ, connectPoint, thickR).ToNurbsCurve();
                Brep spring = Brep.CreateFromSweep(springCurve, thickC, true, RhinoMath.SqrtEpsilon)[0];
                unit.Add(spring);
                */
                //Add the support connection from spring end to body
                double supportThick = 1;
                gap = 5;
                BoundingBox bboxS = new BoundingBox(springEndPoint.X - supportThick / 2, springEndPoint.Y - gap, intersect.Min.Z, springEndPoint.X, springEndPoint.Y + gap, springEndPoint.Z + gap);
                Brep support = new Box(bboxS).ToBrep();
                support = Brep.CreateBooleanDifference(support, body, RhinoMath.SqrtEpsilon)[0];
                unit.Add(support);
                return true;
            }

            public bool ConstructBlock()
            {
                BoundingBox bboxB = body.GetBoundingBox(false);
                double angleRotate = angle / 4;
                double lockH = 0.6;
                double blockThick = 1;
                double blockHeight = 5;
                BoundingBox bboxC = controlUnit.GetBoundingBox(true);
                Point3d p1 = new Point3d(bboxC.Min.X, bboxC.Center.Y, bboxC.Min.Z);
                double minZ = p1.Z + lockH - blockHeight;
                double supportGap = 3;
                if (minZ < bboxB.Max.Z + supportGap)
                    return false;
                BoundingBox bboxBlock = new BoundingBox(p1.X - blockThick, bboxC.Min.Y, minZ, p1.X, bboxC.Max.Y, p1.Z + lockH);
                Brep b1 = new Box(bboxBlock).ToBrep();
                locks.Add(b1);
                var xr = Transform.Rotation(angleRotate / 180 * Math.PI, Vector3d.YAxis, controlPos);
                double maxAngle = 70;
                double minDist = 1;
                double lastX = bboxBlock.Min.X;
                Point3d p2 = new Point3d(p1);
                for (int i = 1; i < 4; i++)
                {
                    if (angleRotate * i > maxAngle)
                        break;
                    p2.Transform(xr);
                    if (lastX - p2.X < minDist)
                        continue;
                    bboxBlock = new BoundingBox(p2.X - blockThick, bboxC.Min.Y, minZ, p2.X, bboxC.Max.Y, p2.Z + lockH);
                    Brep b2 = new Box(bboxBlock).ToBrep();
                    locks.Add(b2);
                    lastX = bboxBlock.Min.X;
                }
                supportAxisPos.Y = p1.Y;
                supportAxisPos.Z = minZ;
                return true;
            }

            public bool ConstructLockAxis()
            {
                Plane intersectPlane = new Plane(supportAxisPos, Vector3d.XAxis);
                BoundingBox intersect = generateIntersection(intersectPlane, body).GetBoundingBox(true);
                double yMax = intersect.Max.Y - 1;
                double yMin = intersect.Min.Y + 1;
                if (yMax < supportAxisPos.Y || yMin > supportAxisPos.Y)
                    return false;
                //Construct Axis
                Point3d axisMinEnd = new Point3d(supportAxisPos.X, yMin, supportAxisPos.Z);
                Brep axis = new Cylinder(new Circle(Plane.WorldZX, axisMinEnd, axisRadius), yMax - yMin).ToBrep(true, true);
                locks.Add(axis);
                //Construct supports which connect axis to body
                double supportWidth = 8;
                double supportThick = 1;
                double gap = 5;
                BoundingBox bboxS = new BoundingBox(axisMinEnd.X - supportWidth / 2, axisMinEnd.Y, (intersect.Min.Z + intersect.Max.Z) / 2, axisMinEnd.X + supportWidth / 2, axisMinEnd.Y + supportThick, axisMinEnd.Z + axisRadius + gap);
                Brep support = new Box(bboxS).ToBrep();
                support = Brep.CreateBooleanDifference(support, body, RhinoMath.SqrtEpsilon)[0];
                locks.Add(support);
                Brep supportMirror = support.DuplicateBrep();
                var xmirror = Transform.Mirror(new Point3d(axisMinEnd.X, (yMin + yMax) / 2, axisMinEnd.Z), Vector3d.YAxis);
                supportMirror.Transform(xmirror);
                locks.Add(supportMirror);
                gap = 2;
                if (supportPosMaxY > yMax - gap) supportPosMaxY = yMax - gap;
                if (supportPosMinY < yMin + gap) supportPosMinY = yMin + gap;
                return true;
            }

            public bool SetTriggerPosition()
            {
                BoundingBox bboxB = body.GetBoundingBox(true);
                double gap = 10;
                double supportPosMaxX = bboxB.Max.X;
                double supportPosMinX = axisPoint.X + gap;
                if (supportPosMaxX < supportPosMinX)
                    return false;
                /*
                if (triggerPos != Point3d.Unset)
                {
                    //Test whether current point can still work
                    if (triggerPos.X > supportPosMinX && triggerPos.X < supportPosMaxX && triggerPos.Y < supportPosMaxY && triggerPos.Y > supportPosMinY)
                        return true;
                }
                */
                List<Point3d> points = new List<Point3d>();
                double X = supportPosMinX;
                double Z = supportAxisPos.Z;
                double pointGap = 5;
                while (X < supportPosMaxX)
                {
                    double Y = supportPosMinY;
                    while (Y < supportPosMaxY)
                    {
                        Point3d p = new Point3d(X, Y, Z);
                        p.Transform(xrotateBack);
                        points.Add(p);
                        Y += pointGap;
                    }
                    X += pointGap;
                }
                if (points.Count == 0)
                    return false;
                RhinoList<Guid> Id = myDoc.Objects.AddPoints(points);
                Point3d selectedPosition = Point3d.Unset;
                /*
                while (selectedPosition == Point3d.Unset)
                {
                    ObjRef obj_ref;
                    var rcommand = RhinoGet.GetOneObject("Set the position of trigger button", false, ObjectType.Point, out obj_ref);
                    if (rcommand == Rhino.Commands.Result.Success)
                    {
                        Guid guid = obj_ref.ObjectId;
                        foreach (Guid g in Id)
                        {
                            if (g == guid)
                            {
                                selectedPosition = obj_ref.Point().Location;
                                selectedPosition.Transform(xrotate);
                                break;
                            }
                        }
                        if (selectedPosition == Point3d.Unset)
                        {
                            RhinoApp.WriteLine("The point is invalid.");
                        }
                    }
                }
                */
                ObjRef obj_ref;
                var rcommand = RhinoGet.GetOneObject("Set the position of trigger button", false, ObjectType.Point, out obj_ref);
                if (rcommand == Rhino.Commands.Result.Success)
                {
                    Guid guid = obj_ref.ObjectId;
                    foreach (Guid g in Id)
                    {
                        if (g == guid)
                        {
                            selectedPosition = obj_ref.Point().Location;
                            selectedPosition.Transform(xrotate);
                            break;
                        }
                    }
                    if (selectedPosition == Point3d.Unset)
                    {
                        RhinoApp.WriteLine("The point is invalid.");
                    }
                }
                foreach (Guid g in Id)
                {
                    myDoc.Objects.Delete(g, true);
                }
                if (selectedPosition == Point3d.Unset)
                    return false;
                triggerPos = selectedPosition;
                return true;
            }

            public bool ConstructBlockSupport()
            {
                //Construct Trigger
                double widthT = 5;
                double heightT = 5;
                double lenT = 5;
                BoundingBox bboxT = new BoundingBox(triggerPos.X - lenT / 2, triggerPos.Y - widthT / 2, supportAxisPos.Z, triggerPos.X + lenT / 2, triggerPos.Y + widthT / 2, supportAxisPos.Z + heightT);
                Brep trigger = new Box(bboxT).ToBrep();
                locks.Add(trigger);
                //Construct Support
                double thickS = 1;
                BoundingBox bboxS = new BoundingBox(supportAxisPos.X, supportPosMinY, supportAxisPos.Z - thickS / 2, bboxT.Max.X, supportPosMaxY, supportAxisPos.Z + thickS / 2);
                Brep support = new Box(bboxS).ToBrep();
                locks.Add(support);
                return true;
            }

            public bool RotateBack()
            {
                if (unit.Count == 0)
                    return false;
                foreach (Brep brep in unit)
                    brep.Transform(xrotateBack);
                if (addLock)
                {
                    foreach (Brep brep in locks)
                        brep.Transform(xrotateBack);
                }
                return true;
            }

            public List<Brep> getUnit()
            {
                return unit;
            }

            public List<Brep> getLock()
            {
                return locks;
            }

        }

    }
}
