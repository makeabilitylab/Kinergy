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


namespace Kinergy
{
    namespace Geom
    {
        public class Rack : Component
        {
            private double length = 0;
            private double rootHeight = 0;
            private double tipHeight = 0;
            private double toothDepth = 0;//=tR-rR;
            //private double factorA = -1, factorB = -1;//Cancelled since adjustment of teeth should be automatically done.
            private int z;//Total number of teeth
            private double pitch = 0;//pitch ,distance between corresponding points on adjacent teeth. p=m*Pi;
            private double module = 0;//module of teeth size, 
            private double faceWidth = 0;
            private double pressureAngle = 0;
            private Point3d centerPoint = Point3d.Unset;
            private Point3d startPoint = Point3d.Unset;
            private Point3d endPoint = Point3d.Unset;
            private Plane rackPlane = new Plane();
            private Vector3d faceDirection = Vector3d.Unset;
            private Vector3d rackDirection = Vector3d.Unset;
            private Vector3d extrudeDirection = Vector3d.Unset;
            private Line backBone = Line.Unset;

            public double Length { get => length; private set => length = value; }
            public double RootHeight { get => rootHeight; private set => rootHeight = value; }
            public double TipHeight { get => tipHeight; private set => tipHeight = value; }
            public double ToothDepth { get => toothDepth; private set => toothDepth = value; }
            /// <summary>
            /// Number of teeth. It's auto calculated using module in constructor.
            /// </summary>
            public int Z { get => z; private set => z = value; }
            /// <summary>
            /// Actual distance between teeth.
            /// </summary>
            public double Pitch { get => pitch; private set => pitch = value; }
            public double Module { get => module; private set => module = value; }
            public double FaceWidth { get => faceWidth; private set => faceWidth = value; }
            public Point3d CenterPoint { get => centerPoint; private set => centerPoint = value; }
            public Point3d StartPoint { get => startPoint; private set => startPoint = value; }
            public Point3d EndPoint { get => endPoint; private set => endPoint = value; }
            public Vector3d FaceDirection { get => faceDirection; private set => faceDirection = value; }
            public Line BackBone { get => backBone; private set => backBone = value; }
            public Vector3d RackDirection { get => rackDirection; private set => rackDirection = value; }

            public Rack(Point3d CP, Vector3d RackLineDirection, Vector3d RackFaceDirection, double len, double rH, double tD, double m = 1, double Thickness = 4)
            {
                rackDirection = RackLineDirection / RackLineDirection.Length;
                faceDirection = RackFaceDirection;
                faceDirection.Unitize();
                length = len;
                faceWidth = Thickness;
                centerPoint = CP;
                startPoint = CP - RackLineDirection / RackLineDirection.Length * (len / 2);
                endPoint = CP + RackLineDirection / RackLineDirection.Length * (len / 2);
                backBone = new Line(startPoint, endPoint);
                module = m;
                pitch = m * Math.PI;
                RootHeight = rH;
                toothDepth = tD;
                tipHeight = rootHeight + toothDepth;
                z = (int)Math.Round(length / pitch);
                pitch = length / z;
                extrudeDirection = new Plane(CP, rackDirection, faceDirection).Normal;
                Generate();
            }

            public Rack(Point3d start, Point3d end, Vector3d RackFaceDirection, double rH, double tD, double m = 1, double Thickness = 4)
            {
                backBone = new Line(start, end);
                centerPoint = backBone.PointAt(0.5);
                rackDirection = new Vector3d(end) - new Vector3d(start);
                rackDirection /= rackDirection.Length;
                faceDirection = RackFaceDirection;
                faceDirection.Unitize();
                length = backBone.Length;
                faceWidth = Thickness;
                startPoint = start;
                endPoint = end;

                module = m;
                pitch = m * Math.PI;
                RootHeight = rH;
                toothDepth = tD;
                tipHeight = rootHeight + toothDepth;
                z = (int)Math.Round(length / pitch);
                pitch = length / z;
                extrudeDirection = new Plane(centerPoint, rackDirection, faceDirection).Normal;
                Generate();
            }

            public Rack(Point3d start, Point3d end, double m, double pressure_angle = 20)
            {
                startPoint = start;
                endPoint = end;
                rackDirection = new Vector3d(end) - new Vector3d(start);
                length = start.DistanceTo(end);
                module = m;
                pitch = m * Math.PI;
                pressureAngle = pressure_angle * Math.PI / 180;
                Generate();
            }

            public Rack(Point3d start, Point3d end, Point3d directionPoint, double m, double pressure_angle = 20)
            {
                startPoint = start;
                endPoint = end;
                rackDirection = new Vector3d(end) - new Vector3d(start);
                rackPlane = new Plane(startPoint, endPoint, directionPoint);
                //extrudeDirection = rackPlane.Normal;
                length = start.DistanceTo(end);
                module = m;
                pitch = m * Math.PI;
                pressureAngle = pressure_angle * Math.PI / 180;
                Generate();
            }

            private void FixParameter()
            {

            }

            /*
            protected override void GenerateBaseCurve()
            {
                FixParameter();
                
                Transform move = Transform.Translation(faceDirection * rootHeight);
                Curve rootLine = backBone.ToNurbsCurve();
                rootLine.Transform(move);
                List<Point3d> pts = new List<Point3d>();
                move = Transform.Translation(faceDirection * toothDepth);
                for (int i=0;i<=z*4;i++)
                {
                    pts.Add(rootLine.PointAtNormalizedLength(i / (z * 4)));
                    if(i % 4 == 1 || i % 4 == 2)
                    {
                        pts[i].Transform(move);
                    }
                }
                pts.Add(endPoint);
                pts.Add(startPoint);
                List<Curve> LineSegments = new List<Curve>();
                for(int i=1;i<z*4+3;i++)
                {
                    LineSegments.Add(new Line(pts[i - 1], pts[i]).ToNurbsCurve());
                }
                LineSegments.Add(new Line(pts[z * 4 + 2], pts[0]).ToNurbsCurve());
                base.BaseCurve = PolyCurve.JoinCurves(LineSegments)[0];
            }
            */

            protected override void GenerateBaseCurve()
            {
                double tipWidth = pitch / 2 - 2 * module * Math.Tan(pressureAngle);
                int numInstance = (int)((length - tipWidth) / (2 * pitch)) + 1;
                RhinoApp.WriteLine("get base parameters" + module + "  " + pressureAngle + "   " + numInstance);
                List<double> yVals1 = new List<double>();
                yVals1.Add(module);
                for (int i = 0; i < numInstance; i++)
                {
                    yVals1.Add(-module);
                    yVals1.Add(-module);
                    yVals1.Add(module);
                    yVals1.Add(module);
                }
                //RhinoApp.WriteLine("get rack y1 values" + yVals1.Count());
                List<double> yVals2 = new List<double>();
                for (int i = 0; i < yVals1.Count(); i++)
                {
                    yVals2.Add(yVals1[i]);
                }
                //RhinoApp.WriteLine("get rack y2 values" + yVals2.Count());
                yVals1.Reverse();
                //RhinoApp.WriteLine("get rack y1 values" + yVals1.Count());

                for (int i = 0; i < yVals2.Count(); i++)
                {
                    //RhinoApp.WriteLine("yvals2 rank" + i);
                    //RhinoApp.WriteLine("yvals2 value" + yVals2[i]);
                    yVals1.Add(yVals2[i]);
                }

                RhinoApp.WriteLine("get rack y values" + yVals1.Count());

                List<double> xVals0 = new List<double>();
                double x0 = tipWidth / 2;
                double x1 = pitch / 2 - tipWidth;
                xVals0.Add(x0);
                for (int i = 0; i < 2 * numInstance; i++)
                {
                    xVals0.Add(x1);
                    xVals0.Add(2 * x0);
                }
                List<double> xVals1 = new List<double>();
                double xSum = 0;
                for (int i = 0; i < xVals0.Count(); i++)
                {
                    xSum += xVals0[i];
                    xVals1.Add(xSum);
                }
                List<double> xVals2 = new List<double>();
                for (int i = 0; i < xVals1.Count(); i++)
                {
                    xVals2.Add(xVals1[i]);
                }
                xVals1.Reverse();
                for (int i = 0; i < xVals1.Count(); i++)
                {
                    xVals1[i] = -xVals1[i];
                }
                for (int i = 0; i < xVals2.Count(); i++)
                {
                    xVals1.Add(xVals2[i]);
                }
                RhinoApp.WriteLine("get rack x values" + xVals1);

                List<Point3d> rackPts = new List<Point3d>();
                double thickness = 3;
                Point3d rackStart = new Point3d(xVals1[0], 0, yVals1[0] - 2 * module - thickness);
                Point3d rackEnd = new Point3d(xVals1[xVals1.Count() - 1], 0, yVals1[yVals1.Count() - 1] - 2 * module - thickness);

                rackPts.Add(rackStart);

                for (int i = 0; i < xVals1.Count(); i++)
                {
                    rackPts.Add(new Point3d(xVals1[i], 0, yVals1[i]));
                }

                rackPts.Add(rackEnd);
                rackPts.Add(rackStart);

                Curve rackCrv = new Polyline(rackPts).ToNurbsCurve();

                Transform transRack = Transform.Translation(new Vector3d(0, 0, module + thickness));
                rackCrv.Transform(transRack);
                //Vector3d rackDirec = new Vector3d(startPoint.X - endPoint.X, startPoint.Y - endPoint.Y, startPoint.Z - endPoint.Z);
                //cover rack to the start-end direction

                base.BaseCurve = rackCrv;
            }


            protected void GenerateRack()
            {
                GenerateBaseCurve();
                RhinoApp.WriteLine("ready to transform rack");
                faceWidth = 2.6;
                //Plane currPlane = new Plane();
                //base.BaseCurve.TryGetPlane(out currPlane);
                //RhinoApp.WriteLine("curr y Axis" + currPlane.YAxis);
                //RhinoApp.WriteLine("trans x Axis" + rackPlane.XAxis);
                //RhinoApp.WriteLine("trans y Axis" + rackPlane.YAxis);
                //Transform corTrans = Transform.ChangeBasis(currPlane, rackPlane);
                //base.BaseCurve.Transform(corTrans);
                Point3d midPoint = new Point3d((startPoint.X + EndPoint.X) / 2, (startPoint.Y + EndPoint.Y) / 2, (startPoint.Z + EndPoint.Z) / 2);
                Transform trans0 = Transform.Translation(new Vector3d(midPoint));
                Transform rotat0 = Transform.Rotation(new Vector3d(1, 0, 0), rackDirection, midPoint);
                base.BaseCurve.Transform(trans0);
                base.BaseCurve.Transform(rotat0);

                /*
                Vector3d rackX = rackDirection;
                Vector3d rackZ = rackPlane.Normal;
                Vector3d rackY = new Vector3d(rackX.Y * rackZ.Z - rackX.Y * rackZ.Z,
                rackX.Z * rackZ.X - rackX.Z * rackZ.X,
                rackX.X * rackZ.Y - rackX.X * rackZ.Y);

                Transform corTrans = Transform.ChangeBasis(new Vector3d(1,0,0), new Vector3d(0,0,1), new Vector3d(0,1,0),
                    rackX, rackY, rackZ);
                base.BaseCurve.Transform(corTrans);
                */

                Extrusion extrudeCrv = Extrusion.Create(base.BaseCurve, faceWidth, true);
                base.Model = extrudeCrv.ToBrep();

                //Transform rotatNormal = Transform.Rotation(new Vector3d(0, 1, 0), extrudeDirection, midPoint);
                //base.model.Transform(rotatNormal);
                //Point3d startPoint = centerPoint - extrudeDirection * length*0.5;
                //Point3d endPoint = centerPoint + extrudeDirection * length * 0.5;
                //Curve rail = new Line(startPoint, endPoint).ToNurbsCurve();
                //base.Model = Brep.CreateFromSweep(rail, base.BaseCurve, true, 0.00000001)[0];
            }


            public override void Generate()
            {
                GenerateRack();
            }
            public override bool IsEngaged(Component obj)
            {

                if (obj.GetType() == typeof(Gear))
                {
                    Gear g = (Gear)obj;
                    //If two gears are engaged, they have to be in same direction, their interval of z direction have to overlap
                    if (Math.Abs(g.Direction * extrudeDirection) < 0.99)
                    { return false; }
                    if (Math.Abs(g.Direction * new Vector3d(g.CenterPoint)) - Math.Abs(extrudeDirection * new Vector3d(CenterPoint)) > (faceWidth + g.FaceWidth) / 2)
                    { return false; }
                    //and the distance of centers have to be between R1+R2 and r1+r2+h(smaller).
                    double distance1 = backBone.DistanceTo(g.CenterPoint, true);
                    double distance2 = new Plane(CenterPoint, faceDirection, rackDirection).DistanceTo(g.CenterPoint);
                    double distance3 = Math.Sqrt(distance1 * distance1 - distance2 * distance2);
                    if (distance3 > tipHeight + g.TipRadius || distance3 < rootHeight + g.RootRadius + (toothDepth + g.ToothDepth) / 2)
                    { return false; }
                    //Their teeth relationship
                    if (Math.Abs(module - g.Module) > 0.01)
                    { return false; }
                    //Actually the engagement of gears could be very complicated. Herer I only cover the basic parts due to my very limited knowledge in this field.

                }
                else { return false; }
                return true;
            }
        }
    }
}
