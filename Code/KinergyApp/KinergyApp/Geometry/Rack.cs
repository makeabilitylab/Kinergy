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
            private double _length = 0;
            private double _rootHeight = 0;
            private double _tipHeight = 0;
            private double _thickness = 0; // the thickness of the rack base
            private double _toothDepth = 0;//=tR-rR;
            //private double factorA = -1, factorB = -1;//Cancelled since adjustment of teeth should be automatically done.
            private int _z;//Total number of teeth
            private double _pitch = 0;//pitch ,distance between corresponding points on adjacent teeth. p=m*Pi;
            private double _module = 0;//module of teeth size, 
            private double _faceWidth = 0;
            private double _pressureAngle = 0;
            private Point3d _centerPoint = Point3d.Unset;
            private Point3d _startPoint = Point3d.Unset;
            private Point3d _endPoint = Point3d.Unset;
            private Plane _rackPlane = new Plane();
            private Vector3d _faceDirection = Vector3d.Unset;
            private Vector3d _rackDirection = Vector3d.Unset;
            private Vector3d _extrudeDirection = Vector3d.Unset;
            private Line _backBone = Line.Unset;

            RhinoDoc myDoc = RhinoDoc.ActiveDoc;

            public double Length { get => _length; private set => _length = value; }
            public double RootHeight { get => _rootHeight; private set => _rootHeight = value; }
            public double TipHeight { get => _tipHeight; private set => _tipHeight = value; }
            public double ToothDepth { get => _toothDepth; private set => _toothDepth = value; }
            /// <summary>
            /// Number of teeth. It's auto calculated using module in constructor.
            /// </summary>
            public int Z { get => _z; private set => _z = value; }
            /// <summary>
            /// Actual distance between teeth.
            /// </summary>
            public double Pitch { get => _pitch; private set => _pitch = value; }
            public double Module { get => _module; private set => _module = value; }
            public double FaceWidth { get => _faceWidth; private set => _faceWidth = value; }
            public Point3d CenterPoint { get => _centerPoint; private set => _centerPoint = value; }
            public Point3d StartPoint { get => _startPoint; private set => _startPoint = value; }
            public Point3d EndPoint { get => _endPoint; private set => _endPoint = value; }
            public Vector3d FaceDirection { get => _faceDirection; private set => _faceDirection = value; }
            public Line BackBone { get => _backBone; private set => _backBone = value; }
            public Vector3d RackDirection { get => _rackDirection; private set => _rackDirection = value; }

            public Rack(Point3d CP, Vector3d RackLineDirection, Vector3d RackFaceDirection, double len, double m, double Thickness, Vector3d extrusionDir, double rackThickness, double pressureAngle)
            {
                _rackDirection = RackLineDirection / RackLineDirection.Length;
                _faceDirection = RackFaceDirection;
                _faceDirection.Unitize();
                _length = len;
                _faceWidth = Thickness;
                _centerPoint = CP;
                _startPoint = CP - RackLineDirection / RackLineDirection.Length * (len / 2);
                _endPoint = CP + RackLineDirection / RackLineDirection.Length * (len / 2);
                _backBone = new Line(_startPoint, _endPoint);
                _module = m;
                _pitch = m * Math.PI;
                _tipHeight = _rootHeight + _toothDepth;
                _z = (int)Math.Floor(_length / _pitch);
                _pitch = _length / _z;
                _extrudeDirection = extrusionDir;
                _thickness = rackThickness;
                _pressureAngle = pressureAngle * Math.PI / 180;
                Generate();
            }

            //public Rack(Point3d start, Point3d end, Vector3d RackFaceDirection, double rH, double tD, double m, double Thickness, Vector3d extrusionDir, double thickness, double pressAngle)
            //{
            //    _backBone = new Line(start, end);
            //    _centerPoint = _backBone.PointAt(0.5);
            //    _rackDirection = new Vector3d(end) - new Vector3d(start);
            //    _rackDirection /= _rackDirection.Length;
            //    _faceDirection = RackFaceDirection;
            //    _faceDirection.Unitize();
            //    _length = _backBone.Length;
            //    _faceWidth = Thickness;
            //    _startPoint = start;
            //    _endPoint = end;
            //    _thickness = thickness;
            //    _pressureAngle = pressAngle * Math.PI / 180;
            //    _module = m;
            //    _pitch = m * Math.PI;
            //    _rootHeight = rH;
            //    _toothDepth = tD;
            //    _tipHeight = _rootHeight + _toothDepth;
            //    _z = (int)Math.Floor(_length / _pitch);
            //    _pitch = _length / _z;
            //    _extrudeDirection = extrusionDir;
            //    Generate();
            //}

            //public Rack(Point3d start, Point3d end, double m, double pressure_angle, double thickness)
            //{
            //    _startPoint = start;
            //    _endPoint = end;
            //    _rackDirection = new Vector3d(end) - new Vector3d(start);
            //    _length = start.DistanceTo(end);
            //    _module = m;
            //    _pitch = m * Math.PI;
            //    _pressureAngle = pressure_angle * Math.PI / 180;
            //    _thickness = thickness;
            //    Generate();
            //}

            public Rack(Point3d start, Point3d end, Point3d directionPoint, double m, double pressure_angle = 20)
            {
                _startPoint = start;
                _endPoint = end;
                _rackDirection = new Vector3d(end) - new Vector3d(start);
                _rackPlane = new Plane(_startPoint, _endPoint, directionPoint);
                //extrudeDirection = rackPlane.Normal;
                _length = start.DistanceTo(end);
                _module = m;
                _pitch = m * Math.PI;
                _pressureAngle = pressure_angle * Math.PI / 180;
                Generate();
            }

            protected override void GenerateBaseCurve()
            {
                // More informaiton about rack generation: https://khkgears.net/new/gear_knowledge/gear_technical_reference/involute_gear_profile.html

                _pitch = Math.PI * _module;

                double Rf = 0.38 * _module;
                double c = 0.25 * _module;
                double L1 = Math.Sqrt(Math.Pow(Rf, 2) - Math.Pow(Rf - c, 2));
                double toothWidth = _pitch - 2 * L1 - 4 * _module * Math.Tan(_pressureAngle);

                // generate the half valley before the tooth

                Arc firstValley = new Arc(new Point3d(0,0,0), new Vector3d(1,0,0),new Point3d(L1, c,0));
                Curve firstValleyCrv = firstValley.ToNurbsCurve();

                // generate the tooth

                Curve toothFlank = new Line(new Point3d(L1, c, 0), new Point3d(L1 + Math.Tan(_pressureAngle) * 2 * _module, c + 2 * _module, 0)).ToNurbsCurve();
                Curve toothTipHalf = new Line(new Point3d(L1 + Math.Tan(_pressureAngle) * 2 * _module, c + 2 * _module, 0), new Point3d(L1 + Math.Tan(_pressureAngle) * 2 * _module + toothWidth / 2, c + 2 * _module, 0)).ToNurbsCurve();

                // generate the valley after the tooth
                Curve toothHalfCrv = Curve.JoinCurves(new List<Curve> { firstValleyCrv, toothFlank, toothTipHalf }, myDoc.ModelAbsoluteTolerance, false)[0];

                Transform mirror = Transform.Mirror(new Point3d(L1 + Math.Tan(_pressureAngle) * 2 * _module + toothWidth / 2, 0, 0), new Vector3d(1, 0, 0));
                Curve toothSecondHalfCrv = toothHalfCrv.DuplicateCurve();
                toothSecondHalfCrv.Transform(mirror);

                Curve toothProfileCrv = Curve.JoinCurves(new List<Curve> { toothHalfCrv, toothSecondHalfCrv }, myDoc.ModelAbsoluteTolerance, false)[0];

                // generate all the teeth on the rack

                _z = (int)Math.Floor(_length / _pitch);
                double leftover = _length - _z * _pitch;
                List<Curve> rackTeeth = new List<Curve>();

                for (int i= 0; i <_z; i++)
                {
                    Transform move = Transform.Translation(new Vector3d(_pitch * i, 0, 0));
                    Curve tempCrv = toothProfileCrv.DuplicateCurve();
                    tempCrv.Transform(move);
                    rackTeeth.Add(tempCrv);
                }
                rackTeeth.Add(new Line(new Point3d(_z * _pitch, 0, 0), new Point3d(_length, 0, 0)).ToNurbsCurve());
                rackTeeth.Add(new Line(new Point3d(_length, 0, 0), new Point3d(_length, -_thickness, 0)).ToNurbsCurve());
                rackTeeth.Add(new Line(new Point3d(_length, -_thickness, 0), new Point3d(0, -_thickness, 0)).ToNurbsCurve());
                rackTeeth.Add(new Line(new Point3d(0, -_thickness, 0), new Point3d(0, 0, 0)).ToNurbsCurve());

                base.BaseCurve = Curve.JoinCurves(rackTeeth, myDoc.ModelAbsoluteTolerance, false)[0];

                #region old generation algorithm


                //double tipWidth = pitch / 2 - 2 * module * Math.Tan(pressureAngle);
                //int numInstance = (int)((length - tipWidth) / (2 * pitch)) + 1;
                //RhinoApp.WriteLine("get base parameters" + module + "  " + pressureAngle + "   " + numInstance);
                //List<double> yVals1 = new List<double>();
                //yVals1.Add(module);
                //for (int i = 0; i < numInstance; i++)
                //{
                //    yVals1.Add(-module);
                //    yVals1.Add(-module);
                //    yVals1.Add(module);
                //    yVals1.Add(module);
                //}
                ////RhinoApp.WriteLine("get rack y1 values" + yVals1.Count());
                //List<double> yVals2 = new List<double>();
                //for (int i = 0; i < yVals1.Count(); i++)
                //{
                //    yVals2.Add(yVals1[i]);
                //}
                ////RhinoApp.WriteLine("get rack y2 values" + yVals2.Count());
                //yVals1.Reverse();
                ////RhinoApp.WriteLine("get rack y1 values" + yVals1.Count());

                //for (int i = 0; i < yVals2.Count(); i++)
                //{
                //    //RhinoApp.WriteLine("yvals2 rank" + i);
                //    //RhinoApp.WriteLine("yvals2 value" + yVals2[i]);
                //    yVals1.Add(yVals2[i]);
                //}

                //RhinoApp.WriteLine("get rack y values" + yVals1.Count());

                //List<double> xVals0 = new List<double>();
                //double x0 = tipWidth / 2;
                //double x1 = pitch / 2 - tipWidth;
                //xVals0.Add(x0);
                //for (int i = 0; i < 2 * numInstance; i++)
                //{
                //    xVals0.Add(x1);
                //    xVals0.Add(2 * x0);
                //}
                //List<double> xVals1 = new List<double>();
                //double xSum = 0;
                //for (int i = 0; i < xVals0.Count(); i++)
                //{
                //    xSum += xVals0[i];
                //    xVals1.Add(xSum);
                //}
                //List<double> xVals2 = new List<double>();
                //for (int i = 0; i < xVals1.Count(); i++)
                //{
                //    xVals2.Add(xVals1[i]);
                //}
                //xVals1.Reverse();
                //for (int i = 0; i < xVals1.Count(); i++)
                //{
                //    xVals1[i] = -xVals1[i];
                //}
                //for (int i = 0; i < xVals2.Count(); i++)
                //{
                //    xVals1.Add(xVals2[i]);
                //}
                //RhinoApp.WriteLine("get rack x values" + xVals1);

                //List<Point3d> rackPts = new List<Point3d>();
                //double thickness = 3;
                //Point3d rackStart = new Point3d(xVals1[0], 0, yVals1[0] - 2 * module - thickness);
                //Point3d rackEnd = new Point3d(xVals1[xVals1.Count() - 1], 0, yVals1[yVals1.Count() - 1] - 2 * module - thickness);

                //rackPts.Add(rackStart);

                //for (int i = 0; i < xVals1.Count(); i++)
                //{
                //    rackPts.Add(new Point3d(xVals1[i], 0, yVals1[i]));
                //}

                //rackPts.Add(rackEnd);
                //rackPts.Add(rackStart);

                //Curve rackCrv = new Polyline(rackPts).ToNurbsCurve();

                //Transform transRack = Transform.Translation(new Vector3d(0, 0, module + thickness));
                //rackCrv.Transform(transRack);
                ////Vector3d rackDirec = new Vector3d(startPoint.X - endPoint.X, startPoint.Y - endPoint.Y, startPoint.Z - endPoint.Z);
                ////cover rack to the start-end direction
                //base.BaseCurve = rackCrv;

                #endregion

            }


            protected void GenerateRack()
            {
                GenerateBaseCurve();

                RhinoApp.WriteLine("ready to transform rack");
                _faceWidth = 3.6;

                var sweep = new SweepOneRail();
                sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
                sweep.ClosedSweep = false;
                sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

                Curve rackPathCrv = new Line(new Point3d(0, 0, 0), new Point3d(0, 0, _faceWidth)).ToNurbsCurve();
                Brep[] rackBreps = sweep.PerformSweep(rackPathCrv, base.BaseCurve);
                Brep rackBrep = rackBreps[0];
                Brep rackSolid = rackBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                rackSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == rackSolid.SolidOrientation)
                    rackSolid.Flip();

                base.Model = rackSolid;

                Point3d baseMidPt = new Point3d(_length / 2, 0, 0);
                Point3d midPoint = (_startPoint + _endPoint) / 2;
                Transform trans0 = Transform.Translation(new Vector3d(midPoint - baseMidPt));
                Transform rotat0 = Transform.Rotation(new Vector3d(1, 0, 0), _rackDirection, midPoint);
                Transform selfRotate = Transform.Rotation(new Vector3d(0, 1, 0), _faceDirection, midPoint);
                base.BaseCurve.Transform(trans0);
                base.BaseCurve.Transform(rotat0);
                base.BaseCurve.Transform(selfRotate);
                base.Model.Transform(trans0);
                base.Model.Transform(rotat0);
                base.Model.Transform(selfRotate);
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
                    if (Math.Abs(g.Direction * _extrudeDirection) < 0.99)
                    { return false; }
                    if (Math.Abs(g.Direction * new Vector3d(g.CenterPoint)) - Math.Abs(_extrudeDirection * new Vector3d(CenterPoint)) > (_faceWidth + g.FaceWidth) / 2)
                    { return false; }
                    //and the distance of centers have to be between R1+R2 and r1+r2+h(smaller).
                    double distance1 = _backBone.DistanceTo(g.CenterPoint, true);
                    double distance2 = new Plane(CenterPoint, _faceDirection, _rackDirection).DistanceTo(g.CenterPoint);
                    double distance3 = Math.Sqrt(distance1 * distance1 - distance2 * distance2);
                    if (distance3 > _tipHeight + g.TipRadius || distance3 < _rootHeight + g.RootRadius + (_toothDepth + g.ToothDepth) / 2)
                    { return false; }
                    //Their teeth relationship
                    if (Math.Abs(_module - g.Module) > 0.01)
                    { return false; }
                    //Actually the engagement of gears could be very complicated. Herer I only cover the basic parts due to my very limited knowledge in this field.

                }
                else { return false; }
                return true;
            }
        }
    }
}
