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
        public class Gear : Component
        {
            //private double factorA = -1, factorB = -1;//Cancelled since adjustment of teeth should be automatically done.
            private int _numTeeth = 0;//Total number of teeth
            private double _pitch = 0;//pitch ,distance between corresponding points on adjacent teeth. p=m*Pi;
            private double _module = 0;//module of teeth size, 
            private double _faceWidth = 0;
            private double _pressureAngle = 20; // degree
            private double _coneAngle = 0;
            private double _clearance = 0.167;
            private double _involSample = 10;
            private double _tipRadius = 0;
            private double _rootRadius = 0;
            private double _toothDepth = 0;
            private double _selfRotAngle = 0; // degree
            private bool _isMovable = false;
            private Vector3d _x_direction = Vector3d.Unset;
            private Vector3d _x_original_direction = new Vector3d(1, 0, 0);

            private RhinoDoc mydoc = RhinoDoc.ActiveDoc;

            private Point3d _centerPoint = Point3d.Unset;
            private Vector3d _direction = Vector3d.Unset;
            private Boolean _twoLayer = false;
            private List<Vector3d> teethDirections=new List<Vector3d>();
            private List<Point3d> teethTips = new List<Point3d>();

            public int NumTeeth { get => _numTeeth; protected set => _numTeeth = value; }
            public double Pitch { get => _pitch; protected set => _pitch = value; }
            public double Module { get => _module; protected set => _module = value; }
            public double FaceWidth { get => _faceWidth; protected set => _faceWidth = value; }
            public double TipRadius { get => _tipRadius; protected set => _tipRadius = value; }
            public double RootRadius { get => _rootRadius; protected set => _rootRadius = value; }
            public double ToothDepth { get => _toothDepth; protected set => _toothDepth = value; }
            public Point3d CenterPoint { get => _centerPoint; protected set => _centerPoint = value; }
            public Vector3d Direction { get => _direction; protected set => _direction = value; }
            public bool IsMovable { get => _isMovable; set => _isMovable = value; }
            public Vector3d X_direction { get => _x_direction; set => _x_direction = value; }
            public List<Vector3d> TeethDirections { get => teethDirections; }
            public List<Point3d> TeethTips { get => teethTips; }

            //private Brep surface = null;

            /// <summary> Constructor with center point and direction given </summary>
            /// <returns> Returns instance with gear brep generated</returns>
            /// <param name="CP">Center Point of gear</param>
            /// <param name="module">Module of gear teeth, this parameter determines how big teeth are</param>            
            /// <summary> Constructor with parameter but no center point given </summary>
            /// <returns> Returns instance with gear brep generated</returns>            
            public Gear(Point3d center_point, Vector3d gear_direction, Vector3d gear_x_dir, int teethNum, double mod, double pressure_angle, double Thickness, double selfRotAngle, bool movable)
            {
                _centerPoint = center_point;
                _direction = gear_direction;
                _numTeeth = teethNum;
                _module = mod;
                _pressureAngle = pressure_angle;
                _faceWidth = Thickness;
                _twoLayer = false;
                _selfRotAngle = selfRotAngle;
                _isMovable = movable;
                _x_direction = gear_x_dir;

                GenerateGear();
            }

            //protected List<Point3d> GenerateInvolPoints(double baseDiameter, double startAngle, double endAngle, double angleModule, double samples)
            //{
            //    List<Point3d> involPoints = new List<Point3d>();
            //    double step = (startAngle - angleModule - endAngle) / samples;
            //    double baseRadius = baseDiameter / 2;

            //    for (int i = 0; i <= samples; i++)
            //    {
            //        double position = angleModule + i * step;
            //        double height = Math.Sqrt(Math.Pow(position, 2) * Math.Pow(baseRadius, 2) + Math.Pow(baseRadius, 2));
            //        double heightAngle = startAngle - position + Math.Atan(position);

            //        Point3d point = new Point3d(height * Math.Cos(heightAngle), height * Math.Sin(heightAngle), 0);

            //        //RhinoDoc.ActiveDoc.Objects.AddPoint(point);
            //        //RhinoDoc.ActiveDoc.Views.Redraw();
            //        //RhinoApp.Write("involPoint" + point);
            //        involPoints.Add(point);
            //    }

            //    return involPoints;
            //}


            protected Point3d tiltPointAroundCircle(Point3d pt, double coneAngle, double circleDiameter)
            {
                if (coneAngle == 0)
                {
                    return pt;
                }

                coneAngle = (Math.PI / 180) * coneAngle;
                double dist_to_circle = Math.Sqrt(Math.Pow(pt.X, 2) + Math.Pow(pt.Y, 2)) - circleDiameter / 2;
                double xy_scale_factor = circleDiameter / 2 + dist_to_circle * Math.Cos(coneAngle);
                xy_scale_factor = xy_scale_factor / (circleDiameter / 2 + dist_to_circle);

                Point3d tiltPt = new Point3d(pt.X * xy_scale_factor, pt.Y * xy_scale_factor, dist_to_circle * Math.Sin(coneAngle));
                return tiltPt;
            }
            public List<Point3d> GenerateToothSide(double baseDia, double outDia, double sampleNum, double pressureAngle, double module, double s0, double pitchDia)
            {
                List<Point3d> result = new List<Point3d>();
                double stepLen = (outDia - baseDia) / sampleNum;

                for(int i = 0; i <= sampleNum; i++)
                {
                    double d = baseDia + i * stepLen;
                    double alpha = Math.Acos(pitchDia / d * Math.Cos(pressureAngle));
                    double theta = s0 / pitchDia + Math.Tan(pressureAngle) - pressureAngle + alpha - Math.Tan(alpha);

                    Point3d pt = new Point3d(-d/2 * Math.Sin(theta), d/2 * Math.Cos(theta), 0);
                    result.Add(pt);
                }

                return result;
            }

            protected override void GenerateBaseCurve()
            {
                double pressureAngle = (Math.PI / 180) * _pressureAngle; // convert the pressure angle into radian

                double pitchDiameter = _module * _numTeeth;
                double baseDiameter = pitchDiameter * Math.Cos(pressureAngle);
                double addendum = _module;
                double dedendum = (1 + _clearance) * _module;
                double outDiameter = pitchDiameter + 2 * _module;
                double rootDiameter = pitchDiameter - 2 * dedendum;
                //double chordalThickness = pitchDiameter * Math.Sin((Math.PI / 2) / _numTeeth);
                //double x = 1 - (_numTeeth * Math.Pow(Math.Sin(pressureAngle), 2)) / 2.0;
                double x = 0;
                double chordalThickness = Math.PI / 2 * _module + _module * 2 * x * Math.Tan(pressureAngle);
                
                //double chordalThickness = (Math.PI / 2 + 2 * x * Math.Tan(pressureAngle)) * _module; 

                _tipRadius = outDiameter / 2;
                _rootRadius = rootDiameter / 2;
                _toothDepth = _tipRadius - _rootRadius;
                _pitch = Math.PI * _module;

                // More details about involute spur gear: https://www.tec-science.com/mechanical-power-transmission/involute-gear/calculation-of-involute-gears/
                double fi = Math.Tan(pressureAngle) - pressureAngle;
                double maxAngle = Math.Sqrt(Math.Pow(outDiameter / 2, 2) - Math.Pow(baseDiameter / 2, 2)) / (baseDiameter / 2) + fi;

                List<Point3d> involPts = GenerateToothSide(baseDiameter, outDiameter, _involSample, pressureAngle, _module, chordalThickness, pitchDiameter);

                #region old algorithm for generating the points on the tooth
                //double invol_start_angle = (Math.PI / 2 + Math.Asin(chordalThickness / pitchDiameter)
                //    - pressureAngle
                //    + Math.Sqrt(Math.Pow((pitchDiameter / baseDiameter), 2) - 1));
                //double invol_end_angle = (invol_start_angle - Math.Sqrt(Math.Pow((outDiameter / baseDiameter), 2) - 1));

                ///*
                //RhinoApp.WriteLine("invol_start_angle" + invol_start_angle);
                //RhinoApp.WriteLine("invol_end_angle" + invol_start_angle);
                //*/

                //double invol_angle_module = 0;

                //if (rootDiameter > baseDiameter)
                //{
                //    invol_angle_module = Math.Sqrt(Math.Pow((rootDiameter / baseDiameter), 2) - 1);
                //    RhinoApp.WriteLine("invol_angle_module" + invol_angle_module);
                //}

                //List<Point3d> involPts = GenerateInvolPoints(baseDiameter, invol_start_angle, invol_end_angle, invol_angle_module, _involSample, _module);
                #endregion

                List<Curve> toothCrv = new List<Curve>();
                Curve involCrv1 = Curve.CreateInterpolatedCurve(involPts, 3, CurveKnotStyle.Chord);
                Transform mirror = Transform.Mirror(new Point3d(0, 0, 0), new Vector3d(1, 0, 0));
                Curve involCrv2 = involCrv1.DuplicateCurve();
                involCrv2.Transform(mirror);

                Point3d ptArc = tiltPointAroundCircle(new Point3d(0, _tipRadius, 0), _coneAngle / 2, pitchDiameter);
                //RhinoApp.WriteLine("point on up arc" + ptArc);
                Arc topArc = new Arc(involCrv1.PointAtEnd, ptArc, involCrv2.PointAtEnd);
                Curve arcCrv = new ArcCurve(topArc);

                //dedendum
                if (rootDiameter < baseDiameter)
                {
                    RhinoApp.WriteLine("rootDiameter < baseDiameter");
                    double theta = Math.PI * _module / (2 * baseDiameter);
                    Point3d pt = new Point3d(-_rootRadius * Math.Sin(theta), _rootRadius * Math.Cos(theta), 0);

                    Curve dedCrv1 = new Line(involCrv1.PointAtStart, tiltPointAroundCircle(pt, _coneAngle / 2, pitchDiameter)).ToNurbsCurve();
                    Curve dedCrv2 = dedCrv1.DuplicateCurve();
                    dedCrv2.Transform(mirror);

                    toothCrv.Add(dedCrv1);
                    toothCrv.Add(involCrv1);
                    toothCrv.Add(arcCrv);
                    toothCrv.Add(involCrv2);
                    toothCrv.Add(dedCrv2);
                }
                else
                {
                    toothCrv.Add(involCrv1);
                    toothCrv.Add(arcCrv);
                    toothCrv.Add(involCrv2);
                }

                //RhinoApp.WriteLine("toothCrv got Crvs" + toothCrv.Count());
                Curve tooth = Curve.JoinCurves(toothCrv, mydoc.ModelAbsoluteTolerance, false)[0];

                //tooth bottom
                double angle = 2 * Math.PI / _numTeeth;
                Point3d startPt = tooth.PointAtStart;
                Point3d endPt = tooth.PointAtEnd;
                Point3d btEndPt = new Point3d(endPt.X * Math.Cos(angle) - endPt.Y * Math.Sin(angle),
                    endPt.Y * Math.Cos(angle) + endPt.X * Math.Sin(angle), endPt.Z);
                Point3d bottomPt = new Point3d(-Math.Sin(angle / 2) * rootDiameter / 2, Math.Cos(angle / 2) * rootDiameter / 2, 0);
                bottomPt = tiltPointAroundCircle(bottomPt, _coneAngle / 2, pitchDiameter);
                //RhinoApp.WriteLine("pt on bottom arc" + bottomPt);
                Arc bottomArc = new Arc(startPt, bottomPt, btEndPt);
                Curve bottomCrv = new ArcCurve(bottomArc);
                //RhinoApp.WriteLine("bottom crv" + startPt + bottomPt + btEndPt);

                List<Curve> tooth_with_bottom = new List<Curve>();
                tooth_with_bottom.Add(bottomCrv);
                tooth_with_bottom.Add(tooth);
                //RhinoApp.WriteLine("toothwithbottom got crvs" + tooth_with_bottom.Count());
                tooth = Curve.JoinCurves(tooth_with_bottom, 0.00000001, false)[0];

                List<Curve> gearTooth = new List<Curve>();
                //gearTooth.Add(tooth);
                Vector3d firstDir = new Vector3d(0, 1, 0);
                Point3d firstTip = new Point3d(0, _tipRadius, 0);
                for (int i = 0; i < _numTeeth; i++)
                {
                    Transform rotat = Transform.Rotation(-i * angle, new Vector3d(0, 0, 1), new Point3d(0, 0, 0));
                    Curve nextTooth = tooth.DuplicateCurve();
                    nextTooth.Transform(rotat);
                    gearTooth.Add(nextTooth);
                    Vector3d dir = new Vector3d(firstDir);
                    dir.Transform(rotat);
                    teethDirections.Add(dir);
                    Point3d pt = new Point3d(firstTip);
                    pt.Transform(rotat);
                    teethTips.Add(pt);
                }

                //RhinoApp.WriteLine("geartooth got crvs" + gearTooth.Count());
                Curve gear_tooth_crv = Curve.JoinCurves(gearTooth, 0.1, true)[0];

                //CurveOrientation normalDirection = gear_tooth_crv.ClosedCurveOrientation();
                ////RhinoApp.WriteLine("get curve direction" + normalDirection);
                //if (normalDirection == CurveOrientation.CounterClockwise)
                //{
                //    gear_tooth_crv.Reverse();
                //}

                base.BaseCurve = gear_tooth_crv;
            }

            private void GenerateGear()
            {
                GenerateBaseCurve();
                RhinoApp.WriteLine("extruding facewidth" + _faceWidth);
                var sweep = new SweepOneRail();
                sweep.AngleToleranceRadians = mydoc.ModelAngleToleranceRadians;
                sweep.ClosedSweep = false;
                sweep.SweepTolerance = mydoc.ModelAbsoluteTolerance;
                Curve gearPathCrv = new Line(new Point3d(0, 0, 0), new Point3d(0, 0, _faceWidth)).ToNurbsCurve();
                //Brep[] gearBreps = sweep.PerformSweep(gearPathCrv, base.BaseCurve);
                //Brep gearBrep = gearBreps[0];
                //Brep gearSolid = gearBrep.CapPlanarHoles(mydoc.ModelAbsoluteTolerance);

                Plane tempPln = new Plane();
                base.BaseCurve.TryGetPlane(out tempPln);
                Brep gearSolid = new Brep();

                if (tempPln.Normal == new Vector3d(0, 0, 1))
                {
                    gearSolid = Extrusion.Create(base.BaseCurve, _faceWidth, true).ToBrep();
                }
                else
                {
                    gearSolid = Extrusion.Create(base.BaseCurve, -_faceWidth, true).ToBrep();
                }
                //base.BaseCurve.ClosedCurveOrientation = CurveOrientation.Clockwise
               

                gearSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == gearSolid.SolidOrientation)
                    gearSolid.Flip();

                if (_isMovable)
                {
                    // drill a central hole so that the gear will be movable on the shaft
                    double clearance = 0.4;//Xia's note: added 0.1 to this based on experiment result
                    double shaftRadius = 2;
                    double holdRadius = clearance + shaftRadius;
                    Brep holeBrep = Brep.CreatePipe(gearPathCrv, holdRadius, false, PipeCapMode.Round, true, mydoc.ModelAbsoluteTolerance, mydoc.ModelAngleToleranceRadians)[0];

                    holeBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                    if (BrepSolidOrientation.Inward == holeBrep.SolidOrientation)
                        holeBrep.Flip();

                    gearSolid = Brep.CreateBooleanDifference(gearSolid, holeBrep, mydoc.ModelAbsoluteTolerance)[0];
                }

                base.Model = gearSolid;

                if (_centerPoint != Point3d.Unset)
                {
                    Vector3d centerDirection = new Vector3d(_centerPoint);
                    Transform centerTrans = Transform.Translation(centerDirection);
                    base.Model.Transform(centerTrans);
                    base.BaseCurve.Transform(centerTrans);
                    _x_original_direction.Transform(centerTrans);

                    for (int i = 0; i <teethTips.Count(); i++)
                    {
                        Point3d p = teethTips[i];
                        p.Transform(centerTrans);
                        teethTips[i] = p;
                    }

                }

                if (_direction != Vector3d.Unset)
                {
                    Transform centerRotate = Transform.Rotation(new Vector3d(0, 0, 1), _direction, _centerPoint);
                    base.Model.Transform(centerRotate);
                    base.BaseCurve.Transform(centerRotate);
                    _x_original_direction.Transform(centerRotate);
                    for (int i = 0; i < teethTips.Count(); i++)
                    {
                        Point3d p = teethTips[i];
                        p.Transform(centerRotate);
                        teethTips[i] = p;
                    }
                    for (int i = 0; i < teethDirections.Count(); i++)
                    {
                        Vector3d p =teethDirections[i];
                        p.Transform(centerRotate);
                        teethDirections[i] = p;
                    }
                    //if (base.BaseCurve.ClosedCurveOrientation() == CurveOrientation.Clockwise)
                    //{
                    //    Transform centerRotate = Transform.Rotation(new Vector3d(0, 0, 1), _direction, _centerPoint);
                    //    base.Model.Transform(centerRotate);
                    //}
                    //else
                    //{
                    //    Transform centerRotate = Transform.Rotation(new Vector3d(0, 0, 1), new Vector3d(0, 1, 0), _centerPoint);
                    //    base.Model.Transform(centerRotate);
                    //} 

                    Transform centerRotate1 = Transform.Rotation(_x_original_direction, _x_direction, _centerPoint);
                    base.Model.Transform(centerRotate1);
                    base.BaseCurve.Transform(centerRotate1);
                    for (int i = 0; i < teethTips.Count(); i++)
                    {
                        Point3d p = teethTips[i];
                        p.Transform(centerRotate1);
                        teethTips[i] = p;
                    }
                    for (int i = 0; i < teethDirections.Count(); i++)
                    {
                        Vector3d p = teethDirections[i];
                        p.Transform(centerRotate1);
                        teethDirections[i] = p;
                    }

                }

                double selfRotRad = Math.PI / 180 * _selfRotAngle;
                Transform selfRotation = Transform.Rotation(selfRotRad, _direction, _centerPoint);
                base.Model.Transform(selfRotation);

                for (int i = 0; i < teethTips.Count(); i++)
                {
                    Point3d p = teethTips[i];
                    p.Transform(selfRotation);
                    teethTips[i] = p;
                }
                for (int i = 0; i < teethDirections.Count(); i++)
                {
                    Vector3d p = teethDirections[i];
                    p.Transform(selfRotation);
                    teethDirections[i] = p;
                }
                //generate standard gears first and then do tranlation and rotation from the upper level;
                //Vector3d v = direction / direction.Length * faceWidth;
                //Point3d startPoint = center - v*0.5;
                //Point3d endPoint = center + v*0.5;
                //Curve rail = new Line(startPoint, endPoint).ToNurbsCurve();
                //Brep gearPlane = Brep.CreatePlanarBreps(base.BaseCurve)[0];
                //base.Model = Brep.CreateFromSweep(rail, base.BaseCurve, true, 0.00000001)[0];
                //base.Model = base.Model.CapPlanarHoles(0.00000001);

                //direction = new Vector3d(0, 0, 1);
                //center = new Point3d(0, 0, 0);
                //Transform trans = Transform.Translation(direction);
            }


            public override void Generate()
            {
                GenerateGear();
            }

            public void SetPosition(Point3d CP, Vector3d GearDirection)
            {
                _centerPoint = CP;
                center = CP;
                _direction = GearDirection;
                GenerateGear();
            }


            /// <summary>
            /// Method for telling if the given entity could be engaged with this gear.Minor adjustments would be done if the positions are right but teeth are not engaged
            /// </summary>
            public override bool IsEngaged(Component obj)
            {

                if (obj.GetType() == typeof(Gear))
                {
                    Gear g = (Gear)obj;
                    //If two gears are engaged, they have to be in same direction, their interval of z direction have to overlap
                    Vector3d d1 = _direction / _direction.Length;
                    Vector3d d2 = g.Direction / g.Direction.Length;
                    if (Math.Abs(d1 * d2) < 0.99)
                    { return false; }
                    if (Math.Abs(new Vector3d(CenterPoint) * d1) - Math.Abs(new Vector3d(g.CenterPoint) * d2) > (_faceWidth + g.FaceWidth) / 2)
                    { return false; }
                    //and the distance of centers have to be between R1+R2 and r1+r2+h(smaller)
                    Vector3d vCP1 = new Vector3d(CenterPoint);
                    Vector3d vCP2 = new Vector3d(g.CenterPoint);
                    vCP1 -= vCP1 * d1 * d1;
                    vCP2 -= vCP2 * d2 * d2;
                    double distance = (vCP1 - vCP2).Length;
                    if (distance > _tipRadius + g.TipRadius || distance < _rootRadius + g.RootRadius + (_toothDepth + g.ToothDepth) / 2)
                    { return false; }
                    //Their teeth relationship
                    if (Math.Abs(_module - g.Module) > 0.01)
                    { return false; }
                    //Actually the engagement of gears could be very complicated. Herer I only cover the basic parts due to my very limited knowledge in this field.
                }
                else if (obj.GetType() == typeof(Rack))
                {
                    Rack r = (Rack)obj;
                    //If the gear is engaged with a rack, their direction vector should be prependicular and their interval of z direction have to overlap
                    Vector3d d1 = _direction / _direction.Length;
                    Vector3d d2 = r.RackDirection / r.RackDirection.Length;
                    if (Math.Abs(d1 * d2) > 0.01)
                    { return false; }
                    double distance1 = new Plane(_centerPoint, _direction).DistanceTo(r.CenterPoint);
                    if (distance1 > (_faceWidth + r.FaceWidth) / 2)
                    { return false; }
                    //and the distance should be between R1+R2 and r1+r2+h(smaller).
                    double distance2 = r.BackBone.DistanceTo(_centerPoint, true);
                    if (Math.Sqrt(distance2 * distance2 - distance1 * distance1) > _tipRadius + r.TipHeight
                        || Math.Sqrt(distance2 * distance2 - distance1 * distance1) < _rootRadius + r.RootHeight + (_toothDepth + r.ToothDepth) / 2)
                    {
                        return false;
                    }
                    //Their teeth relationship
                    if (Math.Abs(_module - r.Module) > 0.01)
                    { return false; }
                }
                else { return false; }
                return true;
            }
            protected override void ConductMoveAndUpdateParam(Movement move)
            {
                base.ConductMoveAndUpdateParam(move);
            }
        }

    }
}
