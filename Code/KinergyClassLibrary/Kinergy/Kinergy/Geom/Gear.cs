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
            private int z = 0;//Total number of teeth
            private double pitch = 0;//pitch ,distance between corresponding points on adjacent teeth. p=m*Pi;
            private double module = 0;//module of teeth size, 
            private double faceWidth = 0;
            private double pressureAngle = 20;
            private double coneAngle = 0;
            private double clearance = 0.167;
            private double involSample = 10;
            private double tipRadius = 0;
            private double rootRadius = 0;
            private double toothDepth = 0;

            private Point3d centerPoint = Point3d.Unset;
            private Vector3d direction = Vector3d.Unset;
            private Boolean twoLayer = false;

            public int Z { get => z; protected set => z = value; }
            public double Pitch { get => pitch; protected set => pitch = value; }
            public double Module { get => module; protected set => module = value; }
            public double FaceWidth { get => faceWidth; protected set => faceWidth = value; }
            public double TipRadius { get => tipRadius; protected set => tipRadius = value; }
            public double RootRadius { get => rootRadius; protected set => rootRadius = value; }
            public double ToothDepth { get => toothDepth; protected set => toothDepth = value; }
            public Point3d CenterPoint { get => centerPoint; protected set => centerPoint = value; }
            public Vector3d Direction { get => direction; protected set => direction = value; }

            //private Brep surface = null;

            /// <summary> Constructor with center point and direction given </summary>
            /// <returns> Returns instance with gear brep generated</returns>
            /// <param name="CP">Center Point of gear</param>
            /// <param name="module">Module of gear teeth, this parameter determines how big teeth are</param>            
            /// <summary> Constructor with parameter but no center point given </summary>
            /// <returns> Returns instance with gear brep generated</returns>            



            public Gear(Point3d center_point, Vector3d gear_direction, int teethNum, double mod, double pressure_angle = 20, double Thickness = 1.6, Boolean two_layer = false)
            {
                centerPoint = center_point;
                direction = gear_direction;
                z = teethNum;
                module = mod;
                pressureAngle = pressure_angle;
                faceWidth = Thickness;
                two_layer = false;
                GenerateGear();
            }

            public Gear(int teethNum, double mod, double pressure_angle = 20, double Thickness = 1.6, Boolean two_layer = false)
            {
                z = teethNum;
                module = mod;
                pressureAngle = pressure_angle;
                faceWidth = Thickness;
                two_layer = false;
                GenerateGear();
            }

            public Gear(int teethNum, double mod, double cone_angle, double pressure_angle = 20, double Thickness = 1.6, Boolean two_layer = false)
            {
                z = teethNum;
                module = mod;
                pressureAngle = pressure_angle;
                coneAngle = cone_angle;
                faceWidth = Thickness;
                GenerateGear();
            }

            protected List<Point3d> GenerateInvolPoints(double baseDiameter, double startAngle, double endAngle, double angleModule, double samples)
            {
                List<Point3d> involPoints = new List<Point3d>();
                double step = (startAngle - angleModule - endAngle) / samples;
                double baseRadius = baseDiameter / 2;

                for (int i = 0; i <= samples; i++)
                {
                    double position = angleModule + i * step;
                    double height = Math.Sqrt(Math.Pow(position, 2) * Math.Pow(baseRadius, 2) + Math.Pow(baseRadius, 2));
                    double heightAngle = startAngle - position + Math.Atan(position);

                    Point3d point = new Point3d(height * Math.Cos(heightAngle), height * Math.Sin(heightAngle), 0);
                    RhinoApp.Write("involPoint" + point);
                    involPoints.Add(point);
                }

                return involPoints;
            }


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

            protected override void GenerateBaseCurve()
            {
                pressureAngle = (Math.PI / 180) * pressureAngle;

                RhinoDoc mydoc = RhinoDoc.ActiveDoc;
                double pitchDiameter = module * z;
                double baseDiameter = pitchDiameter * Math.Cos(pressureAngle);
                double addendum = module;
                double dedendum = (1 + clearance) * module;
                double outDiameter = pitchDiameter + 2 * addendum;
                double rootDiameter = pitchDiameter - 2 * dedendum;
                double chordalThickness = pitchDiameter * Math.Sin((Math.PI / 2) / z);

                tipRadius = outDiameter / 2;
                rootRadius = rootDiameter / 2;
                toothDepth = tipRadius - rootRadius;
                pitch = Math.PI * module;

                /*
                RhinoApp.WriteLine("pitchDiameter" + pitchDiameter);
                RhinoApp.WriteLine("baseDiameter" + baseDiameter);
                RhinoApp.WriteLine("outDiameter" + outDiameter);
                RhinoApp.WriteLine("rootDiameter" + rootDiameter);
                RhinoApp.WriteLine("chordalThickness" + chordalThickness);
                */

                double invol_start_angle = (Math.PI / 2 + Math.Asin(chordalThickness / pitchDiameter)
                    - pressureAngle
                    + Math.Sqrt(Math.Pow((pitchDiameter / baseDiameter), 2) - 1));
                double invol_end_angle = (invol_start_angle - Math.Sqrt(Math.Pow((outDiameter / baseDiameter), 2) - 1));

                /*
                RhinoApp.WriteLine("invol_start_angle" + invol_start_angle);
                RhinoApp.WriteLine("invol_end_angle" + invol_start_angle);
                */

                double invol_angle_module = 0;

                if (rootDiameter > baseDiameter)
                {
                    invol_angle_module = Math.Sqrt(Math.Pow((rootDiameter / baseDiameter), 2) - 1);
                    RhinoApp.WriteLine("invol_angle_module" + invol_angle_module);
                }

                List<Point3d> involPts = GenerateInvolPoints(baseDiameter, invol_start_angle, invol_end_angle, invol_angle_module, involSample);

                List<Curve> toothCrv = new List<Curve>();
                Curve involCrv1 = Curve.CreateInterpolatedCurve(involPts, 3, CurveKnotStyle.Chord);
                Transform mirror = Transform.Mirror(new Point3d(0, 0, 0), new Vector3d(1, 0, 0));
                Curve involCrv2 = involCrv1.DuplicateCurve();
                involCrv2.Transform(mirror);
                Point3d ptArc = tiltPointAroundCircle(new Point3d(0, outDiameter / 2, 0), coneAngle / 2, pitchDiameter);
                //RhinoApp.WriteLine("point on up arc" + ptArc);
                Arc topArc = new Arc(involCrv1.PointAtEnd, ptArc, involCrv2.PointAtEnd);
                Curve arcCrv = new ArcCurve(topArc);

                //dedendum
                if (rootDiameter < baseDiameter)
                {
                    RhinoApp.WriteLine("rootDiameter < baseDiameter");
                    Point3d pt = new Point3d(rootDiameter / 2 * Math.Cos(invol_start_angle),
                        rootDiameter / 2 * Math.Sin(invol_start_angle), 0);

                    Curve dedCrv1 = new Line(involCrv1.PointAtStart, tiltPointAroundCircle(pt, coneAngle / 2, pitchDiameter)).ToNurbsCurve();
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
                Curve tooth = Curve.JoinCurves(toothCrv, 0.00000001, false)[0];

                //tooth bottom
                double angle = 2 * Math.PI / z;
                Point3d startPt = tooth.PointAtStart;
                Point3d endPt = tooth.PointAtEnd;
                Point3d btEndPt = new Point3d(endPt.X * Math.Cos(angle) - endPt.Y * Math.Sin(angle),
                    endPt.Y * Math.Cos(angle) + endPt.X * Math.Sin(angle), endPt.Z);
                Point3d bottomPt = new Point3d(-Math.Sin(angle / 2) * rootDiameter / 2, Math.Cos(angle / 2) * rootDiameter / 2, 0);
                bottomPt = tiltPointAroundCircle(bottomPt, coneAngle / 2, pitchDiameter);
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

                for (int i = 0; i < z; i++)
                {
                    Transform rotat = Transform.Rotation(-i * angle, new Vector3d(0, 0, 1), new Point3d(0, 0, 0));
                    Curve nextTooth = tooth.DuplicateCurve();
                    nextTooth.Transform(rotat);
                    gearTooth.Add(nextTooth);
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
                RhinoApp.WriteLine("extruding facewidth" + faceWidth);
                Extrusion extrudeCrv = Extrusion.Create(base.BaseCurve, faceWidth, true);

                base.Model = extrudeCrv.ToBrep();

                /*
                if (twoLayer == true) {

                }
                */

                if (centerPoint != Point3d.Unset)
                {
                    Vector3d centerDirection = new Vector3d(centerPoint);
                    Transform centerTrans = Transform.Translation(centerDirection);
                    base.Model.Transform(centerTrans);
                }

                if (direction != Vector3d.Unset)
                {
                    if(base.BaseCurve.ClosedCurveOrientation() == CurveOrientation.Clockwise)
                    {
                        Transform centerRotate = Transform.Rotation(new Vector3d(0, 0, 1), new Vector3d(0, -1, 0), centerPoint);
                        base.Model.Transform(centerRotate);
                    }
                    else
                    {
                        Transform centerRotate = Transform.Rotation(new Vector3d(0, 0, 1), new Vector3d(0, 1, 0), centerPoint);
                        base.Model.Transform(centerRotate);
                    }
                    
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
                centerPoint = CP;
                center = CP;
                direction = GearDirection;
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
                    Vector3d d1 = direction / direction.Length;
                    Vector3d d2 = g.Direction / g.Direction.Length;
                    if (Math.Abs(d1 * d2) < 0.99)
                    { return false; }
                    if (Math.Abs(new Vector3d(CenterPoint) * d1) - Math.Abs(new Vector3d(g.CenterPoint) * d2) > (faceWidth + g.FaceWidth) / 2)
                    { return false; }
                    //and the distance of centers have to be between R1+R2 and r1+r2+h(smaller)
                    Vector3d vCP1 = new Vector3d(CenterPoint);
                    Vector3d vCP2 = new Vector3d(g.CenterPoint);
                    vCP1 -= vCP1 * d1 * d1;
                    vCP2 -= vCP2 * d2 * d2;
                    double distance = (vCP1 - vCP2).Length;
                    if (distance > tipRadius + g.TipRadius || distance < rootRadius + g.RootRadius + (toothDepth + g.ToothDepth) / 2)
                    { return false; }
                    //Their teeth relationship
                    if (Math.Abs(module - g.Module) > 0.01)
                    { return false; }
                    //Actually the engagement of gears could be very complicated. Herer I only cover the basic parts due to my very limited knowledge in this field.
                }
                else if (obj.GetType() == typeof(Rack))
                {
                    Rack r = (Rack)obj;
                    //If the gear is engaged with a rack, their direction vector should be prependicular and their interval of z direction have to overlap
                    Vector3d d1 = direction / direction.Length;
                    Vector3d d2 = r.RackDirection / r.RackDirection.Length;
                    if (Math.Abs(d1 * d2) > 0.01)
                    { return false; }
                    double distance1 = new Plane(centerPoint, direction).DistanceTo(r.CenterPoint);
                    if (distance1 > (faceWidth + r.FaceWidth) / 2)
                    { return false; }
                    //and the distance should be between R1+R2 and r1+r2+h(smaller).
                    double distance2 = r.BackBone.DistanceTo(centerPoint, true);
                    if (Math.Sqrt(distance2 * distance2 - distance1 * distance1) > tipRadius + r.TipHeight
                        || Math.Sqrt(distance2 * distance2 - distance1 * distance1) < rootRadius + r.RootHeight + (toothDepth + r.ToothDepth) / 2)
                    {
                        return false;
                    }
                    //Their teeth relationship
                    if (Math.Abs(module - r.Module) > 0.01)
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
