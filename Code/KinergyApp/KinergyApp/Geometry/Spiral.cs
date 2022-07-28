﻿using System;
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
using KinergyUtilities;

namespace Kinergy
{
    namespace Geom
    {
        public class Spiral:Component
        {
            private double roundNum = 0;
            private double innerRadius = 0, outerRadius = 0;
            private double thicknessX = 0, thicknessY = 0;  // thicknexxX is the spring strip thickness (t), thicknessY is the coil bandwidth (b)
            private Point3d centerPoint = Point3d.Unset;
            private Vector3d direction = Vector3d.Unset;
            private Curve spiralCurve = null;
            private Brep spiralBrep = null;
            private int pointsPerRound = 20;
            private double angleLoaded;
            private double velocity = 0;
            private int FrameCounter = 0;
            private Point3d startPos;
            private double maxRevolution;

            #region spiral spring constants
            private double min_strip_thickness = 0.8;
            private double inerRadius_constant = 2;
            private double coilBandwidth = 6;

            private bool isSpringDirCW;

            double revLevel = 0;
            double energyLevel = 0;
            double rotateAngle = Math.PI / 2;
            #endregion

            public int PointsPerRound { get => pointsPerRound;private set => pointsPerRound = value; }
            public Point3d CenterPoint { get => centerPoint;private set => centerPoint = value; }
            public Vector3d Direction { get => direction;private  set => direction = value; }
            public double ThicknessX { get => thicknessX;private  set => thicknessX = value; }
            public double ThicknessY { get => thicknessY;private  set => thicknessY = value; }
            public Point3d StartPos { get => startPos; set => startPos = value; }
            public double MaxRevolution { get => maxRevolution;private set => maxRevolution = value; }

            /// <summary> Default constructor without any input parameter </summary>
            /// <returns> Returns empty instance</returns>


            /// <summary> Constructor with center point and direction given </summary>
            /// <returns> Returns instance with spiral brep generated</returns>
            public Spiral(Point3d Center, Vector3d Direction, double R , double r, int RoundNum , double ThicknessX = 1,double ThicknessY = 1, double initialAngle=0, bool isSpringCW=true)
            {
                angleLoaded = initialAngle;
                center = new Point3d(Center);
                centerPoint =new Point3d( Center);
                direction = Direction;
                if (R > 0)
                { outerRadius = R; }
                if (r > 0)
                { innerRadius = r; }
                if (ThicknessX > 0)
                { thicknessX = ThicknessX; }
                if (ThicknessY > 0)
                { thicknessY = ThicknessY; }
                if (RoundNum > 0)
                { roundNum = RoundNum; }
                LoadSpiral(initialAngle);
                isSpringDirCW = isSpringCW;

            }
            /// <summary>
            /// construct the spiral
            /// </summary>
            /// <param name="startPt">the start point of the spiral spring for extrusion</param>
            /// <param name="Direction">the normal direction of the spring</param>
            /// <param name="Center">the spring center</param>
            /// <param name="R">the outer radius of the spring</param>
            /// <param name="maxD">the level of the revolution</param>
            /// <param name="energy">the level of the energy</param>
            /// <param name="initialAngle">the starting deflection angle</param>
            public Spiral(Point3d startPt, Vector3d Direction,Point3d Center,  double R, bool isSpringCW,  int maxD, int energy=5, double initialAngle = 0)
            {
                // maxD: 1 - 10
                // energy: 1 - 10

                startPos = startPt;
                angleLoaded = initialAngle;
                center = new Point3d(Center);
                centerPoint = new Point3d(Center);
                direction = Direction;
                outerRadius = R;
                innerRadius = inerRadius_constant;

                //First calculate n with dis
                revLevel = maxD / 10.0;
                thicknessX = min_strip_thickness;
                thicknessY = coilBandwidth;
               
                //roundNum = 2 + maxDegree / (Math.PI) ;
                //double energy033 = Math.Pow((energy + 2)/2, 0.33);
                //thicknessX *= energy033;

                double revolutions = revLevel * ((Math.Pow(min_strip_thickness, 3) + Math.Pow((outerRadius - innerRadius) / 6, 3)) / 2);
                maxRevolution = revolutions;
                thicknessX = Math.Pow(revolutions, 1.0/3.0);
                energyLevel = energy / 10.0;
                double pitch = energyLevel * (outerRadius - innerRadius) / 2;
                roundNum = (outerRadius - innerRadius) / (pitch + thicknessX);
                isSpringDirCW = isSpringCW;

                LoadSpiral(initialAngle);
            }

            public Spiral(Brep selectedBody, Point3d startPt, Vector3d Direction, Point3d Center, double R, bool isSpringCW, int maxD, bool non_instant, int energy = 5, double initialAngle = 0)
            {
                // maxD: 1 - 10
                // energy: 1 - 10

                startPos = startPt;
                angleLoaded = initialAngle;
                center = new Point3d(Center);
                centerPoint = new Point3d(Center);
                direction = Direction;
                outerRadius = R;
                innerRadius = inerRadius_constant;

                revLevel = maxD / 10.0;
                energyLevel = energy / 10.0;

                isSpringDirCW = isSpringCW;

                ////First calculate n with dis
                //revLevel = maxD / 10.0;
                //thicknessX = min_strip_thickness;
                //thicknessY = coilBandwidth;



                //double revolutions = revLevel * ((Math.Pow(min_strip_thickness, 3) + Math.Pow((outerRadius - innerRadius) / 6, 3)) / 2);
                //maxRevolution = revolutions;
                //thicknessX = Math.Pow(revolutions, 1.0 / 3.0);
                //energyLevel = energy / 10.0;
                //double pitch = energyLevel * (outerRadius - innerRadius) / 2;
                //roundNum = (outerRadius - innerRadius) / (pitch + thicknessX);
                //isSpringDirCW = isSpringCW;

                #region compute the spring parameters

                thicknessX = min_strip_thickness;
                thicknessY = coilBandwidth;

                double revolutions = revLevel * ((Math.Pow(min_strip_thickness, 3) + Math.Pow((outerRadius - innerRadius) / 6, 3)) / 2);
                //thicknessX = Math.Pow(1 / revolutions, 1.0 / 3.0);

                thicknessX = (2 - revLevel) * min_strip_thickness;

                double pitch = energyLevel * (outerRadius - innerRadius) / 3;
                roundNum = (outerRadius - innerRadius) / (pitch + thicknessX);

                #endregion

                angleLoaded += initialAngle;
                Point3d startCoilPos = GenerateSpiralCurve();

                #region generate the spiral and the supporting column for non-instant kinetic units

                Plane basePlane = new Plane(centerPoint, direction);
                Vector3d X = basePlane.XAxis;

                Plane recPlane = new Plane(centerPoint + X * outerRadius, X, direction);
                Rectangle3d outline = new Rectangle3d(recPlane, new Interval(-thicknessX, 0), new Interval(0, thicknessY));
                outline.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));
                Brep b = Brep.CreateFromSweep(base.BaseCurve, outline.ToNurbsCurve(), true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                b = b.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

                b.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == b.SolidOrientation)
                    b.Flip();

                b.Transform(Transform.Rotation(rotateAngle, direction, centerPoint));

                double suppColSideLen = 3.2;
                //Point3d conStart = startCoilPos + thicknessY * direction / direction.Length;
                //Point3d conStart = startCoilPos/* + (suppColSideLen / 2) * direction / direction.Length*/;
                //Point3d conEnd = -(centerPoint.DistanceTo(startPos) + thicknessY) * direction + conStart;

                //conStart.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));

                X.Transform(Transform.Rotation(rotateAngle, direction, centerPoint));

                Point3d conStartFinal = centerPoint + X * (outerRadius - thicknessX / 2) + thicknessY * direction;
                Plane supPlane = new Plane(conStartFinal, direction);
                Rectangle3d outlineSupport = new Rectangle3d(supPlane, new Interval(-suppColSideLen / 2, suppColSideLen / 2), new Interval(-suppColSideLen, 0));
                outlineSupport.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));

                outlineSupport.Transform(Transform.Rotation(rotateAngle, direction, conStartFinal));

                conStartFinal.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));

                Curve crossLineCrv = new Line(conStartFinal - direction * int.MaxValue, conStartFinal + direction * int.MaxValue).ToNurbsCurve();
                Curve[] crvs;
                Point3d[] pts;
                Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, selectedBody, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out crvs, out pts);
                Point3d conEndFinal = new Point3d();

                if ((pts[0] - conStartFinal) / pts[0].DistanceTo(conStartFinal) == direction)
                {
                    conEndFinal = pts[1] + direction * 0.5;
                }
                else
                {
                    conEndFinal = pts[0] + direction * 0.5;
                }

                Curve conPath = new Line(conStartFinal, conEndFinal).ToNurbsCurve();

                Brep connectorBrep = Brep.CreateFromSweep(conPath, outlineSupport.ToNurbsCurve(), true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                connectorBrep = connectorBrep.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                //Brep connectorBrep = Brep.CreatePipe(conPath, thicknessX, false, PipeCapMode.Flat, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];

                connectorBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == connectorBrep.SolidOrientation)
                    connectorBrep.Flip();

                Brep entireSpringModel = Brep.CreateBooleanUnion(new List<Brep> { b, connectorBrep }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                entireSpringModel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == entireSpringModel.SolidOrientation)
                    entireSpringModel.Flip();

                //entireSpringModel.Transform(Transform.Rotation(rotateAngle, direction, centerPoint));
                base.Model = entireSpringModel;

                #endregion
            }


            //public Spiral(Point3d startPt, Vector3d Direction, Point3d Center, double R, bool isSpringCW, int maxD, bool non_instant, int energy = 5, double initialAngle = 0)
            //{
            //    // maxD: 1 - 10
            //    // energy: 1 - 10

            //    startPos = startPt;
            //    angleLoaded = initialAngle;
            //    center = new Point3d(Center);
            //    centerPoint = new Point3d(Center);
            //    direction = Direction;
            //    outerRadius = R;
            //    innerRadius = inerRadius_constant;

            //    //First calculate n with dis
            //    double revLevel = maxD / 10.0;
            //    thicknessX = min_strip_thickness;
            //    thicknessY = coilBandwidth;

            //    //roundNum = 2 + maxDegree / (Math.PI) ;
            //    //double energy033 = Math.Pow((energy + 2)/2, 0.33);
            //    //thicknessX *= energy033;

            //    double revolutions = revLevel * ((Math.Pow(min_strip_thickness, 3) + Math.Pow((outerRadius - innerRadius) / 6, 3)) / 2);
            //    maxRevolution = revolutions;
            //    thicknessX = Math.Pow(revolutions, 1.0 / 3.0);
            //    double energyLevel = energy / 10.0;
            //    double pitch = energyLevel * (outerRadius - innerRadius) / 2;
            //    roundNum = (outerRadius - innerRadius) / (pitch + thicknessX);
            //    isSpringDirCW = isSpringCW;

            //    angleLoaded += initialAngle;
            //    Point3d startCoilPos = GenerateSpiralCurve();


            //    #region generate the spiral and the supporting column for non-instant kinetic units

            //    Plane basePlane = new Plane(centerPoint, direction);
            //    Vector3d X = basePlane.XAxis;
            //    Plane recPlane = new Plane(centerPoint + X * outerRadius, X, direction);
            //    Rectangle3d outline = new Rectangle3d(recPlane, new Interval(-thicknessX, 0), new Interval(0, thicknessY));
            //    outline.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));
            //    Brep b = Brep.CreateFromSweep(base.BaseCurve, outline.ToNurbsCurve(), true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            //    b = b.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

            //    b.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //    if (BrepSolidOrientation.Inward == b.SolidOrientation)
            //        b.Flip();

            //    double suppColSideLen = 3.2;
            //    //Point3d conStart = startCoilPos + thicknessY * direction / direction.Length;
            //    Point3d conStart = startCoilPos + (suppColSideLen/2) * direction / direction.Length;
            //    Point3d conEnd = -(centerPoint.DistanceTo(startPos) + thicknessY) * direction / direction.Length + conStart;
            //    Curve conPath = new Line(conStart, conEnd).ToNurbsCurve();

            //    Plane supPlane = new Plane(centerPoint + X * (outerRadius - thicknessX / 2) + thicknessY * direction / direction.Length, direction);
            //    Rectangle3d outlineSupport = new Rectangle3d(supPlane, new Interval(-suppColSideLen / 2, suppColSideLen / 2), new Interval(-suppColSideLen, 0));
            //    outlineSupport.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));
            //    Brep connectorBrep = Brep.CreateFromSweep(conPath, outlineSupport.ToNurbsCurve(), true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            //    connectorBrep = connectorBrep.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            //    //Brep connectorBrep = Brep.CreatePipe(conPath, thicknessX, false, PipeCapMode.Flat, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];

            //    connectorBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //    if (BrepSolidOrientation.Inward == connectorBrep.SolidOrientation)
            //        connectorBrep.Flip();

            //    Brep entireSpringModel = Brep.CreateBooleanUnion(new List<Brep> { b, connectorBrep }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            //    entireSpringModel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //    if (BrepSolidOrientation.Inward == entireSpringModel.SolidOrientation)
            //        entireSpringModel.Flip();

            //    base.Model = entireSpringModel;

            //    #endregion
            //}
            public void AdjustParam(Vector3d mainAxis, List<GearParameter> gear_info, Brep body, int eeMovingDirectionSelection, int e, int D, bool isSpringCW, ref List<Point3d>lockPos, ref bool spiralLockNorm, ref Vector3d spiralLockDir, Point3d eePos = new Point3d())
            {
                if(gear_info == null)
                {
                    revLevel = D / 10.0;
                    thicknessX = min_strip_thickness;
                    thicknessY = coilBandwidth;

                    double revolutions = revLevel * ((Math.Pow(min_strip_thickness, 3) + Math.Pow((outerRadius - innerRadius) / 6, 3)) / 2);
                    energyLevel = e / 10.0;
                    //thicknessX = Math.Pow(1 / revolutions, 1.0 / 3.0);

                    thicknessX = (2 - revLevel) * min_strip_thickness;

                    double pitch = energyLevel * (outerRadius - innerRadius) / 3;
                    roundNum = (outerRadius - innerRadius) / (pitch + thicknessX);
                    isSpringDirCW = isSpringCW;

                    LoadSpiral(0);
                }
                else
                {
                    #region Step 1: find the spring position and orientation

                    Point3d springCen = new Point3d();

                    Point3d firstGearCen = gear_info.ElementAt(0).center;
                    Point3d secondGearCen = gear_info.ElementAt(1).center;
                    Vector3d axelDir = firstGearCen - secondGearCen;
                    axelDir.Unitize();

                    Curve crossLineCrv = new Line(firstGearCen - axelDir * int.MaxValue, firstGearCen + axelDir * int.MaxValue).ToNurbsCurve();
                    Curve[] crvs;
                    Point3d[] pts;
                    Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, body, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out crvs, out pts);

                    Point3d ptEnd = new Point3d();
                    Point3d ptStart = new Point3d();
                    if ((pts[0] - pts[1]) / (pts[0].DistanceTo(pts[1])) == axelDir)
                    {
                        ptEnd = pts[0] - axelDir * 1;
                        ptStart = pts[1] + axelDir * 1;
                    }
                    else
                    {
                        ptEnd = pts[1] - axelDir * 1;
                        ptStart = pts[0] + axelDir * 1;
                    }

                    // Project the end-effector's position to the first shaft 
                    Vector3d sDir = new Vector3d();
                    if (eePos != new Point3d())
                    {
                        double t = -1;
                        crossLineCrv.ClosestPoint(eePos, out t);
                        Point3d eePos_proj = crossLineCrv.PointAt(t);
                        sDir = secondGearCen - eePos_proj;
                        sDir.Unitize();
                        eePos_proj = eePos_proj + sDir * 7;

                        //myDoc.Objects.AddPoint(eePos_proj);
                        //myDoc.Views.Redraw();

                        //myDoc.Objects.AddPoint(secondGearCen);
                        //myDoc.Views.Redraw();

                        if (secondGearCen.DistanceTo(eePos_proj) > 13)
                        {
                            springCen = eePos_proj + sDir * 0.6;
                        }
                        else
                        {
                            return;
                        }
                    }
                    else
                    {
                        springCen = (ptEnd + firstGearCen) / 2;
                        sDir = firstGearCen - ptEnd;
                        sDir.Unitize();
                        springCen = springCen - sDir * 3;
                    }

                    Point3d axisStart = ptEnd;
                    Vector3d springDir = ptStart - ptEnd;
                    springDir.Unitize();

                    bool isCW = true;
                    int predDir = 1;

                    // determine the spring rotation direction based on the direction of the end rack
                    //if ((gear_info.Count - 1) % 2 == 1)
                    //{
                    //    if (dir != 1 && dir != 2)
                    //    {
                    //        // perpendicular down
                    //        isCW = false;
                    //    }
                    //    else
                    //    {
                    //        isCW = true;
                    //    }
                    //}
                    //else
                    //{
                    //    if (dir != 1 && dir != 2)
                    //    {
                    //        // perpendicular down
                    //        isCW = true;
                    //    }
                    //    else
                    //    {
                    //        isCW = false;
                    //    }
                    //}
                    isCW = isSpringCW;
                    isSpringDirCW = isSpringCW;

                    //Spiral spiralSpring = new Spiral(body, axisStart, springDir, springCen, (gear_info.ElementAt(1).radius + gear_info.ElementAt(0).radius) * 0.7, isCW, displacement, true, energyLevel, 0);

                    #endregion

                    #region Step 2: generate the lock position

                    spiralLockNorm = isCW;
                    spiralLockDir = sDir;

                    Point3d lockCen = springCen + sDir * (3.5 + 6 + 1.8);

                    // find the vector that is orthogonal to both sDir and mainAxis
                    Vector3d lockV = Vector3d.CrossProduct(sDir, mainAxis);
                    lockV.Unitize();
                    Curve lockCrossLineCrv = new Line(lockCen - lockV * int.MaxValue, lockCen + lockV * int.MaxValue).ToNurbsCurve();
                    Curve[] lockCrvs;
                    Point3d[] lockPts;
                    Rhino.Geometry.Intersect.Intersection.CurveBrep(lockCrossLineCrv, body, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out lockCrvs, out lockPts);

                    lockPos.Clear();
                    lockPos.Add(lockPts[0]);
                    lockPos.Add(lockPts[1]);
                    lockPos.Add(lockCen);

                    #endregion


                    #region re-generate the parameters for the new spiral spring

                    startPos = axisStart;
                    angleLoaded = 0;
                    center = new Point3d(springCen);
                    centerPoint = new Point3d(springCen);
                    direction = springDir;
                    outerRadius = (gear_info.ElementAt(1).radius + gear_info.ElementAt(0).radius) * 0.7;
                    innerRadius = inerRadius_constant;

                    revLevel = D / 10.0;
                    energyLevel = e / 10.0;

                    #region compute the spring parameters

                    thicknessX = min_strip_thickness;
                    thicknessY = coilBandwidth;

                    double revolutions = revLevel * ((Math.Pow(min_strip_thickness, 3) + Math.Pow((outerRadius - innerRadius) / 6, 3)) / 2);
                    //thicknessX = Math.Pow(1 / revolutions, 1.0 / 3.0);

                    thicknessX = (2 - revLevel) * min_strip_thickness*1.2;

                    double pitch = energyLevel * (outerRadius - innerRadius) / 3 * 1.5;
                    roundNum = (outerRadius - innerRadius) / (pitch + thicknessX);

                    #endregion

                    #endregion

                    LoadSpiral(0);
                }
            }
            /// <summary> Constructor with parameter but no center point given </summary>
            /// <returns> Returns instance with gear brep generated</returns>

            private void FixParameter()
            {
                //if (outerRadius == 0)
                //{ outerRadius = 30; }
                //if (innerRadius == 0)
                //{ outerRadius = 5; }
                //if (thicknessX == 0)
                //{ thicknessX = min_strip_thickness; }
                //if (thicknessY == 0)
                //{ thicknessX = coilBandwidth; }
                //if (outerRadius < innerRadius)
                //{
                //    double t = innerRadius;
                //    innerRadius = outerRadius;
                //    outerRadius = t;
                //}
                //if (outerRadius == innerRadius)
                //{
                //    innerRadius = outerRadius / 3;
                //}
                //if (thicknessX > (outerRadius - innerRadius) / roundNum)
                //{
                //    thicknessX = (outerRadius - innerRadius) / roundNum / 2;
                //}
                //if(angleLoaded>roundNum*Math.PI*2*(outerRadius-innerRadius)/(2*outerRadius+4*innerRadius)*0.5)
                //{
                //    angleLoaded = roundNum * Math.PI * 2 * (outerRadius - innerRadius) / (2 * outerRadius + 4 * innerRadius)*0.5;
                //}
            }
            private Point3d GenerateSpiralCurve()
            {
                FixParameter();
                int numPoints =(int) Math.Round( pointsPerRound * roundNum )+ 1;
                double PI = Math.PI;
                Plane basePlane = new Plane(centerPoint, direction);
                Vector3d X = basePlane.XAxis;
                Vector3d Y = basePlane.YAxis;
                List<Point3d> pts = new List<Point3d>();
                //double initialAngle = -angleLoaded;
                //double initialAngle = -2*angleLoaded;
                double totalAngle = 2 * PI*roundNum + 2 * angleLoaded;

                
                Point3d spiralStartPt = centerPoint + X * outerRadius;
                Curve s = null;
                if (isSpringDirCW)
                    // the generated spiral spring is CW, which means the turnNum is negative
                    s = NurbsCurve.CreateSpiral(centerPoint, direction, spiralStartPt, 0, -roundNum, outerRadius, 0.5);
                else
                    // the generated spiral spring is CCW, which means the turnNum is positive
                    s = NurbsCurve.CreateSpiral(centerPoint, direction, spiralStartPt, 0, roundNum, outerRadius, 0.5);

                #region old spiral generation
                ////Did some calculus here to use a quadratic polynomial to compute the loaded spiral curve.
                //double alpha = 3*angleLoaded * (outerRadius + innerRadius) / Math.Pow(totalAngle, 3);
                //double beta = (2 * PI * roundNum * (outerRadius - innerRadius) - 2 * angleLoaded * outerRadius - 4 * angleLoaded * innerRadius) / Math.Pow(totalAngle, 2);

                //for (int i = 0; i < numPoints; i++)
                //{
                //    double angle = initialAngle + i * totalAngle / (numPoints - 1);
                //    double radius = alpha *Math.Pow(i * totalAngle / (numPoints - 1), 2) + beta * (i * totalAngle / (numPoints - 1)) + innerRadius;
                //    //Point3d newPt = CP + (X * Math.Cos(angle) + Y * Math.Sin(angle)) * radius;
                //    pts.Add(centerPoint + (X * Math.Cos(angle) + Y * Math.Sin(angle)) *radius);
                //    //pts.Add(newPt);
                //}
                //Curve s = Rhino.Geometry.Curve.CreateInterpolatedCurve(pts, 3);
                #endregion

                /*string body = string.Format("Spiral center is now at {0} , {1} , {2} ", center.X, center.Y, center.Z);
                Rhino.RhinoApp.WriteLine(body);*/
                base.BaseCurve = s;
                spiralCurve = s;

                return spiralStartPt;

                //if (pts == null || pts.Count == 0)
                //    return new Point3d();
                //else
                //    return pts.ElementAt(pts.Count-1);
            }
            private void GenerateSpiralBrep(Point3d springStartPos)
            {
               
                //Point3d CP = new Point3d(centerPoint);
                Plane basePlane = new Plane(centerPoint, direction);
                Vector3d X = basePlane.XAxis;
                //X.Transform(Transform.Rotation(Math.PI / 2, direction, centerPoint));


                Plane recPlane = new Plane(centerPoint + X * outerRadius, X, direction);
                Rectangle3d outline = new Rectangle3d(recPlane, new Interval(-thicknessX, 0), new Interval(0, thicknessY));
                outline.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));
                Brep b = Brep.CreateFromSweep(base.BaseCurve, outline.ToNurbsCurve(), true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                b = b.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

                b.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == b.SolidOrientation)
                    b.Flip();

                b.Transform(Transform.Rotation(rotateAngle, direction, centerPoint));

                double suppColSideLen = 3.2;

                X.Transform(Transform.Rotation(rotateAngle, direction, centerPoint));
                springStartPos.Transform(Transform.Rotation(rotateAngle, direction, centerPoint));

                Point3d conStart = springStartPos + (suppColSideLen / 2) * direction / direction.Length;

                //Point3d conStart = centerPoint + X * (outerRadius - thicknessX / 2) + thicknessY * direction;

                Point3d conEnd = -(centerPoint.DistanceTo(startPos) + thicknessY) * direction / direction.Length + conStart;
                Curve conPath = new Line(conStart, conEnd).ToNurbsCurve();

                Plane supPlane = new Plane(centerPoint + X * (outerRadius - thicknessX / 2) + thicknessY * direction / direction.Length, direction);
                Rectangle3d outlineSupport = new Rectangle3d(supPlane, new Interval(-suppColSideLen / 2, suppColSideLen / 2), new Interval(-suppColSideLen, 0));
                outlineSupport.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));

                outlineSupport.Transform(Transform.Rotation(rotateAngle, direction, centerPoint + X * (outerRadius - thicknessX / 2) + thicknessY * direction / direction.Length));

                Brep connectorBrep = Brep.CreateFromSweep(conPath, outlineSupport.ToNurbsCurve(), true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                connectorBrep = connectorBrep.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                //Brep connectorBrep = Brep.CreatePipe(conPath, thicknessX, false, PipeCapMode.Flat, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];

                connectorBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == connectorBrep.SolidOrientation)
                    connectorBrep.Flip();

                Brep entireSpringModel = Brep.CreateBooleanUnion(new List<Brep> { b, connectorBrep }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                entireSpringModel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == entireSpringModel.SolidOrientation)
                    entireSpringModel.Flip();

                //entireSpringModel.Transform(Transform.Rotation(rotateAngle, direction, centerPoint));
                base.Model = entireSpringModel;

            }
            public void LoadSpiral(double degree)
            {
                angleLoaded += degree;
                Generate();
            }
            public override void Generate()
            {
                Point3d startPos = GenerateSpiralCurve();
                GenerateSpiralBrep(startPos);
            }
            
            //public void SetPosition(Point3d C, Vector3d Direction)
            //{
            //    center = C;
            //    centerPoint = C;
            //    direction = Direction;
            //    GenerateSpiralBrep();
            //}
            //public void SetParameter(double R = 0, double r = 0, double ThicknessX = 0, double ThicknessY = 0, int RoundNum = 0)
            //{
            //    if (R > 0)
            //    { outerRadius = R; }
            //    if (r > 0)
            //    { innerRadius = r; }
            //    if (ThicknessX > 0)
            //    { thicknessX = ThicknessX; }
            //    if (ThicknessY > 0)
            //    { thicknessY = ThicknessY; }
            //    if (RoundNum > 0)
            //    { roundNum = RoundNum; }
            //    GenerateSpiralBrep();
            //}
            
            public override bool Move(Movement move)
            {
                //TODO Do the mobility check
                if (move.Type == 4)
                {
                    DfsMark = true;
                    bool CanIMove = true;
                    //Then move all other constraints to know if this movement can be operated
                    foreach (Relationship.Relationship c in constraints)
                    {
                        if (c.TheOtherEntity(this).DfsMark == true)//Skip the already visited component to avoid cycle triggering.
                        { continue; }
                        if (c.Move(move) == false)
                        {
                            CanIMove = false;
                            string body = string.Format("A movement on {0} typed {1} with value {2} is stopped by {3} to {4}", this.GetType(), move.Type, move.MovementValue,c.GetType(),c.TheOtherEntity(this).GetType());
                            Rhino.RhinoApp.WriteLine(body);
                            break;
                        }
                    }
                    if (CanIMove)
                    {
                        this.ConductMoveAndUpdateParam(move);
                    }
                    else
                    {
                        //string body = string.Format("A movement on {0} typed {1} with value {2} is stopped", this.GetType(), move.Type, move.MovementValue);
                        //Rhino.RhinoApp.WriteLine(body);
                        //throw new Exception("failed to move!");
                    }
                    DfsMark = false;
                    return CanIMove;
                }
                else { return base.Move(move); }
                
            }
            public Movement Activate(double interval, double damp = 0.3)
            {
                velocity *= Math.Pow(1 - damp, interval / 100);
                velocity += -angleLoaded*0.1;//TODO adjust these parameters
                //velocity *= Math.Pow(1 - damp, interval / 100);
                Movement m = new Movement(this, 4, velocity * interval / 1000);
                m.Activate();
                if (Math.Abs(velocity) < 0.5 && Math.Abs(angleLoaded) < 0.1)
                {
                    m.SetConverge();
                }
                return m;
            }
            protected override void ConductMoveAndUpdateParam(Movement move)
            {
                //TODO Do the rotating
                if (move.Type == 4)
                {
                    //FrameCounter++;
                    angleLoaded += move.MovementValue;
                    //Rhino.RhinoApp.WriteLine("Spiral Regenerated");
                    //if(FrameCounter%5==0)
                        //Generate(); //This might not be the ideal way of simulating since its time-consuming
                    //string body = string.Format("Spiral center is now at {0} , {1} , {2} ", center.X, center.Y, center.Z);
                    //Rhino.RhinoApp.WriteLine(body);
                }
                /*else if (move.Type == 1||move.Type==2)
                {
                    Offset = Transform.Multiply(Offset, move.Trans);
                }*/
            }
            public override int GetContactPosition(Entity other)
            {
                //Point3d inner = spiralCurve.PointAtEnd;
                //Point3d outer = spiralCurve.PointAtStart;
                //double d1 = other.Model.ClosestPoint(inner).DistanceTo(inner);
                //double d2 = other.Model.ClosestPoint(outer).DistanceTo(outer);
                //if (d1 < d2)
                //    return 2;
                //else
                //    return 1;
                return 2;
            }
        }
    }
}
