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
            private double inerRadius_constant = 1.5;
            private double coilBandwidth = 7;

            private bool isSpringDirCW;

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
                double revLevel = maxD / 10.0;
                thicknessX = min_strip_thickness;
                thicknessY = coilBandwidth;
               
                //roundNum = 2 + maxDegree / (Math.PI) ;
                //double energy033 = Math.Pow((energy + 2)/2, 0.33);
                //thicknessX *= energy033;

                double revolutions = revLevel * ((Math.Pow(min_strip_thickness, 3) + Math.Pow((outerRadius - innerRadius) / 6, 3)) / 2);
                maxRevolution = revolutions;
                thicknessX = Math.Pow(revolutions, 1.0/3.0);
                double energyLevel = energy / 10.0;
                double pitch = energyLevel * (outerRadius - innerRadius) / 2;
                roundNum = (outerRadius - innerRadius) / (pitch + thicknessX);
                isSpringDirCW = isSpringCW;

                LoadSpiral(initialAngle);
            }

            public Spiral(Point3d startPt, Vector3d Direction, Point3d Center, double R, bool isSpringCW, int maxD, bool non_instant, int energy = 5, double initialAngle = 0)
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
                double revLevel = maxD / 10.0;
                thicknessX = min_strip_thickness;
                thicknessY = coilBandwidth;

                //roundNum = 2 + maxDegree / (Math.PI) ;
                //double energy033 = Math.Pow((energy + 2)/2, 0.33);
                //thicknessX *= energy033;

                double revolutions = revLevel * ((Math.Pow(min_strip_thickness, 3) + Math.Pow((outerRadius - innerRadius) / 6, 3)) / 2);
                maxRevolution = revolutions;
                thicknessX = Math.Pow(revolutions, 1.0 / 3.0);
                double energyLevel = energy / 10.0;
                double pitch = energyLevel * (outerRadius - innerRadius) / 2;
                roundNum = (outerRadius - innerRadius) / (pitch + thicknessX);
                isSpringDirCW = isSpringCW;

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

                double suppColSideLen = 3.2;
                //Point3d conStart = startCoilPos + thicknessY * direction / direction.Length;
                Point3d conStart = startCoilPos + (suppColSideLen/2) * direction / direction.Length;
                Point3d conEnd = -(centerPoint.DistanceTo(startPos) + thicknessY) * direction / direction.Length + conStart;
                Curve conPath = new Line(conStart, conEnd).ToNurbsCurve();

                Plane supPlane = new Plane(centerPoint + X * (outerRadius - thicknessX / 2) + thicknessY * direction / direction.Length, direction);
                Rectangle3d outlineSupport = new Rectangle3d(supPlane, new Interval(-suppColSideLen / 2, suppColSideLen / 2), new Interval(-suppColSideLen, 0));
                outlineSupport.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));
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

                base.Model = entireSpringModel;

                #endregion
            }
            public  void AdjustParam(int e, int D, bool isSpringCW)
            {
                double revLevel = D / 10.0;
                thicknessX = min_strip_thickness;
                thicknessY = coilBandwidth;

                //roundNum = 2 + maxDegree / (Math.PI);
                ////Then calculate thicknessY&X with energy
                ////double Len = roundNum * (outerRadius + innerRadius) * Math.PI;
                ////Maybe just simplify it by turning the 1-10 M to the power of 0.25, and 0.75, then expand Y&X respectively
                //double energy033 = Math.Pow((e + 2)/ 2, 0.33);
                //thicknessX *= energy033;

                double revolutions = revLevel * ((Math.Pow(min_strip_thickness, 3) + Math.Pow((outerRadius - innerRadius) / 6, 3)) / 2);
                thicknessX = Math.Pow(revolutions, 1.0 / 3.0);
                double energyLevel = e / 10.0;
                double pitch = energyLevel * (outerRadius - innerRadius) / 2;
                roundNum = (outerRadius - innerRadius) / (pitch + thicknessX);
                isSpringDirCW = isSpringCW;

                LoadSpiral(0);
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
                double initialAngle = -angleLoaded;
                double totalAngle = 2 * PI*roundNum + angleLoaded;

                
                Point3d spiralStartPt = centerPoint + X * outerRadius;
                Curve s = null;
                if (isSpringDirCW)
                    // the generated spiral spring is CW, which means the turnNum is negative
                    s = NurbsCurve.CreateSpiral(centerPoint, direction, spiralStartPt, 0, -roundNum, outerRadius, innerRadius);
                else
                    // the generated spiral spring is CCW, which means the turnNum is positive
                    s = NurbsCurve.CreateSpiral(centerPoint, direction, spiralStartPt, 0, roundNum, outerRadius, innerRadius);

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
                Plane recPlane = new Plane(centerPoint + X * outerRadius, X, direction);
                Rectangle3d outline = new Rectangle3d(recPlane, new Interval(-thicknessX, 0), new Interval(0, thicknessY));
                outline.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));
                Brep b = Brep.CreateFromSweep(base.BaseCurve, outline.ToNurbsCurve(), true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                b = b.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

                b.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == b.SolidOrientation)
                    b.Flip();

                double suppColSideLen = 6;
                Point3d conStart = springStartPos + thicknessY  * direction / direction.Length;
                Point3d conEnd = -(centerPoint.DistanceTo(startPos) + thicknessY) * direction / direction.Length + conStart;
                Curve conPath = new Line(conStart, conEnd).ToNurbsCurve();

                Plane supPlane = new Plane(centerPoint + X * (outerRadius - thicknessX / 2) + thicknessY * direction / direction.Length, direction);
                Rectangle3d outlineSupport = new Rectangle3d(supPlane, new Interval(-suppColSideLen/2, suppColSideLen/2), new Interval(-suppColSideLen, 0));
                outlineSupport.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));
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
                Point3d inner = spiralCurve.PointAtEnd;
                Point3d outer = spiralCurve.PointAtStart;
                double d1 = other.Model.ClosestPoint(inner).DistanceTo(inner);
                double d2 = other.Model.ClosestPoint(outer).DistanceTo(outer);
                if (d1 < d2)
                    return 2;
                else
                    return 1;
            }
        }
    }
}
