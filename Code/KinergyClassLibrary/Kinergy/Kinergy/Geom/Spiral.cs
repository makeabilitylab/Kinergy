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
            private double thicknessX = 0, thicknessY = 0;
            private Point3d centerPoint = Point3d.Unset;
            private Vector3d direction = Vector3d.Unset;
            private Curve spiralCurve = null;
            private Brep spiralBrep = null;
            private int pointsPerRound = 20;
            private double angleLoaded;
            private double velocity = 0;
            private int FrameCounter = 0;
            public int PointsPerRound { get => pointsPerRound;private set => pointsPerRound = value; }
            public Point3d CenterPoint { get => centerPoint;private set => centerPoint = value; }
            public Vector3d Direction { get => direction;private  set => direction = value; }
            public double ThicknessX { get => thicknessX;private  set => thicknessX = value; }
            public double ThicknessY { get => thicknessY;private  set => thicknessY = value; }

            /// <summary> Default constructor without any input parameter </summary>
            /// <returns> Returns empty instance</returns>


            /// <summary> Constructor with center point and direction given </summary>
            /// <returns> Returns instance with spiral brep generated</returns>
            public Spiral(Point3d Center, Vector3d Direction, double R , double r, int RoundNum , double ThicknessX = 1,double ThicknessY = 1, double initialAngle=0)
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
            }
            public Spiral(Vector3d Direction,Point3d Center,  double R,  double maxD,int energy=5, double initialAngle = 0)
            {
                angleLoaded = initialAngle;
                center = new Point3d(Center);
                centerPoint = new Point3d(Center);
                direction = Direction;
                outerRadius = R;
                innerRadius = 2;
                thicknessX = 1.5;
                //First calculate n with dis
                double maxDegree = maxD;
                double theta = maxDegree;
                int bestN = 1;
                thicknessX = 0.8;
                thicknessY = 7;
                /*double bestMatchingLoss = double.MaxValue;
                for (int i = 1; i < 10; i++)
                {
                    //find out the best N
                    double L = i * (outerRadius + innerRadius) * Math.PI;
                    double term1 = (outerRadius+innerRadius) * Math.PI * (Math.Sqrt(4 * innerRadius * innerRadius + 1.27 * L * thicknessX) - innerRadius * 2-2*thicknessX*theta);
                    double term2 = L *2*thicknessX;
                    double matchingLoss = Math.Abs(Math.Log(Math.Abs(term1 / term2)));
                    if (matchingLoss < bestMatchingLoss)
                    {
                        bestMatchingLoss = matchingLoss;
                        bestN = i;
                    }
                }
                roundNum = bestN;*/
                roundNum = 2 + maxDegree / (Math.PI) ;
                //Then calculate thicknessY&X with energy
                //double Len = roundNum * (outerRadius + innerRadius) * Math.PI;
                //Maybe just simplify it by turning the 1-10 M to the power of 0.25, and 0.75, then expand Y&X respectively
                double energy033 = Math.Pow((energy + 2)/2, 0.33);
                thicknessX *= energy033;

                LoadSpiral(initialAngle);
            }
            public  void AdjustParam(int e, double D)
            {
                double maxDegree = D;
                double theta = maxDegree;
                int bestN = 1;
                thicknessX = 1.2;
                thicknessY = 8;
                /*double bestMatchingLoss = double.MaxValue;
                for (int i = 1; i < 10; i++)
                {
                    double L = i * (outerRadius + innerRadius) * Math.PI;
                    double term1 = (outerRadius + innerRadius) * Math.PI * (Math.Sqrt(4 * innerRadius * innerRadius + 1.27 * L * thicknessX) - innerRadius * 2 - 2 * thicknessX * theta);
                    double term2 = L * 2 * thicknessX;
                    double matchingLoss = Math.Abs(Math.Log(Math.Abs(term1 / term2)));
                    if (matchingLoss < bestMatchingLoss)
                    {
                        bestMatchingLoss = matchingLoss;
                        bestN = i;
                    }
                }
                roundNum = bestN;*/
                roundNum = 2 + maxDegree / (Math.PI);
                //Then calculate thicknessY&X with energy
                //double Len = roundNum * (outerRadius + innerRadius) * Math.PI;
                //Maybe just simplify it by turning the 1-10 M to the power of 0.25, and 0.75, then expand Y&X respectively
                double energy033 = Math.Pow((e + 1) , 0.33);
                thicknessX *= energy033;
                
                LoadSpiral(0);
            }
            /// <summary> Constructor with parameter but no center point given </summary>
            /// <returns> Returns instance with gear brep generated</returns>

            private void FixParameter()
            {
                if (outerRadius == 0)
                { outerRadius = 30; }
                if (innerRadius == 0)
                { outerRadius = 5; }
                if (thicknessX == 0)
                { thicknessX = 2; }
                if (thicknessY == 0)
                { thicknessX = 2; }
                if (outerRadius < innerRadius)
                {
                    double t = innerRadius;
                    innerRadius = outerRadius;
                    outerRadius = t;
                }
                if (outerRadius == innerRadius)
                {
                    innerRadius = outerRadius / 3;
                }
                if (thicknessX > (outerRadius - innerRadius) / roundNum)
                {
                    thicknessX = (outerRadius - innerRadius) / roundNum / 2;
                }
                if(angleLoaded>roundNum*Math.PI*2*(outerRadius-innerRadius)/(2*outerRadius+4*innerRadius)*0.5)
                {
                    angleLoaded = roundNum * Math.PI * 2 * (outerRadius - innerRadius) / (2 * outerRadius + 4 * innerRadius)*0.5;
                }
            }
            private void GenerateSpiralCurve()
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
                //Did some calculus here to use a quadratic polynomial to compute the loaded spiral curve.
                double alpha = 3*angleLoaded * (outerRadius + innerRadius) / Math.Pow(totalAngle, 3);
                double beta = (2 * PI * roundNum * (outerRadius - innerRadius) - 2 * angleLoaded * outerRadius - 4 * angleLoaded * innerRadius) / Math.Pow(totalAngle, 2);

                for (int i = 0; i < numPoints; i++)
                {
                    double angle = initialAngle + i * totalAngle / (numPoints - 1);
                    double radius = alpha *Math.Pow(i * totalAngle / (numPoints - 1), 2) + beta * (i * totalAngle / (numPoints - 1)) + innerRadius;
                    //Point3d newPt = CP + (X * Math.Cos(angle) + Y * Math.Sin(angle)) * radius;
                    pts.Add(centerPoint + (X * Math.Cos(angle) + Y * Math.Sin(angle)) *radius);
                    //pts.Add(newPt);
                }
                Curve s = Rhino.Geometry.Curve.CreateInterpolatedCurve(pts, 3);
                
                /*string body = string.Format("Spiral center is now at {0} , {1} , {2} ", center.X, center.Y, center.Z);
                Rhino.RhinoApp.WriteLine(body);*/
                base.BaseCurve = s;
                spiralCurve = s;
            }
            private void GenerateSpiralBrep()
            {
                //Point3d CP = new Point3d(centerPoint);
                Plane basePlane = new Plane(centerPoint, direction);
                Vector3d X = basePlane.XAxis;
                Plane recPlane = new Plane(centerPoint + X * innerRadius, X, direction);
                Rectangle3d outline = new Rectangle3d(recPlane, new Interval(-thicknessX * 0.5, thicknessX * 0.5), new Interval(0, thicknessY));
                outline.Transform(Transform.Rotation(-angleLoaded, direction, centerPoint));
                Brep b = Brep.CreateFromSweep(base.BaseCurve, outline.ToNurbsCurve(), true, 0.00000001)[0];
                b = b.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                if (b == null)
                    throw new Exception("Failed when capping");
                base.Model = b;

            }
            public void LoadSpiral(double degree)
            {
                angleLoaded += degree;
                Generate();
            }
            public override void Generate()
            {
                GenerateSpiralCurve();
                GenerateSpiralBrep();
            }
            
            public void SetPosition(Point3d C, Vector3d Direction)
            {
                center = C;
                centerPoint = C;
                direction = Direction;
                GenerateSpiralBrep();
            }
            public void SetParameter(double R = 0, double r = 0, double ThicknessX = 0, double ThicknessY = 0, int RoundNum = 0)
            {
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
                GenerateSpiralBrep();
            }
            
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
                Point3d inner = spiralCurve.PointAtStart;
                Point3d outer = spiralCurve.PointAtEnd;
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
