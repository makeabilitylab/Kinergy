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
using Kinergy.Relationship;
using KinergyUtilities;

namespace Kinergy
{
    namespace Geom
    {
        public class Helix:Component
        {
            //the constructors only take direct parameters since calculation of parameter should be finished in motion classes.
            
            private Point3d startPoint = Point3d.Unset;
            private Point3d endPoint= Point3d.Unset;
            private double startT = -1;
            private double endT = -1;
            private Curve skeleton = null;
            private double springRadius = 0;
            private double wireRadius = 0;
            private double length=0;
            private double roundNum=0;
            private double deltaSpringRadius = 0;
            private double deltaWireRadius = 0;
            private double strength = 1;
            private int deltaRoundNum = 0;
            private Vector3d direction = Vector3d.Unset;
            private double maxPressDistance = 0;
            private double compressionDistance = 0;
            private double velocity = 0;
            private double travel = 0;
            private List<LinearDimension> springDimensions;

            public Point3d StartPoint { get => startPoint;private set => startPoint = value; }
            public Point3d EndPoint { get => endPoint; private set => endPoint = value; }
            public double SpringRadius { get => springRadius; private set => springRadius = value; }
            public double WireRadius { get => wireRadius; private set => wireRadius = value; }
            public double Length { get => length; private set => length = value; }
            public double RoundNum { get => roundNum; private set => roundNum = value; }
            public Vector3d Direction { get => direction;private set => direction = value; }
            public Curve Skeleton { get => skeleton;private set => skeleton = value; }
            public List<LinearDimension> SpringDimensions { get => springDimensions;private set => springDimensions = value; }
            public double CompressionDistance { get => compressionDistance;private set => compressionDistance = value; }

            /// <summary> Constructor with only start point and end point given.</summary>
            /// <returns> Returns instance with brep generated</returns>
            public Helix(Point3d startPoint, Point3d endPoint):base(true)
            {
                this.startPoint = startPoint;
                this.endPoint = endPoint;
                Generate();
            }
            /// <summary>
            /// Constructor with only partial parameters given
            /// </summary>
            /// <remarks> Given no parameter,return empty spring with no brep, waiting for future editing</remarks>
            /// <returns> Returns instance with parameters calculated and spring generated</returns>
            public Helix(double len,double spring_Radius =0,double wire_Radius = 0,int round_Num = 0,double maxPress=0.5,double springStrength=1) : base(true)
            {
                length = len;
                if(spring_Radius!=0)
                {springRadius = spring_Radius/2; }
                else { springRadius = 7.5 * length / 25; }
                if(wire_Radius!=0)
                { wireRadius = wire_Radius;}
                else { wireRadius = 0.5 * length / 25; }
                if (round_Num != 0)
                { roundNum = round_Num; }
                else { roundNum = 3; }
                startPoint = new Point3d(0, 0, 0);
                endPoint = new Point3d(length, 0, 0);
                maxPressDistance = maxPress;
                velocity = 0;
                travel = 0;
                strength = springStrength;
                Generate();
            }
            /// <summary>
            /// Constructor with all input parameter given
            /// </summary>
            /// <returns> Returns instance with brep</returns>
            public Helix(Point3d startPoint, Point3d endPoint, double spring_Radius = 0, double wire_Radius =0, int round_Num = 0, double maxPress=0.5, double springStrength = 1) : base(true)
            {
                this.startPoint = startPoint;
                this.endPoint = endPoint;
                direction = new Vector3d(endPoint) - new Vector3d(startPoint);
                length = new Line(this.startPoint, this.endPoint).Length;
                if (spring_Radius != 0)
                { springRadius = spring_Radius; }
                else { springRadius = 7.5 * length / 25; }
                if (wire_Radius != 0)
                { wireRadius = wire_Radius; }
                else { wireRadius = 0.5 * length / 25; }
                if (round_Num != 0)
                { roundNum = round_Num; }
                else { roundNum = 3; }
                maxPressDistance = maxPress;
                velocity = 0;
                travel = 0;
                strength = springStrength;
                Generate();
            }
            public Helix(Curve s,double start,double end, double spring_Radius = 0, double wire_Radius = 0, int round_Num = 0, double maxPress = 0.5,double springStrength = 1) : base(true)
            {
                skeleton = s;
                startT = start;
                endT = end;
                length = skeleton.GetLength() * Math.Abs(start - end);
                if (spring_Radius != 0)
                { springRadius = spring_Radius/2; }
                else { springRadius = 7.5 * length / 25; }
                if (wire_Radius != 0)
                { wireRadius = wire_Radius; }
                else { wireRadius = 0.5 * length / 25; }
                if (round_Num != 0)
                { roundNum = round_Num; }
                else { roundNum = 3; }
                maxPressDistance = maxPress;
                velocity = 0;
                travel = 0;
                strength = springStrength;
                Generate();
            }
            private void FixParameter()
            {

                double clearance = 0.6;
                double G = 2.4;
                double roundNumMin = 3;
                double wireRadiusMin = Math.Max(3, springRadius * 2 / 12);
                double wireRadiusMax = Math.Min(springRadius * 2 / 4, (Length - (roundNumMin - 1) * 0.6) / roundNumMin);
                double roundNumMax = (Length + clearance) / (wireRadiusMin + clearance);

                #region New version by LH

                // Calculate the range of compression displacement
                // double compressionCurr = Length * (1 - (roundNumMin * wireRadiusMax + clearance * (roundNumMin - 1)) / Length) * maxPressDistance;
                double compressionCurr = Length * maxPressDistance;
                compressionDistance = compressionCurr;
                // Calculate the range of energy
                //double strengthMin = Math.Pow(wireRadiusMin, 5) / Math.Pow(Length, 3);
                //double strengthMax = Math.Pow(wireRadiusMax , 2) / Math.Pow(roundNumMin, 3);
                double strengthMin = G * Math.Pow(Length * (1 - (roundNumMax * wireRadiusMin + clearance * (roundNumMax - 1)) / Length), 2) * Math.Pow(wireRadiusMin, 4) /
                                            (16 * Math.Pow(springRadius * 2, 3) * roundNumMax);
                double strengthMax = G * Math.Pow(Length * (1 - (roundNumMin * wireRadiusMax + clearance * (roundNumMin - 1)) / Length), 2) * Math.Pow(wireRadiusMax, 4) /
                                            (16 * Math.Pow(springRadius * 2, 3) * roundNumMin);
   
                double strengthCurr = strengthMin + (strengthMax - strengthMin) * strength;
                strengthCurr = strengthCurr * 4;


                // Test if wire radius is valid
                WireRadius = Math.Log((16 * Math.Pow(springRadius * 2, 3) * strengthCurr * (Length + clearance))/(G * Math.Pow(compressionCurr,3)), 5);

                if (WireRadius < wireRadiusMin)
                {
                    WireRadius = wireRadiusMin;
                }
                else
                {
                    if (WireRadius > wireRadiusMax)
                    {
                        WireRadius = wireRadiusMax;
                    }
                }

                RoundNum = (Length - compressionCurr) / WireRadius;

                #endregion

                #region Xia: Adding dimension lines for user to see the pressed and full length of spring
                if(springDimensions==null)
                {
                    Vector3d v = new Vector3d(new Vector3d(endPoint) - new Vector3d(startPoint));
                    v = v  / v.Length;
                    Plane plane = new Plane(startPoint, v);
                    Vector3d v1 = plane.XAxis, v2 = plane.YAxis;
                    Plane dimensionPlane = new Plane(startPoint, v1);
                    Point3d pt1 = startPoint + v2 * springRadius*1.3, pt2 = endPoint + v2 * springRadius*1.3, pt12 = pt1 + v * new Line(pt1, pt2).Length / 2;
                    Point3d pt3 = startPoint + v2 * springRadius*1.3+v* (Length - compressionCurr), pt13 = pt1 + v * new Line(pt1, pt3).Length / 2;
                    Point3d pt4 = endPoint + v2 * springRadius*1.3-v* (Length - compressionCurr), pt24 = pt2 - v * new Line(pt2, pt4).Length / 2;
                    springDimensions = new List<LinearDimension>();
                    DimensionStyle style = RhinoDoc.ActiveDoc.DimStyles.Current;
                    LinearDimension fullLength = LinearDimension.Create(AnnotationType.Aligned,style, dimensionPlane, v2, pt1, pt2, pt12,Math.PI/2);
                    fullLength.PlainText = "THe full length of spring is "+ Math.Round(fullLength.DistanceBetweenArrowTips,2);
                    LinearDimension pressedLength1 = LinearDimension.Create(AnnotationType.Aligned, style, dimensionPlane, v2, pt1, pt3, pt13, Math.PI / 2);
                    pressedLength1.PlainText = "THe pressed length of spring is "+ Math.Round(pressedLength1.DistanceBetweenArrowTips,2);
                    pressedLength1.Translate(v2 * springRadius * 0.2);
                    LinearDimension pressedLength2 = LinearDimension.Create(AnnotationType.Aligned, style, dimensionPlane, v2, pt2, pt4, pt24, Math.PI / 2);
                    pressedLength2.PlainText = "THe pressed length of spring is "+Math.Round( pressedLength2.DistanceBetweenArrowTips,2);
                    pressedLength2.Translate(v2 * springRadius * 0.4);
                    springDimensions.Add(fullLength);
                    springDimensions.Add(pressedLength1);
                    springDimensions.Add(pressedLength2);
                }
                
                #endregion
            }
            private void GenerateSpring()
            {
                FixParameter();
                
                int points_per_round = 10, point_num = (int)(roundNum * points_per_round);
                var Pi = Math.PI;
                if(skeleton==null)//No skeleton is given, so the spring woul be generated with 2 pts.
                {
                    Vector3d v = new Vector3d(new Vector3d(endPoint) - new Vector3d(startPoint));
                    v = v * (v.Length + travel) / v.Length;
                    //For springs, only their end moves when pressed
                    Plane plane = new Rhino.Geometry.Plane(startPoint, v);
                    Vector3d v1 = plane.XAxis, v2 = plane.YAxis;
                    //Transform rotate = Transform.Rotation(v1, v, new Point3d(0, 0, 0));
                    //v1.Transform(rotate);
                    //v2.Transform(rotate);
                    List<Point3d> pts = new List<Point3d>();
                    for (int i = 0; i <= point_num; i++)
                    {
                        Point3d p = new Point3d(startPoint + v * 1 / point_num * i);
                        p = p + v1 * System.Math.Sin(2 * Pi / points_per_round * i) * springRadius
                            + v2 * System.Math.Cos(2 * Pi / points_per_round * i) * springRadius ;
                        pts.Add(p);
                    }
                    Curve s = Rhino.Geometry.Curve.CreateInterpolatedCurve(pts, 3);
                    List<Curve> sections = new List<Curve>();
                    for(int i=0;i<=point_num;i++)
                    {
                        double t = i / point_num;
                        Plane base_plane = new Rhino.Geometry.Plane(s.PointAtNormalizedLength(t), s.TangentAt(t));//TODO check if average tangent is needed here
                        Circle c = new Circle(base_plane, s.PointAtNormalizedLength(t), wireRadius);
                        Curve cc = NurbsCurve.CreateFromCircle(c);
                        sections.Add(cc);
                    }
                    base.BaseCurve = s;
                    //Brep[] new_Spring = Rhino.Geometry.Brep.CreateFromSweep(s, cc, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                    //Brep[] new_Spring = Rhino.Geometry.Brep.CreateFromLoft(sections,Point3d.Unset, Point3d.Unset,LoftType.Tight,false);
                    Brep[] new_Spring = Brep.CreatePipe(s, wireRadius/2, false, PipeCapMode.Round, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAngleToleranceRadians);
                    base.Model = new_Spring[0];
                }
                else //skeleton is given, so the spring should be generated along skeleton
                {
                    double st = startT;
                    double ed = endT + travel / skeleton.GetLength();
                    //if(st<ed) ed=ed+ travel / skeleton.GetLength();
                    //else ed=ed- travel / skeleton.GetLength();
                    List<Point3d> pts = new List<Point3d>();
                    for (int i = 0; i <= point_num; i++)
                    {
                        Point3d p = skeleton.PointAtNormalizedLength(st+(ed-st)/point_num*i);
                        //tangent is not stable here! use average value instead
                        //Plane pl = new Plane(p, skeleton.TangentAt(skeleton.Domain.ParameterAt(st + (ed - st) / point_num * i)));
                        
                        Vector3d tangent = AverageTangent(skeleton, st + (ed - st) / point_num * i);
                        Plane pl = Plane.WorldXY;
                        Transform rotate = Transform.Rotation(Vector3d.ZAxis, tangent, Point3d.Origin);
                        pl.Transform(rotate);
                        Vector3d v1 = pl.XAxis;
                        v1.Unitize();
                        Vector3d v2 = pl.YAxis;
                        v2.Unitize();
                        p = p + v1 * System.Math.Sin(2 * Pi / points_per_round * i) * springRadius
                            + v2 * System.Math.Cos(2 * Pi / points_per_round * i) * springRadius ;
                        pts.Add(p);
                    }
                    Curve s = Rhino.Geometry.Curve.CreateInterpolatedCurve(pts, 3);
                    
                    base.BaseCurve = s;
                    //Brep[] new_Spring = Rhino.Geometry.Brep.CreateFromSweep(s, cc, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                    Brep[] new_Spring = Brep.CreatePipe(s,wireRadius/2,false,PipeCapMode.Round,true,RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,RhinoDoc.ActiveDoc.ModelAngleToleranceRadians);
                    base.Model = new_Spring[0];
                }

            }
            public static Vector3d AverageTangent(Curve c, double pos, double span=0.05)
            {
                double start = Math.Min(1, pos + span);
                double end = Math.Max(0, pos - span);
                Point3d pt1 = c.PointAtNormalizedLength(start);
                Point3d pt2 = c.PointAtNormalizedLength(end);
                Vector3d tangent = new Vector3d(pt1) - new Vector3d(pt2);
                return tangent;
            }
            
            public override void Generate()
            {
                GenerateSpring();
            }
            public void SetParameter(double spring_Radius=0,double wire_Radius=0,int round_Num=0)
            {
                if(spring_Radius!=0)
                {springRadius = spring_Radius;}
                if(wire_Radius!=0)
                { wireRadius = wire_Radius; }
                if(round_Num!=0)
                { roundNum = round_Num; }
                Generate();
            }
            public void SetPosition(Point3d startPoint,Point3d endPoint)
            {
                this.startPoint = startPoint;
                this.endPoint = endPoint;
                
                 Generate();
            }
            public void ResetPosition()
            {
                startPoint = Point3d.Unset;
                endPoint = Point3d.Unset;
                Generate();
            }
            public void ResetParameter()
            {
                springRadius = 0;
                wireRadius = 0;
                length = 0;
                roundNum = 0;
                Generate();
            }
            public override int GetContactPosition(Entity obj)
            {
                double distance1 = 0,distance2=0;
                if(startPoint==Point3d.Unset)
                {
                    distance1 = new Line(obj.Model.ClosestPoint(skeleton.PointAtNormalizedLength(startT)), skeleton.PointAtNormalizedLength(startT)).Length;
                    distance2 = new Line(obj.Model.ClosestPoint(skeleton.PointAtNormalizedLength(endT)), skeleton.PointAtNormalizedLength(endT)).Length;
                }
                else
                {
                    distance1 = new Line(obj.Model.ClosestPoint(startPoint), startPoint).Length;
                    distance2 = new Line(obj.Model.ClosestPoint(endPoint), endPoint).Length;
                }
                
                if(distance1<distance2)
                { return 1; }
                else { return 2; }
            }
            public override bool Move(Movement move)
            {
                //TODO mobility check
                if(move.Type==3)
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
                            break;
                        }
                    }
                    if (CanIMove)
                    {
                        this.ConductMoveAndUpdateParam(move);
                    }
                    DfsMark = false;
                    return CanIMove;
                }
                else { return base.Move(move); }
                
            }
            public override void ResetState()
            {
                base.ResetState();
                travel = 0;
                GenerateSpring();
            }
            protected override void ConductMoveAndUpdateParam(Movement move)
            {
                //TODO do the linear scaling
                if(move.Type==3)
                {
                    travel += move.MovementValue;
                    GenerateSpring(); //This might not be the ideal way of simulating since its time-consuming
                    
                }
                else if(move.Type==1)
                {
                    Offset =Transform.Multiply(Offset, move.Trans);
                }
            }
            public void AdjustParam(Vector3d mainAxis, List<GearParameter> gear_info, Brep body, int eeMovingDirectionSelection, int e, int D, bool dirCtrl, Point3d eePos = new Point3d())
            {
                if (gear_info != null)
                {
                    // non-instant instances
                    bool lockNorm = false;

                    double gearThickness = 3.6;
                    double rkTeethHeight = 2.25;

                    #region Step 1: find the spring position and length
                    //Now just use first gear center with some offset as the spring end point
                    Vector3d shaftDir = gear_info[0].center - gear_info[1].center;
                    shaftDir.Unitize();
                    //Find the vector that is orthogonal to both the mainAxis and the shaftDir
                    Vector3d rkDir = Vector3d.CrossProduct(shaftDir, mainAxis);
                    rkDir.Unitize();

                    if (dirCtrl)
                    {
                        rkDir = rkDir;
                        //lockNorm = false;
                    }
                    else
                    {
                        rkDir = -rkDir;
                        //lockNorm = true;
                    }

                    Point3d springRkGrConPt = new Point3d();
                    if (lockNorm)
                        springRkGrConPt = gear_info[0].center - rkDir * (gear_info[0].radius + 0.6 + rkTeethHeight / 2) + shaftDir * gearThickness / 2;
                    else
                        springRkGrConPt = gear_info[0].center - rkDir * (gear_info[0].radius + 0.6 + rkTeethHeight / 2) - shaftDir * gearThickness / 2;

                    #region compute spring start point, spring end point, spring length

                    //Find length - 1.5 times the available space
                    double helicalLengthMultiplier = 1.5;//TODO adjust this value;
                    Point3d skeletonStartPoint;
                    if (new Vector3d(skeleton.PointAtNormalizedLength(0)) * mainAxis < new Vector3d(skeleton.PointAtNormalizedLength(1)) * mainAxis)
                        skeletonStartPoint = skeleton.PointAtNormalizedLength(0);
                    else
                        skeletonStartPoint = skeleton.PointAtNormalizedLength(1);

                    double springGearGap = 4;
                    double availableSpace = gear_info[0].center.DistanceTo(skeletonStartPoint) - gear_info[1].radius - springGearGap;
                    double helicalLength = availableSpace * helicalLengthMultiplier;
                    Point3d helicalStartPoint = springRkGrConPt - mainAxis * (helicalLength + gear_info[1].radius + springGearGap);
                    Point3d helicalEndPoint = helicalStartPoint + mainAxis * helicalLength;

                    length = helicalLength;

                    #endregion
                    #endregion

                    #region Step 2: calculate spring parameters

                    //double min_wire_diamter = 2;
                    //double min_coil_num = 2;
                    //double wireRadius = 2;
                    //int roundNum = 0;
                    //double springRadius = 0;

                    //double maxDisp = Math.Max(helicalLength - min_wire_diamter * min_coil_num, min_coil_num * 0.6);
                    //double dis = (D * 0.05 + 0.5) * maxDisp / helicalLength;     // convert the input displacement level into percentage

                    //// Parse the energy based on E ~= d^4/n * x^2
                    //double x = dis * helicalLength;
                    //double energy = e / 10.0;

                    //Curve springCrossLineCrv = new Line(helicalEndPoint - shaftDir * int.MaxValue, helicalEndPoint + shaftDir * int.MaxValue).ToNurbsCurve();
                    //Curve[] springBodyCrvs;
                    //Point3d[] springBodyPts;
                    //Rhino.Geometry.Intersect.Intersection.CurveBrep(springCrossLineCrv, body, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out springBodyCrvs, out springBodyPts);

                    //springRadius = (springBodyPts[0].DistanceTo(springBodyPts[1]) - 2) / 2 - wireRadius - gearThickness / 2;

                    strength = e / 10.0;
                    maxPressDistance = D / 10.0;

                    #endregion

                    #region Step 3: construct spring and rack
                    //Helix helical = new Helix(helicalStartPoint, helicalEndPoint, springRadius, wireRadius, roundNum, dis, energy);
                    //helical.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                    //if (BrepSolidOrientation.Inward == helical.Model.SolidOrientation)
                    //    helical.Model.Flip();
                    //models.Add(helical);

                    //// create the cylinder for deduction
                    //double springSocketRadius = springRadius + wireRadius + 0.6;
                    //Line socketTraj = new Line(helicalEndPoint - mainAxis * wireRadius, helicalEndPoint - mainAxis * wireRadius - mainAxis * helicalLength * 5);
                    //Curve socketCrv = socketTraj.ToNurbsCurve();
                    //helicalSpringSocket = Brep.CreatePipe(socketCrv, springSocketRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
                    //helicalSpringSocket.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                    //if (BrepSolidOrientation.Inward == helicalSpringSocket.SolidOrientation)
                    //    helicalSpringSocket.Flip();
                    Generate();
                    #endregion
                }
            }
            public Movement Activate(double interval,double damp=0.05)
            {
                double G = 2.4 * Math.Pow(10,9);
                double mass = 0.5;
                //velocity += -travel*10 ;// k * travel  / mass * interval / 1000
                double delta = (G * Math.Pow(wireRadius / 2 / 1000, 4) / (8 * roundNum * Math.Pow(SpringRadius / 1000, 3))) / mass * interval / 1000;
                velocity += -travel * delta;
                velocity *= Math.Pow(1 - damp,interval/10);
                Movement m = new Movement(this, 3, velocity * interval / 1000);
                SetMovement(m);
                if(Math.Abs(velocity)<2 &&Math.Abs(travel)<1)
                {
                    m.SetConverge();
                }
                return m;
            }
            public bool SetMovement(Movement m)
            {
                if(m.Type!=3)
                    return false;
                Transform t;
                double v = m.MovementValue;
                /*Vector3d translation = new Vector3d(skeleton.PointAtNormalizedLength(endT + travel / skeleton.GetLength() + v / skeleton.GetLength()))
                    - new Vector3d(skeleton.PointAtNormalizedLength(endT + travel / skeleton.GetLength()));*/
                Vector3d translation = direction / direction.Length * v;
                t =  Transform.Translation(translation);
                /*Transform r = Transform.Rotation(AverageTangent(skeleton, endT + travel / skeleton.GetLength()),
                    AverageTangent(skeleton, endT + travel / skeleton.GetLength() + v / skeleton.GetLength()),
                    skeleton.PointAtNormalizedLength(endT + travel / skeleton.GetLength()));*/
                //m.Trans = t * r;
                m.Trans = t;
                return true;
            }
            /// <summary>
            /// This method reverse the spring, switching its start and end.
            /// </summary>
            public void Reverse()
            {
                //Reverse the start point and end point
                
                if(skeleton!=null)
                {
                    double t= startT;
                    startT = endT;
                    endT = t;
                }
                else
                {
                    Point3d t = startPoint;
                    startPoint = endPoint;
                    endPoint = t;
                }
                Generate();
                //Reverse the contact position state of all constraints
                foreach(Fixation f in constraints)
                {
                    f.ReverseContactPosition();
                }
            }
        }
    }
}
