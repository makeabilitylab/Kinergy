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
using Kinergy.Constraints;

namespace Kinergy
{
    namespace Geom
    {
        public class Spring:Component
        {
            //the constructors only take direct parameters since calculation of parameter should be finished in motion classes.
            
            private Point3d start = Point3d.Unset;
            private Point3d end= Point3d.Unset;
            private double springRadius = 0;
            private double wireRadius = 0;
            private double length=0;
            private int roundNum=0;
            private double deltaSpringRadius = 0;
            private double deltaWireRadius = 0;
            private int deltaRoundNum = 0;
            private Vector3d direction = Vector3d.Unset;
            private double maxPressDistance = 0;
            private double velocity = 0;
            private double travel = 0;

            public Point3d Start { get => start;private set => start = value; }
            public Point3d End { get => end; private set => end = value; }
            public double SpringRadius { get => springRadius; private set => springRadius = value; }
            public double WireRadius { get => wireRadius; private set => wireRadius = value; }
            public double Length { get => length; private set => length = value; }
            public int RoundNum { get => roundNum; private set => roundNum = value; }
            public Vector3d Direction { get => direction;private set => direction = value; }
            public double MaxPressDistance { get => maxPressDistance;private set => maxPressDistance = value; }

            /// <summary> Constructor with only start point and end point given.</summary>
            /// <returns> Returns instance with brep generated</returns>
            public Spring(Point3d startPoint, Point3d endPoint):base(true)
            {
                start = startPoint;
                end = endPoint;
                Generate();
            }
            /// <summary>
            /// Constructor with only partial parameters given
            /// </summary>
            /// <remarks> Given no parameter,return empty spring with no brep, waiting for future editing</remarks>
            /// <returns> Returns instance with parameters calculated and spring generated</returns>
            public Spring(double len,double spring_Radius =0,double wire_Radius = 0,int round_Num = 0,double maxPress=0.5) : base(true)
            {
                length = len;
                if(spring_Radius!=0)
                {springRadius = spring_Radius; }
                else { springRadius = 7.5 * length / 25; }
                if(wire_Radius!=0)
                { wireRadius = wire_Radius;}
                else { wireRadius = 0.5 * length / 25; }
                if (round_Num != 0)
                { roundNum = round_Num; }
                else { roundNum = 5; }
                start = new Point3d(0, 0, 0);
                end = new Point3d(length, 0, 0);
                maxPressDistance = maxPress;
                velocity = 0;
                travel = 0;
                Generate();
            }
            /// <summary>
            /// Constructor with all input parameter given
            /// </summary>
            /// <returns> Returns instance with brep</returns>
            public Spring(Point3d startPoint, Point3d endPoint, double spring_Radius = 0, double wire_Radius =0, int round_Num = 0, double maxPress=0.5) : base(true)
            {
                start = startPoint;
                end = endPoint;
                length = new Line(start, end).Length;
                if (spring_Radius != 0)
                { springRadius = spring_Radius; }
                else { springRadius = 7.5 * length / 25; }
                if (wire_Radius != 0)
                { wireRadius = wire_Radius; }
                else { wireRadius = 0.5 * length / 25; }
                if (round_Num != 0)
                { roundNum = round_Num; }
                else { roundNum = 5; }
                maxPressDistance = maxPress;
                velocity = 0;
                travel = 0;
                Generate();
            }
            private void FixParameter()
            {   
                if (start == Point3d.Unset) { start = new Point3d(0, 0, 0); }
                if (end == Point3d.Unset && length>0) 
                { end = new Point3d(length, 0, 0); }
                else if(end == Point3d.Unset && length == 0) 
                { length = 25; end = new Point3d(length, 0, 0); }
                if (length == 0) { length = 25; }
                if (springRadius == 0) { springRadius = 7.5; }
                if (wireRadius == 0) { wireRadius = 0.5; }
                if (roundNum == 0) { roundNum = 5; }
                direction = new Vector3d(end) - new Vector3d(start);
            }
            private void GenerateSpring()
            {
                FixParameter();
                if(roundNum + deltaRoundNum<3)
                {
                    deltaRoundNum = 3 - roundNum;
                }
                if(springRadius + deltaSpringRadius<4)
                {
                    deltaSpringRadius = 4 - springRadius;
                }
                if(wireRadius+deltaWireRadius<0.2)
                {
                    deltaWireRadius = 0.2 - wireRadius;
                }
                int points_per_round = 10, point_num = (roundNum+deltaRoundNum) * points_per_round;
                var Pi = Math.PI;
                Vector3d v = new Vector3d(new Vector3d(end) - new Vector3d(start));
                v = v * (v.Length + travel) / v.Length;
                Plane plane = new Rhino.Geometry.Plane(start, v);
                Vector3d v1 = plane.XAxis, v2 = plane.YAxis;
                Transform rotate = Transform.Rotation(v1, v, new Point3d(0, 0, 0));
                v1.Transform(rotate);
                v2.Transform(rotate);
                List<Point3d> pts = new List<Point3d>();
                for (int i = 0; i <= point_num; i++)
                {
                    Point3d p = new Point3d(start + v * 1 / point_num * i);
                    p = p + v1 * System.Math.Sin(2 * Pi / points_per_round * i) * (springRadius+deltaSpringRadius) 
                        + v2 * System.Math.Cos(2 * Pi / points_per_round * i) * (springRadius + deltaSpringRadius);
                    pts.Add(p);
                }
                Curve s = Rhino.Geometry.Curve.CreateInterpolatedCurve(pts, 3);
                Plane spring_plane = new Rhino.Geometry.Plane(s.PointAtNormalizedLength(0), s.TangentAt(0));
                Circle c = new Circle(spring_plane, pts[0], wireRadius+deltaWireRadius);
                Curve cc = NurbsCurve.CreateFromCircle(c);
                base.BaseCurve = cc;
                Brep[] new_Spring = Rhino.Geometry.Brep.CreateFromSweep(s, cc, true, 0.000001);
                base.Model= new_Spring[0];
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
                start = startPoint;
                end = endPoint;
                
                 Generate();
            }
            public void ResetPosition()
            {
                start = Point3d.Unset;
                end = Point3d.Unset;
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
                double distance1 = new Line(obj.Model.ClosestPoint(start), start).Length;
                double distance2 = new Line(obj.Model.ClosestPoint(end), end).Length;
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
                    foreach (Constraint c in constraints)
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
            protected override void ConductMoveAndUpdateParam(Movement move)
            {
                //TODO do the linear scaling
                if(move.Type==3)
                {
                    travel += move.MovementValue;
                    GenerateSpring();
                }
                else if(move.Type==1)
                {
                    Offset =Transform.Multiply(Offset, move.Trans);
                }
            }
            public void AdjustParameter(double deltaR,int deltaN,double deltaT)
            {
                deltaSpringRadius = deltaR;
                deltaRoundNum = deltaN;
                deltaWireRadius = deltaT;
            }
            public Movement Activate(double interval,double damp=0.01)
            {
                velocity += -travel * 0.1;//TODO adjust these parameters
                velocity *= (1 - damp);
                return new Movement(this, 3, velocity * interval / 1000);
            }
        }
    }
}
