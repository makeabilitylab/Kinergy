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
        public class Spring
        {
            //the constructors only take direct parameters since calculation of parameter should be finished in motion classes.
            private Brep spring=null;
            private Point3d start=Point3d.Unset, end= Point3d.Unset;
            private double springRadius=0, wireRadius=0, length=0;
            private int roundNum=0;

            /// <summary> Default constructor without any input parameter </summary>
            /// <returns> Returns empty instance</returns>
            public Spring()
            {
                
            }
            /// <summary> Constructor with only start point and end point given.</summary>
            /// <returns> Returns instance with brep generated</returns>
            public Spring(Point3d startPoint, Point3d endPoint)
            {
                start = startPoint;
                end = endPoint;
                spring = GenerateSpring();
            }
            /// <summary>
            /// Constructor with only partial parameters given
            /// </summary>
            /// <remarks> Given no parameter,return empty spring with no brep, waiting for future editing</remarks>
            /// <returns> Returns instance with parameters calculated and spring generated</returns>
            public Spring(double len,double spring_Radius =0,double wire_Radius = 0,int round_Num = 0)
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
                spring = GenerateSpring();
            }
            /// <summary>
            /// Constructor with all input parameter given
            /// </summary>
            /// <returns> Returns instance with brep</returns>
            public Spring(Point3d startPoint, Point3d endPoint, double spring_Radius = 0, double wire_Radius =0, int round_Num = 0)
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
                spring = GenerateSpring();
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
                
                
            }
            private Brep GenerateSpring()
            {
                FixParameter();
                
                int points_per_round = 10, point_num = roundNum * points_per_round;
                var Pi = Math.PI;
                Vector3d v = new Vector3d(new Vector3d(end) - new Vector3d(start));
                Plane plane = new Rhino.Geometry.Plane(start, v);
                Vector3d v1 = plane.XAxis, v2 = plane.YAxis;
                /*Transform rotate = Transform.Rotation(v1, v, new Point3d(0, 0, 0));
                v1.Transform(rotate);
                v2.Transform(rotate);*/
                List<Point3d> pts = new List<Point3d>();
                for (int i = 0; i <= point_num; i++)
                {
                    Point3d p = new Point3d(start + v * 1 / point_num * i);
                    p = p + v1 * System.Math.Sin(2 * Pi / points_per_round * i) * springRadius + v2 * System.Math.Cos(2 * Pi / points_per_round * i) * springRadius;
                    pts.Add(p);
                }
                Curve s = Rhino.Geometry.Curve.CreateInterpolatedCurve(pts, 3);
                Plane spring_plane = new Rhino.Geometry.Plane(s.PointAtNormalizedLength(0), s.TangentAt(0));
                Circle c = new Circle(spring_plane, pts[0], wireRadius);
                Curve cc = NurbsCurve.CreateFromCircle(c);
                Brep[] new_Spring = Rhino.Geometry.Brep.CreateFromSweep(s, cc, true, 0.000001);
                return new_Spring[0];
            }
            public Brep GetModel()
            {
                return spring;
            }
            public Vector3d GetVector()
            {
                return new Vector3d(end)- new Vector3d(start);
            }
            public void Generate()
            {
                GenerateSpring();
            }
            public Brep SetParameter(double spring_Radius=0,double wire_Radius=0,int round_Num=0)
            {
                if(spring_Radius!=0)
                {springRadius = spring_Radius;}
                if(wire_Radius!=0)
                { wireRadius = wire_Radius; }
                if(round_Num!=0)
                { roundNum = round_Num; }
                spring = GenerateSpring();
                return spring;
            }
            public Brep SetPosition(Point3d startPoint,Point3d endPoint)
            {
                start = startPoint;
                end = endPoint;
                
                spring = GenerateSpring();
                return spring;
            }
            public Brep ResetPosition()
            {
                start = Point3d.Unset;
                end = Point3d.Unset;
                spring = GenerateSpring();
                return spring;
            }
            public Brep ResetParameter()
            {
                springRadius = 0;
                wireRadius = 0;
                length = 0;
                roundNum = 0;
                spring = GenerateSpring();
                return spring;
            }
            public Brep Rotate(double angle)
            {
                spring.Rotate(angle, new Vector3d(end)-new Vector3d(start), start);
                return spring;
            }
        }
    }
}
