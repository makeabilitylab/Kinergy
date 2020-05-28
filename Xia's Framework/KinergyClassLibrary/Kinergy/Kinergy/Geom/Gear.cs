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
        public class Gear:Component
        {
            private double rootRadius=0;
            private double tipRadius=0;
            private double toothDepth=0;//=tR-rR;
            //private double factorA = -1, factorB = -1;//Cancelled since adjustment of teeth should be automatically done.
            private int z=0 ;//Total number of teeth
            private double pitch=0;//pitch ,distance between corresponding points on adjacent teeth. p=m*Pi;
            private double module = 0;//module of teeth size, 
            private double faceWidth = 0;
            private Point3d centerPoint=Point3d.Unset;
            private Vector3d direction=Vector3d.Unset;

            public double RootRadius { get => rootRadius; protected set => rootRadius = value; }
            public double TipRadius { get => tipRadius; protected set => tipRadius = value; }
            public double ToothDepth { get => toothDepth; protected set => toothDepth = value; }
            public int Z { get => z; protected set => z = value; }
            public double Pitch { get => pitch; protected set => pitch = value; }
            public double Module { get => module; protected set => module = value; }
            public double FaceWidth { get => faceWidth; protected set => faceWidth = value; }
            public Point3d CenterPoint { get => centerPoint; protected set => centerPoint = value; }
            public Vector3d Direction { get => direction; protected set => direction = value; }

            //private Brep surface = null;

            /// <summary> Constructor with center point and direction given </summary>
            /// <returns> Returns instance with gear brep generated</returns>
            /// <param name="CP">Center Point of gear</param>
            /// <param name="module">Module of gear teeth, this parameter determines how big teeth are</param>
            public Gear(Point3d CP, Vector3d GearDirection, double rR , double tD=1 , double mod = 1, double Thickness = 4)
            {
                center = CP;
                direction = GearDirection;
                
                rootRadius = rR;
                toothDepth = tD;
                module = mod;
                faceWidth = Thickness;
                GenerateGear();
            }
            /// <summary> Constructor with parameter but no center point given </summary>
            /// <returns> Returns instance with gear brep generated</returns>
            public Gear(double rR, double tD=1, double mod = 1, double Thickness = 4)
            {

                rootRadius = rR;
                toothDepth = tD;
                module = mod;
                faceWidth = Thickness;
                GenerateGear();
            }
            public Gear(int teethNum, double tD = 1, double mod = 1, double Thickness = 4)
            {
                z = teethNum;
                
                toothDepth = tD;
                module = mod;
                faceWidth = Thickness;
                GenerateGear();
            }
            private void FixParameter()
            {
                
                if (rootRadius == 0 && z==0)
                { rootRadius = 24; }
                else if(rootRadius==0 && z>0)//given z and mod,calculate r
                {
                    pitch = module * Math.PI;
                    rootRadius = z * module / 2;
                }
                tipRadius = rootRadius + toothDepth;
                if (toothDepth==0)
                { toothDepth = rootRadius / 10; }
                
                if (center == Point3d.Unset)
                { center = new Point3d(0, 0, 0); }
                if (direction == Vector3d.Unset)
                { direction = new Vector3d(0, 0, 1); }
                direction.Unitize();
                if(z==0)
                {
                z =(int) Math.Round(2 * rootRadius / module);
                pitch = 2 * rootRadius * Math.PI / z;
                }
                
            }

            protected override void  GenerateBaseCurve()
            {
                FixParameter();
                //First draw circle and extract points
                Plane basePlane = new Plane(center, direction);
                Circle baseCircle = new Circle(basePlane, rootRadius);
                Curve circle = baseCircle.ToNurbsCurve();
                //draw tiny lines with points and move out
                List<Point3d> pts = new List<Point3d>();
                for (int i = 0; i < z * 2; i++)
                {
                    pts.Add(circle.PointAtNormalizedLength(i / z * 2));
                }
                List<Curve> lines1 = new List<Curve>();
                List<Curve> lines2 = new List<Curve>();
                for (int i = 0; i < z; i++)
                {
                    lines1.Add(new Line(pts[i * 2], pts[i * 2 + 1]).ToNurbsCurve());
                }
                for (int i = 0; i < z - 1; i++)
                {
                    lines2.Add(new Line(pts[i * 2 + 1], pts[i * 2 + 2]).ToNurbsCurve());
                }
                lines2.Add(new Line(pts[z * 2 - 1], pts[0]).ToNurbsCurve());

                List<Point3d> outerPoints = new List<Point3d>();
                List<Point3d> innerPoints = new List<Point3d>();
                for (int i = 0; i < z; i++)
                {
                    Point3d middlePoint = lines1[i].PointAtNormalizedLength(0.5);
                    Vector3d v = new Vector3d(middlePoint) - new Vector3d(center);
                    v.Unitize();
                    v *= toothDepth;
                    Transform move = Transform.Translation(v);
                    lines1[i].Transform(move);
                    Point3d p1 = lines1[i].PointAtNormalizedLength(0.5 - 0.5 / 2);
                    Point3d p2 = lines1[i].PointAtNormalizedLength(0.5 + 0.5 / 2);
                    outerPoints.Add(p1);
                    outerPoints.Add(p2);
                }
                for (int i = 0; i < z; i++)
                {
                    Point3d p1 = lines2[i].PointAtNormalizedLength(0.5 - 0.8 / 2);
                    Point3d p2 = lines2[i].PointAtNormalizedLength(0.5 + 0.8 / 2);
                    innerPoints.Add(p1);
                    innerPoints.Add(p2);
                }
                //link all line segments and generate surface
                List<Curve> lineSegments = new List<Curve>();
                /*List<Curve> lineSegments2 = new List<Curve>();
                List<Curve> lineSegments3 = new List<Curve>();
                List<Curve> lineSegments4 = new List<Curve>();*/
                for (int i = 0; i < z - 1; i++)
                {
                    lineSegments.Add(new Line(outerPoints[i * 2], outerPoints[i * 2 + 1]).ToNurbsCurve());
                    lineSegments.Add(new Line(outerPoints[i * 2 + 1], innerPoints[i * 2]).ToNurbsCurve());
                    lineSegments.Add(new Line(innerPoints[i * 2], innerPoints[i * 2 + 1]).ToNurbsCurve());
                    lineSegments.Add(new Line(innerPoints[i * 2 + 1], outerPoints[i * 2 + 2]).ToNurbsCurve());
                }
                lineSegments.Add(new Line(outerPoints[z * 2 - 2], outerPoints[z * 2 - 1]).ToNurbsCurve());
                lineSegments.Add(new Line(outerPoints[z * 2 - 1], innerPoints[z * 2 - 2]).ToNurbsCurve());
                lineSegments.Add(new Line(innerPoints[z * 2 - 2], innerPoints[z * 2 - 1]).ToNurbsCurve());
                lineSegments.Add(new Line(innerPoints[z * 2 - 1], outerPoints[0]).ToNurbsCurve());
                base.BaseCurve = PolyCurve.JoinCurves(lineSegments)[0];
                //TODO improve this generation process by drawing tiny tooth curves
            }

            private void GenerateGear()
            {
                GenerateBaseCurve();
                //Extrude
                Vector3d v = direction / direction.Length * faceWidth;
                Point3d startPoint = center - v*0.5;
                Point3d endPoint = center + v*0.5;
                Curve rail = new Line(startPoint, endPoint).ToNurbsCurve();
                base.Model = Brep.CreateFromSweep(rail, base.BaseCurve, true, 0.00000001)[0];

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
                
                if(obj.GetType() == typeof(Gear))
                {
                    Gear g = (Gear)obj;
                    //If two gears are engaged, they have to be in same direction, their interval of z direction have to overlap
                    Vector3d d1 = direction / direction.Length;
                    Vector3d d2 = g.Direction / g.Direction.Length;
                    if (Math.Abs(d1*d2)<0.99)
                    { return false; }
                    if(Math.Abs(new Vector3d(CenterPoint)*d1)-Math.Abs(new Vector3d(g.CenterPoint)*d2)>(faceWidth+g.FaceWidth)/2)
                    { return false; }
                    //and the distance of centers have to be between R1+R2 and r1+r2+h(smaller)
                    Vector3d vCP1 = new Vector3d(CenterPoint);
                    Vector3d vCP2 = new Vector3d(g.CenterPoint);
                    vCP1 -= vCP1 * d1 * d1;
                    vCP2 -= vCP2 * d2 * d2;
                    double distance = (vCP1 - vCP2).Length;
                    if(distance>tipRadius+g.TipRadius || distance<rootRadius+g.RootRadius+(toothDepth+g.ToothDepth)/2)
                    { return false; }
                    //Their teeth relationship
                    if (Math.Abs(module-g.Module)>0.01)
                    { return false; }
                    //Actually the engagement of gears could be very complicated. Herer I only cover the basic parts due to my very limited knowledge in this field.

                }
                else if(obj.GetType() == typeof(Rack))
                {
                    Rack r = (Rack)obj;
                    //If the gear is engaged with a rack, their direction vector should be prependicular and their interval of z direction have to overlap
                    Vector3d d1 = direction / direction.Length;
                    Vector3d d2 = r.RackDirection / r.RackDirection.Length;
                    if (Math.Abs(d1 * d2) >0.01)
                    { return false; }
                    double distance1 = new Plane(centerPoint, direction).DistanceTo(r.CenterPoint);
                    if (distance1>(faceWidth+r.FaceWidth)/2)
                    { return false; }
                    //and the distance should be between R1+R2 and r1+r2+h(smaller).
                    double distance2 = r.BackBone.DistanceTo(centerPoint,true);
                    if(Math.Sqrt(distance2*distance2-distance1*distance1)> tipRadius + r.TipHeight
                        || Math.Sqrt(distance2 * distance2 - distance1 * distance1) < rootRadius + r.RootHeight + (toothDepth + r.ToothDepth) / 2)
                    {
                        return false;
                    }
                    //Their teeth relationship
                    if(Math.Abs(module - r.Module) > 0.01)
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
