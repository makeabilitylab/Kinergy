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
        public class Rack:Component
        {
            private double length = 0;
            private double rootHeight = 0;
            private double tipHeight = 0;
            private double toothDepth = 0;//=tR-rR;
            //private double factorA = -1, factorB = -1;//Cancelled since adjustment of teeth should be automatically done.
            private int z;//Total number of teeth
            private double pitch = 0;//pitch ,distance between corresponding points on adjacent teeth. p=m*Pi;
            private double module = 0;//module of teeth size, 
            private double faceWidth = 0;
            private Point3d centerPoint = Point3d.Unset;
            private Point3d startPoint = Point3d.Unset;
            private Point3d endPoint = Point3d.Unset;
            private Vector3d faceDirection = Vector3d.Unset;
            private Vector3d rackDirection = Vector3d.Unset;
            private Vector3d extrudeDirection = Vector3d.Unset;
            private Line backBone = Line.Unset;

            public double Length { get => length; private set => length = value; }
            public double RootHeight { get => rootHeight; private set => rootHeight = value; }
            public double TipHeight { get => tipHeight; private set => tipHeight = value; }
            public double ToothDepth { get => toothDepth; private set => toothDepth = value; }
            /// <summary>
            /// Number of teeth. It's auto calculated using module in constructor.
            /// </summary>
            public int Z { get => z; private set => z = value; }
            /// <summary>
            /// Actual distance between teeth.
            /// </summary>
            public double Pitch { get => pitch; private set => pitch = value; }
            public double Module { get => module; private set => module = value; }
            public double FaceWidth { get => faceWidth; private set => faceWidth = value; }
            public Point3d CenterPoint { get => centerPoint; private set => centerPoint = value; }
            public Point3d StartPoint { get => startPoint; private set => startPoint = value; }
            public Point3d EndPoint { get => endPoint; private set => endPoint = value; }
            public Vector3d FaceDirection { get => faceDirection; private set => faceDirection = value; }
            public Line BackBone { get => backBone; private set => backBone = value; }
            public Vector3d RackDirection { get => rackDirection;private set => rackDirection = value; }

            public Rack(Point3d CP,Vector3d RackLineDirection,Vector3d RackFaceDirection,double len,double rH,double tD,double m=1,double Thickness = 4)
            {
                rackDirection = RackLineDirection/RackLineDirection.Length;
                faceDirection = RackFaceDirection;
                faceDirection.Unitize();
                length = len;
                faceWidth = Thickness;
                centerPoint = CP;
                startPoint = CP - RackLineDirection / RackLineDirection.Length * (len / 2);
                endPoint = CP + RackLineDirection / RackLineDirection.Length * (len / 2);
                backBone = new Line(startPoint, endPoint);
                module = m;
                pitch = m * Math.PI;
                RootHeight = rH;
                toothDepth = tD;
                tipHeight = rootHeight + toothDepth;
                z = (int) Math.Round(length / pitch);
                pitch = length / z;
                extrudeDirection = new Plane(CP, rackDirection, faceDirection).Normal;
                Generate();
            }

            public Rack(Point3d start,Point3d end, Vector3d RackFaceDirection, double rH, double tD, double m = 1, double Thickness = 4)
            {
                backBone = new Line(start, end);
                centerPoint = backBone.PointAt(0.5);
                rackDirection = new Vector3d(end)- new Vector3d(start);
                rackDirection /= rackDirection.Length;
                faceDirection = RackFaceDirection;
                faceDirection.Unitize();
                length = backBone.Length;
                faceWidth = Thickness;
                startPoint = start;
                endPoint = end;
                
                module = m;
                pitch = m * Math.PI;
                RootHeight = rH;
                toothDepth = tD;
                tipHeight = rootHeight + toothDepth;
                z = (int)Math.Round(length / pitch);
                pitch = length / z;
                extrudeDirection = new Plane(centerPoint, rackDirection, faceDirection).Normal;
                Generate();
            }
            private void FixParameter()
            {

            }
            protected override void GenerateBaseCurve()
            {
                FixParameter();
                
                Transform move = Transform.Translation(faceDirection * rootHeight);
                Curve rootLine = backBone.ToNurbsCurve();
                rootLine.Transform(move);
                List<Point3d> pts = new List<Point3d>();
                move = Transform.Translation(faceDirection * toothDepth);
                for (int i=0;i<=z*4;i++)
                {
                    pts.Add(rootLine.PointAtNormalizedLength(i / (z * 4)));
                    if(i % 4 == 1 || i % 4 == 2)
                    {
                        pts[i].Transform(move);
                    }
                }
                pts.Add(endPoint);
                pts.Add(startPoint);
                List<Curve> LineSegments = new List<Curve>();
                for(int i=1;i<z*4+3;i++)
                {
                    LineSegments.Add(new Line(pts[i - 1], pts[i]).ToNurbsCurve());
                }
                LineSegments.Add(new Line(pts[z * 4 + 2], pts[0]).ToNurbsCurve());
                base.BaseCurve = PolyCurve.JoinCurves(LineSegments)[0];
            }
            protected void GenerateRack()
            {
                GenerateBaseCurve();
                
                Point3d startPoint = centerPoint - extrudeDirection * length*0.5;
                Point3d endPoint = centerPoint + extrudeDirection * length * 0.5;
                Curve rail = new Line(startPoint, endPoint).ToNurbsCurve();
                base.Model = Brep.CreateFromSweep(rail, base.BaseCurve, true, 0.00000001)[0];
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
                    if(Math.Abs(g.Direction*extrudeDirection)<0.99)
                    { return false; }
                    if(Math.Abs(g.Direction*new Vector3d(g.CenterPoint))- Math.Abs(extrudeDirection * new Vector3d(CenterPoint))>(faceWidth+g.FaceWidth)/2)
                    { return false; }
                    //and the distance of centers have to be between R1+R2 and r1+r2+h(smaller).
                    double distance1 = backBone.DistanceTo(g.CenterPoint,true);
                    double distance2 = new Plane(CenterPoint,faceDirection,rackDirection).DistanceTo(g.CenterPoint);
                    double distance3 = Math.Sqrt(distance1 * distance1 - distance2 * distance2);
                    if(distance3>tipHeight+g.TipRadius || distance3 < rootHeight + g.RootRadius + (toothDepth + g.ToothDepth) / 2) 
                    { return false; }
                    //Their teeth relationship
                    if (Math.Abs(module - g.Module) > 0.01)
                    { return false; }
                    //Actually the engagement of gears could be very complicated. Herer I only cover the basic parts due to my very limited knowledge in this field.

                }
                else { return false; }
                return true;
            }
        }
    }
}
