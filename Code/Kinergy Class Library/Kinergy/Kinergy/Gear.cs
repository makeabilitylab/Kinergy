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
        public class Gear
        {
            private double radius = 0;
            private double amplitude = 0;
            private double factorA = -1, factorB = -1;
            private int count = 0;
            private double thickness = 0;
            private Point3d center = Point3d.Unset;
            private Vector3d direction = Vector3d.Unset;
            private Curve outline = null;
            private Brep surface = null;
            private Brep gear = null;

            /// <summary> Default constructor without any input parameter </summary>
            /// <returns> Returns empty instance</returns>
            public Gear()
            {

            }
            /// <summary> Constructor with center point and direction given </summary>
            /// <returns> Returns instance with gear brep generated</returns>
            public Gear(Point3d Center, Vector3d Direction, double Radius = 0, double Amplitude = -1, double A = -1, double B = -1, int Count = 0, double Thickness = 0)
            {
                center = Center;
                direction = Direction;
                if (Radius > 0)
                { radius = Radius; }
                if (Amplitude > 0)
                { amplitude = Amplitude; }
                if (A >= 0 && A <= 1)
                { factorA = A; }
                if (B >= 0 && B <= 1)
                { factorB = B; }
                if (Count > 0)
                { count = Count; }
                if (Thickness > 0)
                { thickness = Thickness; }
                gear = GenerateGear();
            }
            /// <summary> Constructor with parameter but no center point given </summary>
            /// <returns> Returns instance with gear brep generated</returns>
            public Gear(double Radius = 0, double Amplitude = -1, double A = -1, double B = -1, int Count = 0, double Thickness = 0)
            {
                if (Radius > 0)
                { radius = Radius; }
                if (Amplitude > 0)
                { amplitude = Amplitude; }
                if (A >= 0 && A <= 1)
                { factorA = A; }
                if (B >= 0 && B <= 1)
                { factorB = B; }
                if (Count > 0)
                { count = Count; }
                if (Thickness > 0)
                { thickness = Thickness; }
                gear = GenerateGear();
            }

            private void FixParameter()
            {
                if (radius <= 0)
                { radius = 24; }
                if (amplitude <= 0)
                { amplitude = 4; }
                if (factorA < 0 || factorA > 1)
                { factorA = 0.5; }
                if (factorB > 1 || factorB < 0)
                { factorB = 1; }
                if (count <= 0)
                { count = 36; }
                if (thickness > 0)
                { thickness = 1; }
                if (center == Point3d.Unset)
                { center = new Point3d(0, 0, 0); }
                if (direction == Vector3d.Unset)
                { direction = new Vector3d(0, 0, 1); }
            }

            private Brep GenerateGearSurface()
            {
                FixParameter();
                //First draw circle and extract points
                Plane basePlane = new Plane(center, direction);
                Circle baseCircle = new Circle(basePlane, radius);
                Curve circle = baseCircle.ToNurbsCurve();
                //draw tiny lines with points and move out
                List<Point3d> pts = new List<Point3d>();
                for (int i = 0; i < count * 2; i++)
                {
                    pts.Add(circle.PointAtNormalizedLength(i / count * 2));
                }
                List<Curve> lines1 = new List<Curve>();
                List<Curve> lines2 = new List<Curve>();
                for (int i = 0; i < count; i++)
                {
                    lines1.Add(new Line(pts[i * 2], pts[i * 2 + 1]).ToNurbsCurve());
                }
                for (int i = 0; i < count - 1; i++)
                {
                    lines2.Add(new Line(pts[i * 2 + 1], pts[i * 2 + 2]).ToNurbsCurve());
                }
                lines2.Add(new Line(pts[count * 2 - 1], pts[0]).ToNurbsCurve());

                List<Point3d> outerPoints = new List<Point3d>();
                List<Point3d> innerPoints = new List<Point3d>();
                for (int i = 0; i < count; i++)
                {
                    Point3d middlePoint = lines1[i].PointAtNormalizedLength(0.5);
                    Vector3d v = new Vector3d(middlePoint) - new Vector3d(center);
                    v.Unitize();
                    v = v * amplitude;
                    Transform move = Transform.Translation(v);
                    lines1[i].Transform(move);
                    Point3d p1 = lines1[i].PointAtNormalizedLength(0.5 - factorA / 2);
                    Point3d p2 = lines1[i].PointAtNormalizedLength(0.5 + factorA / 2);
                    outerPoints.Add(p1);
                    outerPoints.Add(p2);
                }
                for (int i = 0; i < count; i++)
                {
                    Point3d p1 = lines2[i].PointAtNormalizedLength(0.5 - factorB / 2);
                    Point3d p2 = lines2[i].PointAtNormalizedLength(0.5 + factorB / 2);
                    innerPoints.Add(p1);
                    innerPoints.Add(p2);
                }
                //link all line segments and generate surface
                List<Curve> lineSegments = new List<Curve>();
                /*List<Curve> lineSegments2 = new List<Curve>();
                List<Curve> lineSegments3 = new List<Curve>();
                List<Curve> lineSegments4 = new List<Curve>();*/
                for (int i = 0; i < count - 1; i++)
                {
                    lineSegments.Add(new Line(outerPoints[i * 2], outerPoints[i * 2 + 1]).ToNurbsCurve());
                    lineSegments.Add(new Line(outerPoints[i * 2 + 1], innerPoints[i * 2]).ToNurbsCurve());
                    lineSegments.Add(new Line(innerPoints[i * 2], innerPoints[i * 2 + 1]).ToNurbsCurve());
                    lineSegments.Add(new Line(innerPoints[i * 2 + 1], outerPoints[i * 2 + 2]).ToNurbsCurve());
                }
                lineSegments.Add(new Line(outerPoints[count * 2 - 2], outerPoints[count * 2 - 1]).ToNurbsCurve());
                lineSegments.Add(new Line(outerPoints[count * 2 - 1], innerPoints[count * 2 - 2]).ToNurbsCurve());
                lineSegments.Add(new Line(innerPoints[count * 2 - 2], innerPoints[count * 2 - 1]).ToNurbsCurve());
                lineSegments.Add(new Line(innerPoints[count * 2 - 1], outerPoints[0]).ToNurbsCurve());
                outline = PolyCurve.JoinCurves(lineSegments)[0];

                Brep s = Brep.CreatePlanarBreps(outline, 0.000001)[0];
                return s;
            }

            private Brep GenerateGear()
            {
                surface = GenerateGearSurface();
                //Extrude
                Point3d endPoint = center + direction / direction.Length * thickness;
                Curve rail = new Line(center, endPoint).ToNurbsCurve();
                gear = Brep.CreateFromSweep(rail, outline, true, 0.00000001)[0];

                return gear;
            }

            public Brep GetGearSurface()
            {
                return surface;
            }

            public Brep GetGearBrep()
            {
                return gear;
            }

            public void Generate()
            {
                gear = GenerateGear();
            }
            public Brep SetPosition(Point3d Position, Vector3d Direction)
            {
                center = Position;
                direction = Direction;
                gear = GenerateGear();
                return gear;
            }

            public Brep SetParameter(double Radius = 0, double Amplitude = -1, double A = -1, double B = -1, int Count = 0, double Thickness = 0)
            {
                if (Radius > 0)
                { radius = Radius; }
                if (Amplitude > 1)
                { amplitude = Amplitude; }
                if (A >= 0 && A <= 1)
                { factorA = A; }
                if (B >= 0 && B <= 1)
                { factorB = B; }
                if (Count > 0)
                { count = Count; }
                if (Thickness > 0)
                { thickness = Thickness; }
                gear = GenerateGear();
                return gear;
            }
            public Brep ResetPosition()
            {

                center = Point3d.Unset;
                direction = Vector3d.Unset;
                gear = GenerateGear();
                return gear;
            }
            public Brep ResetParameter()
            {
                radius = 0;
                amplitude = 0;
                factorA = -1;
                factorB = -1;
                count = 0;
                thickness = 0;
                gear = GenerateGear();
                return gear;
            }
            public Brep Rotate(double angle)
            {
                gear.Rotate(angle, direction, center);
                return gear;
            }

        }
    }
}
