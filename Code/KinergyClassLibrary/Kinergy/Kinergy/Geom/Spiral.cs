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
            private int roundNum = 0;
            private double innerRadius = 0, outerRadius = 0;
            private double thicknessX = 0, thicknessY = 0;
            private Point3d centerPoint = Point3d.Unset;
            private Vector3d direction = Vector3d.Unset;
            private Curve spiralCurve = null;
            private Brep spiralBrep = null;
            private int pointsPerRound = 20;

            public int PointsPerRound { get => pointsPerRound;private set => pointsPerRound = value; }
            public Point3d CenterPoint { get => centerPoint;private set => centerPoint = value; }

            /// <summary> Default constructor without any input parameter </summary>
            /// <returns> Returns empty instance</returns>


            /// <summary> Constructor with center point and direction given </summary>
            /// <returns> Returns instance with spiral brep generated</returns>
            public Spiral(Point3d Center, Vector3d Direction, double R = 0, double r = 0, double ThicknessX = 0, double ThicknessY = 0, int RoundNum = 0)
            {
                center = Center;
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
                Generate();

            }

            /// <summary> Constructor with parameter but no center point given </summary>
            /// <returns> Returns instance with gear brep generated</returns>
            public Spiral(double R = 0, double r = 0, double ThicknessX = 0, double ThicknessY = 0, int RoundNum = 0)
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
                Generate();
            }
            private void FixParameter()
            {
                if (center == Point3d.Unset)
                {
                    center = new Point3d(0, 0, 0);
                }
                if (direction == Vector3d.Unset)
                {
                    direction = new Vector3d(0, 0, 1);
                }
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
                    innerRadius = outerRadius / 5;
                }
                if (thicknessX > (outerRadius - innerRadius) / roundNum)
                {
                    thicknessX = (outerRadius - innerRadius) / roundNum / 2;
                }
            }
            private void GenerateSpiralCurve()
            {
                FixParameter();
                int numPoints = pointsPerRound * roundNum + 1;
                double PI = Math.PI;
                Plane basePlane = new Plane(center, direction);
                Vector3d X = basePlane.XAxis;
                Vector3d Y = basePlane.YAxis;
                List<Point3d> pts = new List<Point3d>();
                for (int i = 0; i < numPoints; i++)
                {
                    pts.Add(center + (X * Math.Cos(i * PI * 2 / pointsPerRound) + Y * Math.Sin(i * PI * 2 / pointsPerRound)) * (innerRadius + (outerRadius - innerRadius) * (i / roundNum / pointsPerRound)));
                }
                Curve s = Rhino.Geometry.Curve.CreateInterpolatedCurve(pts, 3);
                base.BaseCurve = s;
            }
            private void GenerateSpiralBrep()
            {
                
                Plane basePlane = new Plane(center, direction);
                Vector3d X = basePlane.XAxis;
                Plane recPlane = new Plane(center + X * innerRadius, X, direction);
                Rectangle3d outline = new Rectangle3d(basePlane, new Interval(-thicknessX * 0.5, thicknessX * 0.5), new Interval(0, thicknessY));

                Brep b = Brep.CreateFromSweep(base.BaseCurve, outline.ToNurbsCurve(), true, 0.00000001)[0];
                base.Model = b;

            }
            public override void Generate()
            {
                GenerateSpiralBrep();
            }
            
            public void SetPosition(Point3d Center, Vector3d Direction)
            {
                center = Center;
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
            public void ResetParameter()
            {
                roundNum = 0;
                innerRadius = 0;
                outerRadius = 0;
                thicknessX = 0;
                thicknessY = 0;

                GenerateSpiralBrep();
            }
            public void ResetPosition()
            {
                center = Point3d.Unset;
                direction = Vector3d.Unset;
                GenerateSpiralBrep();
            }
            public void Rotate(double angle)
            {
                spiralBrep.Rotate(angle, direction, center);
                
            }
            public override bool Move(Movement move)
            {
                //TODO Do the mobility check
                return false;
            }
            protected override void ConductMoveAndUpdateParam(Movement move)
            {
                //TODO Do the rotating
            }
        }
    }
}
