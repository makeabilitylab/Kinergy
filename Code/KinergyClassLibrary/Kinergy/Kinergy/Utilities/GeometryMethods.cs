using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Rhino.Geometry.Intersect;

namespace Kinergy.Utilities
{
    public class GeometryMethods
    {
        /// <summary>
        /// Construct a parallelogram with center, two directions and two dimension intervals
        /// </summary>
        /// <param name="center"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="xdim"></param>
        /// <param name="ydim"></param>
        /// <returns></returns>
        public static Curve Parallelogram(Point3d center, Vector3d x,Vector3d y,Interval xdim, Interval ydim)
        {
            Point3d p1 = center + x / x.Length * xdim.Min+ y / y.Length * ydim.Max;
            Point3d p2 = center + x / x.Length * xdim.Min+ y / y.Length * ydim.Min;
            Point3d p3 = center + x / x.Length * xdim.Max+ y / y.Length * ydim.Min;
            Point3d p4 = center + x / x.Length * xdim.Max+ y / y.Length * ydim.Max;
            List<Point3d> pts = new List<Point3d>();
            pts.Add(p1); pts.Add(p2); pts.Add(p3); pts.Add(p4); pts.Add(p1);
            Polyline l = new Polyline(pts);
            Curve c = l.ToNurbsCurve();
            c.MakeClosed(0.001);

            return c;
        }
        /// <summary>
        /// Construct a rectangle curve with center, normal and x.
        /// </summary>
        /// <param name="center"></param>
        /// <param name="normal"></param>
        /// <param name="x"></param>
        /// <param name="xdim"></param>
        /// <param name="ydim"></param>
        /// <returns></returns>
        public static Curve Rectangle(Point3d center, Vector3d normal, Vector3d x,Interval xdim, Interval ydim)
        {
            Vector3d y = new Plane(center, normal, x).Normal;
            Point3d p1 = center + x / x.Length * xdim.Min + y / y.Length * ydim.Max;
            Point3d p2 = center + x / x.Length * xdim.Min + y / y.Length * ydim.Min;
            Point3d p3 = center + x / x.Length * xdim.Max + y / y.Length * ydim.Min;
            Point3d p4 = center + x / x.Length * xdim.Max + y / y.Length * ydim.Max;
            List<Point3d> pts = new List<Point3d>();
            pts.Add(p1); pts.Add(p2); pts.Add(p3); pts.Add(p4); pts.Add(p1);
            Polyline l = new Polyline(pts);
            Curve c = l.ToNurbsCurve();
            c.MakeClosed(0.001);
            return c;
        }
        public static Curve Capsule(Plane basePlane,double radius,double length,Vector3d mainDirection)
        {
            mainDirection.Unitize();
            mainDirection = mainDirection - mainDirection * basePlane.Normal*basePlane.Normal;
            mainDirection.Unitize();
            //A capsule is formed with 2 semicircles and 2 lines
            Circle c1 = new Circle(basePlane, radius);
            Circle c2 = new Circle(basePlane, radius);
            Transform t = Transform.Translation(mainDirection * length);
            Vector3d x = mainDirection;
            Vector3d y = new Plane(basePlane.Origin, basePlane.Normal, x).Normal;
            Point3d cornerA = basePlane.Origin + y * radius / 2;
            Point3d cornerB = basePlane.Origin - y * radius / 2 + x * length;
            Rectangle3d s = new Rectangle3d(basePlane, cornerA, cornerB);
            Curve square= s.ToNurbsCurve();
            List<Curve> crvs = new List<Curve>();
            crvs.Add(c1.ToNurbsCurve()); crvs.Add(c2.ToNurbsCurve()); crvs.Add(square);
            Curve capsule = Curve.CreateBooleanUnion(crvs,RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            return capsule;
        }
        public static Vector3d AverageTangent(Curve c, double pos, double span = 0.05)
        {
            double start = Math.Min(1, pos + span);
            double end = Math.Max(0, pos - span);
            Point3d pt1 = c.PointAtNormalizedLength(start);
            Point3d pt2 = c.PointAtNormalizedLength(end);
            Vector3d tangent = new Vector3d(pt1) - new Vector3d(pt2);
            return tangent;
        }
        public static bool IfBrepsIntersect(Brep b1,Brep b2)
        {
            Curve[] crvs;
            Point3d[] pts;
            Intersection.BrepBrep(b1, b2, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out crvs, out pts);
            return !(crvs.Length == 0) && (pts.Length == 0);
        }
        public static Point3d ComputeCurveCentroid(Curve crv)
        {
            Point3d p = Point3d.Origin;
            for(double i=0;i<=1;i+=0.01)
            {
                p += new Vector3d(crv.PointAtNormalizedLength(i));
            }
            p /= 101;
            return p;
        }
        public static double CurveDistanceToPoint(Curve c,Point3d p)
        {
            double t = 0;
            c.ClosestPoint(p,out t);
            Point3d p1 = c.PointAt(t);
            return p1.DistanceTo(p);
        }
    }
}
