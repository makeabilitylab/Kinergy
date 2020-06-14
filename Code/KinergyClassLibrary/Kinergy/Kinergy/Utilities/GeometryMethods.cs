using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
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
        public static Vector3d AverageTangent(Curve c, double pos, double span = 0.05)
        {
            double start = Math.Min(1, pos + span);
            double end = Math.Max(0, pos - span);
            Point3d pt1 = c.PointAtNormalizedLength(start);
            Point3d pt2 = c.PointAtNormalizedLength(end);
            Vector3d tangent = new Vector3d(pt1) - new Vector3d(pt2);
            return tangent;
        }
    }
}
