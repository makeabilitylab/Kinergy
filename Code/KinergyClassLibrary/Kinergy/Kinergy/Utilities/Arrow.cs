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



namespace Kinergy.Utilities
{
    
    public class Arrow
        {
            /// <summary> Default arrow builder with scale input </summary>
            /// <returns> Returns arrow as curve</returns>
        public static Curve ConstructArrow(double scale=1)
            {
                
                Point3d p1 = new Point3d(0, 0, 0), p2 = new Point3d(20*scale, 0, 0), p3 = new Point3d(20*scale, 4*scale, 0), p4 = new Point3d(28*scale, 0, 0),
                        p5 = new Point3d(20*scale, -4*scale, 0), p6 = new Point3d(20*scale, 0, 0);
                List<Point3d> pts = new List<Point3d>();
                pts.Add(p1); pts.Add(p2); pts.Add(p3); pts.Add(p4); pts.Add(p5); pts.Add(p6);
                Polyline l = new Polyline(pts);
                Curve arrow = l.ToNurbsCurve();
                
                return arrow;
            }
        private Vector3d direction = Vector3d.Unset;
        private Point3d startPosition = Point3d.Unset;
        private Curve arrowCurve = null;
        public Vector3d Direction { get => direction;private set => direction = value; }
        public Point3d StartPosition { get => startPosition;private set => startPosition = value; }
        public Curve ArrowCurve { get => arrowCurve;private set => arrowCurve = value; }

        public Arrow(Vector3d D,Point3d P,double scale=1)
        {

            direction = D;
            startPosition = P;
            arrowCurve = ConstructArrow(scale);
            arrowCurve.Transform(Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin));
            arrowCurve.Transform(Transform.Translation(new Vector3d(P)));
        }
        
    }
    
}
