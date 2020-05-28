using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel.Components;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Kinergy
{
    namespace Geom
    { 
        public class RodLike:Entity
        {
            Curve skeleton;
            Vector3d direction;
            public Curve Skeleton { get => skeleton;private set => skeleton = value; }
            public Vector3d Direction { get => direction;private set => direction = value; }

            public RodLike(Brep brep, bool stat = false, string n = "") : base(brep, stat, n)
            {
                CalculateSkeleton();
            }
            private void CalculateSkeleton()
            {
                //Currently just use bounding box to calculate skeleton. In this case, the brep model should be in orthogonal direction
                double x=Bbox.Max.X - Bbox.Min.X;
                double y= Bbox.Max.Y - Bbox.Min.Y;
                double z = Bbox.Max.Z - Bbox.Min.Z;
                Point3d start, end;
                if(x>=y && x>z)
                {
                    start = new Point3d(Bbox.Min.X, Bbox.Max.Y / 2 + Bbox.Min.Y / 2, Bbox.Max.Z / 2 + Bbox.Min.Z / 2);
                    end= new Point3d(Bbox.Max.X, Bbox.Max.Y / 2 + Bbox.Min.Y / 2, Bbox.Max.Z / 2 + Bbox.Min.Z / 2);
                    skeleton = new Line(start, end).ToNurbsCurve();
                    direction = new Vector3d(end) - new Vector3d(start);
                }
                else if(y>=x && y>=z)
                {
                    start = new Point3d(Bbox.Min.X / 2 + Bbox.Max.X / 2, Bbox.Min.Y, Bbox.Max.Z / 2 + Bbox.Min.Z / 2);
                    end = new Point3d(Bbox.Min.X / 2 + Bbox.Max.X / 2, Bbox.Max.Y , Bbox.Max.Z / 2 + Bbox.Min.Z / 2);
                    skeleton = new Line(start, end).ToNurbsCurve();
                    direction = new Vector3d(end) - new Vector3d(start);
                }
                else
                {
                    start = new Point3d(Bbox.Min.X / 2 + Bbox.Max.X / 2, Bbox.Max.Y / 2 + Bbox.Min.Y / 2,  Bbox.Min.Z);
                    end = new Point3d(Bbox.Min.X / 2 + Bbox.Max.X / 2, Bbox.Max.Y / 2 + Bbox.Min.Y / 2, Bbox.Max.Z);
                    skeleton = new Line(start, end).ToNurbsCurve();
                    direction = new Vector3d(end) - new Vector3d(start);
                }
            }
            
        }
    }
}
