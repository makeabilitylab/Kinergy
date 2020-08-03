using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel.Components;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino;

namespace Kinergy
{
    namespace Geom
    { 
        public class RodLike:Entity
        {
            Cylinder rodCylinder;
            Curve skeleton;
            Vector3d direction;
            public Curve Skeleton { get => skeleton;private set => skeleton = value; }
            public Vector3d Direction { get => direction;private set => direction = value; }
            public Cylinder RodCylinder { get => rodCylinder;private  set => rodCylinder = value; }

            public RodLike(Brep brep,Vector3d rodDirection, bool stat = false, string n = "") : base(brep, stat, n)
            {
                direction = rodDirection;
                model = brep.DuplicateBrep();
                CalculateSkeleton();
                rodCylinder = Cylinder.Unset;
            }
            public RodLike(Brep brep,Vector3d rodDirection,Curve givenSkeleton, bool stat = false, string n = "") : base(brep, stat, n)
            {
                direction = rodDirection;
                model = brep.DuplicateBrep();
                skeleton = givenSkeleton;
                rodCylinder = Cylinder.Unset;
            }
            public RodLike(Point3d center, double radius,Vector3d rodDirection, Interval range, bool stat = false, string n = "") : base(null, stat, n)
            {
                direction = rodDirection;
                Circle c = new Circle(new Plane(center, direction), radius);
                c.Transform(Transform.Translation(direction / direction.Length * range.Min));
                rodCylinder = new Cylinder(c, range.Length);
                model = rodCylinder.ToBrep(true, true);
                skeleton = new Line(rodCylinder.Center, rodCylinder.Center + rodCylinder.Axis / rodCylinder.Axis.Length * rodCylinder.TotalHeight).ToNurbsCurve();
            }
            public void AddKnob(double t,double radius,double height)
            {
                if (t < 0 || t > 1)
                    throw new Exception("Invalid t value given");
                Plane p1 = new Plane(skeleton.PointAtNormalizedLength(t), direction);
                Vector3d x = p1.XAxis;
                Plane p2 = new Plane(skeleton.PointAtNormalizedLength(t), x);
                Circle c = new Circle(p2, radius);
                c.Transform(Transform.Translation(-x * height / 2));
                Cylinder knob = new Cylinder(c, height);
                model = Brep.CreateBooleanUnion(new List<Brep> { model, knob.ToBrep(true, true) }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
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
