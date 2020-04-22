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
        /// <summary>
        /// Defines a box-like brep.Include convenient checking method and geometric methods.
        /// </summary>
        public class BoxLike:Entity
        {
            
            private Vector3d direction = Vector3d.Unset;
            private bool validity;
            private Brep innerEmptySpaceBoxBrep=null;
            private BoundingBox innerEmptySpaceBbox;
            private Point3d centerPoint;

            public Vector3d Direction { get => direction; private set => direction = value; }
            public bool Validity { get => validity; private set => validity = value; }
            public Brep InnerEmptySpaceBoxBrep { get => innerEmptySpaceBoxBrep;private set => innerEmptySpaceBoxBrep = value; }
            public BoundingBox InnerEmptySpaceBbox { get => innerEmptySpaceBbox;private set => innerEmptySpaceBbox = value; }

            /// <summary>
            /// Static function checking if a brep model is a legal box-like shape
            /// </summary>
            public static bool CheckBox(Brep model, Vector3d Direction)
            {
                //TODO
                //First check continuity

                //Then cut model to check shape
                
                return true;
            }
            public BoxLike(Brep M,bool IsStatic=false):base(M,IsStatic)
            {
                model = M;
                validity = CheckBox(Model, Direction);
                bbox = model.GetBoundingBox(true);
                centerPoint = bbox.Center;
                validity = CheckBox(M,Vector3d.XAxis);
            }
            public List<Vector3d> GetSurroundingDirections(Vector3d direction)
            {
                List<Vector3d> DirectionCandidates = new List<Vector3d> 
                { new Vector3d(1, 0, 0) , new Vector3d(-1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(0, -1, 0),new Vector3d(0, 0, 1),new Vector3d(0, 0, -1) };
                List<Vector3d> Candidates = new List<Vector3d>();
                foreach(Vector3d v in DirectionCandidates)
                {
                    if(Math.Abs(v*direction/direction.Length)<0.9)
                    {
                        Candidates.Add(v);
                        continue;
                    }
                }
                return Candidates;
            }
            public List<Point3d> GetPointCandidatesByDirection(Vector3d d,BoundingBox b, double offset=10)
            {
                double x = 0, y = 0, z = 0;
                List<Vector3d> DirectionCandidates = new List<Vector3d>();
                DirectionCandidates.Add(new Vector3d(1, 0, 0));
                DirectionCandidates.Add(new Vector3d(-1, 0, 0));
                DirectionCandidates.Add(new Vector3d(0, 1, 0));
                DirectionCandidates.Add(new Vector3d(0, -1, 0));
                DirectionCandidates.Add(new Vector3d(0, 0, 1));
                DirectionCandidates.Add(new Vector3d(0, 0, -1));
                foreach (Vector3d v in DirectionCandidates)
                {
                    if (v * d / d.Length > 0.8)
                    {
                        x= v.X;
                        y = v.Y;
                        z = v.Z;
                        break;
                    }
                }
                double X, Y, Z;
                X = b.Max.X-b.Min.X;
                Y = b.Max.Y - b.Min.Y;
                Z = b.Max.Z - b.Min.Z;
                List<Point3d> Candidates = new List<Point3d>();

                if (x==1)//right face
                {
                    if(offset*2>=Y || offset * 2 >= Z)
                    {  }
                    else 
                    {
                        for(double i=offset/Y;i<1-offset/Y;i+=(Y-offset*2)/Y/(Math.Round((Y-offset*2)/2)))
                        {
                            for (double j = offset / Z; j < 1 - offset / Z; j += (Z - offset * 2)/Z / (Math.Round((Z - offset * 2) / 2)))
                            {
                                Candidates.Add(b.PointAt(1, i, j));
                            }
                        }
                    }
                    
                }
                else if(x==-1)//left face
                {
                    if (offset * 2 >= Y || offset * 2 >= Z)
                    { }
                    else
                    {
                        for (double i = offset / Y; i < 1 - offset / Y; i += (Y - offset * 2)/Y / (Math.Round((Y - offset * 2) / 2)))
                        {
                            for (double j = offset / Z; j < 1 - offset / Z; j += (Z - offset * 2)/Z / (Math.Round((Z - offset * 2) / 2)))
                            {
                                Candidates.Add(b.PointAt(0, i, j));
                            }
                        }
                    }
                }
                else if (y== 1)//backward
                {
                    if (offset * 2 >= X || offset * 2 >= Z)
                    { }
                    else
                    {
                        for (double i = offset / X; i < 1 - offset / X; i += (X - offset * 2)/X / (Math.Round((X - offset * 2) / 2)))
                        {
                            for (double j = offset / Z; j < 1 - offset / Z; j += (Z - offset * 2)/Z / (Math.Round((Z - offset * 2) / 2)))
                            {
                                Candidates.Add(b.PointAt(i, 1, j));
                            }
                        }
                    }
                }
                else if (y == -1)//forward
                {
                    if (offset * 2 >= X || offset * 2 >= Z)
                    { }
                    else
                    {
                        for (double i = offset / X; i < 1 - offset / X; i += (X - offset * 2)/X / (Math.Round((X - offset * 2) / 2)))
                        {
                            for (double j = offset / Z; j < 1 - offset / Z; j += (Z - offset * 2)/Z / (Math.Round((Z - offset * 2) / 2)))
                            {
                                Candidates.Add(b.PointAt(i, 0, j));
                            }
                        }
                    }
                }
                else if (z == 1)//upward
                {
                    if (offset * 2 >= X || offset * 2 >= Y)
                    { }
                    else
                    {
                        for (double i = offset / X; i < 1 - offset / X; i += (X - offset * 2) /X/ (Math.Round((X - offset * 2) / 2)))
                        {
                            for (double j = offset / Y; j < 1 - offset / Y; j += (Y - offset * 2)/Y / (Math.Round((Y - offset * 2) / 2)))
                            {
                                Candidates.Add(b.PointAt(i, j, 1));
                            }
                        }
                    }
                }
                else if (z == -1)//downward
                {
                    if (offset * 2 >= X || offset * 2 >= Y)
                    { }
                    else
                    {
                        for (double i = offset / X; i < 1 - offset / X; i += (X - offset * 2)/X / (Math.Round((X - offset * 2) / 2)))
                        {
                            for (double j = offset / Y; j < 1 - offset / Y; j += (Y - offset * 2)/Y / (Math.Round((Y - offset * 2) / 2)))
                            {
                                Candidates.Add(b.PointAt(i, j, 0));
                            }
                        }
                    }
                }
                return Candidates;
            }
            public double SpanAtDirection(Vector3d v)
            {
                if(Math.Abs(Vector3d.XAxis*v/v.Length)>0.9)
                { return bbox.Max.X - bbox.Min.X; }
                else if (Math.Abs(Vector3d.YAxis * v / v.Length) > 0.9)
                { return bbox.Max.Y - bbox.Min.Y; }
                else if (Math.Abs(Vector3d.ZAxis * v / v.Length) > 0.9)
                { return bbox.Max.Z - bbox.Min.Z; }
                else { return 0; }
            }
            public static double SpanAtDirection(Vector3d v,BoundingBox bbox)
            {
                if (Math.Abs(Vector3d.XAxis * v / v.Length) > 0.9)
                { return bbox.Max.X - bbox.Min.X; }
                else if (Math.Abs(Vector3d.YAxis * v / v.Length) > 0.9)
                { return bbox.Max.Y - bbox.Min.Y; }
                else if (Math.Abs(Vector3d.ZAxis * v / v.Length) > 0.9)
                { return bbox.Max.Z - bbox.Min.Z; }
                else { return 0; }
            }
            public static Point3d GetInnerPointBySurfacePoint(Point3d p,BoundingBox b,double relativePosition=0.5)
            {
                Point3d p1 = b.ClosestPoint(p,false);
                Point3d p2 = GetOppositePoint(p1, b);
                Vector3d d = new Vector3d(p2) - new Vector3d(p1);
                d = -Vector3d.ZAxis;
                d.Unitize();
                Point3d p3 = p + relativePosition * d*SpanAtDirection(d,b);//TODO
                
                return p3;
            }
            public static Point3d GetOppositePoint(Point3d p,BoundingBox b)
            {
                Point3d p1 = b.ClosestPoint(p, false);
                Point3d center = b.Center;
                Point3d p2;
                if(p1.X==b.Max.X || p1.X == b.Min.X)
                {
                    Vector3d v = new Vector3d(center) - new Vector3d(p1);
                    p2 = p1 + 2 * v - 2 * (Vector3d.XAxis * v * Vector3d.XAxis);
                }
                else if(p1.Y == b.Max.Y || p1.Y == b.Min.Y)
                {
                    Vector3d v = new Vector3d(center) - new Vector3d(p1);
                    p2 = p1 + 2 * v - 2 * (Vector3d.YAxis * v * Vector3d.YAxis);
                }
                else
                {
                    Vector3d v = new Vector3d(center) - new Vector3d(p1);
                    p2 = p1 + 2 * v - 2 * (Vector3d.ZAxis * v * Vector3d.ZAxis);
                }
                
                return p2;
            }
            /// <summary>
            /// Calculate the maximum inner space box with newton method. Note that the spanning iteration starts from center.
            /// </summary>
            /// <returns></returns>
            public bool GetInnerEmptySpaceBox(double precision=0.01)
            {
                List<double> steps = new List<double>();
                List<double> scales = new List<double>();
                Curve[] crv;
                for (int i = 0; i < 6; i++) { steps.Add(1);scales.Add(0.001); }
                do
                {
                    for(int i=0;i<6;i++)
                    {
                        if(steps[i]<precision)
                        { continue; }
                        else 
                        {
                            int counter = 0;
                            do
                            {
                                steps[i] *= 0.5;
                                if (counter == 0)
                                { 
                                    scales[i] += steps[i] ; 
                                }
                                else
                                { 
                                    scales[i] -= steps[i];
                                }
                                if(scales[i]>1)
                                { 
                                    scales[i] = 1; 
                                }
                                counter++;
                                string body = string.Format("The inner empty space box searching method run at {0},this scale is{1}",i,scales[i]);
                                RhinoApp.WriteLine(body);
                                if (scales.Min()<0)
                                { return false; }
                                Rhino.Geometry.Intersect.Intersection.BrepBrep(model, ConstructCenterBox(scales[0], scales[1], scales[2], scales[3], scales[4], scales[5]),
                            precision, out crv, out _);
                            } while ( crv.Count()>0 && steps[i]>=precision);
                        }
                    }
                    
                } while (steps.Max()>precision);
                if(scales.Min()<=precision)
                { return false; }
                InnerEmptySpaceBoxBrep=ConstructCenterBox(scales[0], scales[1], scales[2], scales[3], scales[4], scales[5]);
                InnerEmptySpaceBbox = InnerEmptySpaceBoxBrep.GetBoundingBox(true);
                return true;
                
            }
            private Brep ConstructCenterBox(double xp,double xm,double yp,double ym,double zp,double zm)
            {
                double X, Y, Z;
                Point3d p = bbox.PointAt(0.5,0.5,0.5);
                X = bbox.Max.X - bbox.Min.X;
                Y = bbox.Max.Y - bbox.Min.Y;
                Z = bbox.Max.Z - bbox.Min.Z;
                Interval xInterval = new Interval(- xm * X / 2, + xp * X / 2);
                Interval yInterval = new Interval(- ym * Y / 2, + yp * Y / 2);
                Interval zInterval = new Interval(- zm * Z / 2, + zp * Z / 2);
                Box newBox = new Box(new Plane(p, Vector3d.ZAxis), xInterval, yInterval, zInterval);
                return newBox.ToBrep();
            }
            
            public override bool CreateRoundHoleOnSurface(Point3d p, double r)
            {
                Point3d p1 = bbox.ClosestPoint(p);
                Point3d p2 = innerEmptySpaceBbox.ClosestPoint(p);
                if(p1==Point3d.Unset || p2==Point3d.Unset)
                { return false; }
                Vector3d v = new Vector3d(p2) - new Vector3d(p1);
                Plane plane = new Plane(p1, v);
                Circle baseCircle = new Circle(plane, r);
                Cylinder c = new Cylinder(baseCircle, new Line(p1, p2).Length);
                if(c.ToBrep(true, true) == null)
                { return false; }
                model = Brep.CreateBooleanDifference(model, c.ToBrep(true,true), 0.000001)[0];
                return true;
            }
            public override bool CreateRectangularHoleOnSurface(Point3d center, Vector3d xDirection, Vector3d yDirection, double x, double y)
            {
                Point3d p1 = bbox.ClosestPoint(center,false);
                
                Point3d p2 = innerEmptySpaceBbox.ClosestPoint(center,false);
                if (p1 == Point3d.Unset || p2 == Point3d.Unset)
                { return false; }
                Vector3d v = new Vector3d(p2) - new Vector3d(p1);
                Plane plane = new Plane(p1, xDirection,yDirection);
                Box box = new Rhino.Geometry.Box(plane, new Interval(-x / 2,x / 2),
                           new Interval(-y / 2, y / 2),
                           new Interval(0, new Line(p1, p2).Length));
                if(box.ToBrep()==null)
                { return false; }
                model = Brep.CreateBooleanDifference(model, box.ToBrep(), 0.000001)[0];
                return true;
            }
        } 
    }
}
