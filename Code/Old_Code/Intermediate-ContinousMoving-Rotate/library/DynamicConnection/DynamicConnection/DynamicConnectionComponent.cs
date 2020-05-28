using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace DynamicConnection
{
    public class DynamicConnectionComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public DynamicConnectionComponent()
          : base("DynamicConnection", "DynamicConnection",
              "Create a dynamic connection between black box and axises",
              "Brep", "Kinetic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "axis", "Axises to connect with Blackbox", GH_ParamAccess.item);
            pManager.AddBrepParameter("Brep", "gear", "The Gear the axis connects with from the Reload Button", GH_ParamAccess.item);
            pManager.AddBrepParameter("Brep", "bp", "The brep to place the gears in", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Speed", "speed", "The speed the user wants", GH_ParamAccess.item);
            pManager.AddVectorParameter("Vector3d", "vec", "Direction of Movement", GH_ParamAccess.item);
            pManager.AddGenericParameter("double", "initialVelocity", "Initial angle velocity of gear", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "gears", "gears between axis and main gears", GH_ParamAccess.list);
            pManager.AddBrepParameter("Brep", "axises", "axises to connect gears with brep", GH_ParamAccess.list);
            pManager.AddGenericParameter("double", "angleVelocity", "angle velocities of gears", GH_ParamAccess.list);
            //pManager.AddBooleanParameter("bool", "hasSteer", "whether the gears have steer", GH_ParamAccess.item);
            pManager.AddBrepParameter("Brep", "steer", "steer", GH_ParamAccess.list);
            pManager.AddVectorParameter("Vector3d", "finalGearNormal", "the last normal of the gear", GH_ParamAccess.item);
        }

        private Brep CreateAxis(Point3d bottomcenter, double radius, Plane circlePlane, Vector3d direction)
        {
            //Add an knob shape to the box
            Curve c = new Circle(circlePlane, bottomcenter, radius).ToNurbsCurve();
            Brep brep = Brep.CreatePlanarBreps(c, Rhino.RhinoMath.SqrtEpsilon)[0];
            BrepFace surface = brep.Faces[0];
            Curve GearThickness = new Line(c.PointAtStart, direction).ToNurbsCurve();
            Brep Cylinder = surface.CreateExtrusion(GearThickness, true);
            return Cylinder;
        }

        private Brep CreateGearAxis(Point3d MidCenter,Vector3d GearNormal, Plane GearPlane, Brep bp,double length)
        {

            double AxisRadius = 2;
            Curve l = new Line(MidCenter, GearNormal, length * 2).ToNurbsCurve();
            var xf = Transform.Translation(-length * GearNormal);
            l.Transform(xf);
            double[] t;
            Rhino.Geometry.Intersect.Intersection.CurveBrep(l, bp, RhinoMath.SqrtEpsilon, RhinoMath.DefaultAngleTolerance, out t);
            Point3d p1 = l.PointAt(t[0]);
            Point3d p2 = l.PointAt(t[1]);
            return CreateAxis(p1, AxisRadius, GearPlane, p2 - p1);           
        }

        private List<Line> createLines(Point3d[] ps, double factor)
        {
            List<Line> lines = new List<Line>();
            int i =0;
            for (i = 0; i < ps.Length - 1; i+=2)
            {
                Line l = new Line(ps[i], ps[i + 1]);
                var xf = Transform.Scale(l.PointAt(0.5), factor);
                l.Transform(xf);
                lines.Add(l);
            }
            return lines;
        }

        private List<Line> moveLines(List<Line> lines,Point3d Center, double teethHeight, double factor)
        {
            List<Line> linesNew = new List<Line>();
            foreach(Line l in lines)
            {
                Vector3d v = l.PointAt(0.5)-Center;
                var xf = Transform.Translation(teethHeight/v.Length*v);
                l.Transform(xf);
                var xs= Transform.Scale(l.PointAt(0.5), factor);
                l.Transform(xs);
                linesNew.Add(l);
            }
            return linesNew;
        }

        private Brep CreateGear(Plane GearPlane, Point3d MidCenter, double r, int divide, double teethHeight,double Thickness)
        {
            double centerRadius = 3;
            Curve c = new Circle(GearPlane, MidCenter, r).ToNurbsCurve();
            Curve center = new Circle(GearPlane, MidCenter, centerRadius).ToNurbsCurve();
            Point3d[] points1;
            c.DivideByCount(divide, true, out points1);
            List<Line> lines1 = createLines(points1, 1.0);
            List<Line> lines2 = moveLines(lines1, MidCenter, teethHeight, 0.5);
            List<Curve> GearCurve = new List<Curve>();
            for (int i = 0; i < lines1.Count; i++)
            {
                GearCurve.Add(new Line(lines1[i].From, lines2[i].From).ToNurbsCurve());
                GearCurve.Add(lines2[i].ToNurbsCurve());
                GearCurve.Add(new Line(lines2[i].To, lines1[i].To).ToNurbsCurve());
                if (i < lines1.Count - 1)
                    GearCurve.Add(new Line(lines1[i].To, lines1[i + 1].From).ToNurbsCurve());
                else
                    GearCurve.Add(new Line(lines1[i].To, lines1[0].From).ToNurbsCurve());
            }
            Curve curve= Curve.JoinCurves(GearCurve)[0];
            Curve[] curves = { curve, center };
            Brep brep = Brep.CreatePlanarBreps(curves, RhinoMath.SqrtEpsilon)[0];
            BrepFace surface = brep.Faces[0];
            Curve GearThickness = new Line(curve.PointAtStart, GearPlane.Normal * Thickness).ToNurbsCurve();
            Brep Gear = surface.CreateExtrusion(GearThickness, true);
            var xf = Transform.Translation((-Thickness / 2) * GearPlane.Normal);
            Gear.Transform(xf);
            return Gear;
        }

        private List<Brep> CreateSteeringGear(Circle c, double Thickness, double teethHeight)
        {
            List<Brep> SteeringGear=new List<Brep>();
            Curve cr = c.ToNurbsCurve();
            //Create a cylinder as foundation
            Brep brepCircle = Brep.CreatePlanarBreps(cr, RhinoMath.SqrtEpsilon)[0];
            BrepFace circleSurface = brepCircle.Faces[0];
            Curve circleThickness = new Line(c.Center, -c.Normal, Thickness).ToNurbsCurve();
            //Create Teeth
            double r = c.Radius;
            //int divide = (int)r * 4;
            int divide = ((int)r * 3 % 4 == 0|| (int)r * 3 % 4 == 1) ? ((int)r*3 /4)*4 : ((int)r*3/4+1) * 4;
            //int divide = ((int)r % 2 == 0) ? (int)r * 2 : ((int)r + 1) * 2;
            Point3d[] points1;
            cr.DivideByCount(divide, true, out points1);
            List<Line> lines1 = createLines(points1, 1.0);
            var xf = Transform.Translation(teethHeight * c.Normal);
            for (int i = 0; i < lines1.Count; i+=2)
            {
                Line temp_l = new Line();
                temp_l.From = lines1[i].From;
                temp_l.To = lines1[i].To;

                temp_l.Transform(xf);
                var xs = Transform.Scale(temp_l.PointAt(0.5), 0.5);
                temp_l.Transform(xs);
                Point3d[] points = {lines1[i].From, temp_l.From, temp_l.To,lines1[i].To,lines1[i].From};
                PolylineCurve curve = new PolylineCurve(points);
                Brep brep = Brep.CreatePlanarBreps(curve, RhinoMath.SqrtEpsilon)[0];
                BrepFace surface = brep.Faces[0];
                Curve GearThickness = new Line(lines1[i].PointAt(0.5),new Vector3d(c.Center- lines1[i].PointAt(0.5)),Thickness).ToNurbsCurve();
                Brep teeth = surface.CreateExtrusion(GearThickness, true);
                //var xr = Transform.Rotation(Math.Sin(angle), Math.Cos(angle), c.Normal, c.Center);
                //teeth.Transform(xr);
                SteeringGear.Add(teeth);
            }
            SteeringGear.Add(circleSurface.CreateExtrusion(circleThickness, true));
            //double tolerance=0.01;
            //Brep[] steer = Brep.CreateBooleanUnion(SteeringGear, tolerance);
            return SteeringGear;
        }        

        private double maxGearRadius(Point3d p, Curve[] brepOutLine,double max)
        {
            double radius = max;
            foreach (Curve c in brepOutLine)
            {
                double t;
                if (c.ClosestPoint(p, out t, max))
                {
                    int dist = (int)c.PointAt(t).DistanceTo(p)-2;
                    if (dist < radius)
                        radius = dist;
                }
            }
            return radius;
        }

        private Point3d NearestPointToGear(Point3d Center, Point3d AxisCenter, double bX,double bY,Vector3d AxisDirection)
        {
            Point3d start;
            Point3d end;
            if (AxisDirection==Vector3d.XAxis)
            {
                start = new Point3d(AxisCenter.X - bX / 2-5, AxisCenter.Y, AxisCenter.Z);
                end = new Point3d(AxisCenter.X + bX / 2+5, AxisCenter.Y, AxisCenter.Z);                
            }
            else
            {
                start = new Point3d(AxisCenter.X, AxisCenter.Y - bY / 2-5, AxisCenter.Z);
                end = new Point3d(AxisCenter.X , AxisCenter.Y + bY / 2+5, AxisCenter.Z);
            }
            Curve AxisLine = (new Line(start, end)).ToNurbsCurve();
            double t;
            AxisLine.ClosestPoint(Center, out t);
            return AxisLine.PointAt(t);
        }

        /*private Vector3d MoveGear(Point3d AxisCenter, Point3d Center, Vector3d GearNormal, double AxisRadius)
        {
            Vector3d MoveDirection;
            double dist;
            if (GearNormal == Vector3d.XAxis)
            {
                dist = AxisCenter.X - Center.X;
                if (dist < 0)
                    dist += AxisRadius;
                else
                    dist -= AxisRadius;
                MoveDirection = new Vector3d(dist, 0, 0);
            }
            else if(GearNormal==Vector3d.YAxis)
            {
                dist = AxisCenter.Y - Center.Y;
                if (dist < 0)
                    dist += AxisRadius;
                else
                    dist -= AxisRadius;
                MoveDirection = new Vector3d(0, dist, 0);
            }
            else
            {
                dist = AxisCenter.Z -  Center.Z;
                if (dist < 0)
                    dist += AxisRadius;
                else
                    dist -= AxisRadius;
                MoveDirection = new Vector3d(0, 0, dist);
            }
            return MoveDirection;
                
        }
        */

        private Point3d FindLineEnd(Point3d AxisCenter, Point3d Center, Vector3d GearNormal)
        {
            Point3d LineEnd;
            if(GearNormal == Vector3d.YAxis)
                LineEnd= new Point3d(AxisCenter.X, Center.Y, AxisCenter.Z);
            else
                LineEnd = new Point3d(AxisCenter.X, AxisCenter.Y, Center.Z);
            return LineEnd;
        }

        private Circle FindFinalCircle(Brep brep, Point3d AxisCenter, double bX, double bY,Vector3d AxisDirection)
        {
            BoundingBox b = brep.GetBoundingBox(true);
            Point3d start;
            Point3d end;
            Point3d ContactPoint;
            Circle c;
            if (AxisDirection==Vector3d.XAxis)
            {
                ContactPoint = new Point3d(b.Center.X - (b.Max.X - b.Min.X) / 2-3.0, b.Center.Y, b.Center.Z);
                start = new Point3d(AxisCenter.X - bX / 2, AxisCenter.Y, AxisCenter.Z);
                end = new Point3d(AxisCenter.X + bX / 2, AxisCenter.Y, AxisCenter.Z);
                Curve AxisLine = (new Line(start, end)).ToNurbsCurve();
                double t;
                AxisLine.ClosestPoint(ContactPoint, out t);
                Point3d Center=AxisLine.PointAt(t);
                double radius = Center.DistanceTo(ContactPoint);
                c = new Circle(Plane.WorldYZ, Center, radius);
            }
            else
            {
                ContactPoint = new Point3d(b.Center.X , b.Center.Y - (b.Max.Y - b.Min.Y) / 2-3.0, b.Center.Z);
                start = new Point3d(AxisCenter.X, AxisCenter.Y - bY / 2, AxisCenter.Z);
                end = new Point3d(AxisCenter.X, AxisCenter.Y + bY / 2, AxisCenter.Z);
                Curve AxisLine = (new Line(start, end)).ToNurbsCurve();
                double t;
                AxisLine.ClosestPoint(ContactPoint, out t);
                Point3d Center = AxisLine.PointAt(t);
                double radius = Center.DistanceTo(ContactPoint);
                c = new Circle(Plane.WorldZX, Center, radius);
            }
            return c;
        }

        private bool TestAxisValid(Curve[] brepOutline,Curve l,Plane GearPlane,double min)
        {
           
            foreach(Curve c in brepOutline)
            {
                var events = Rhino.Geometry.Intersect.Intersection.CurveCurve(c, l, min, 1000);
                if (events != null)
                    return false;
            }
            return true;        
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Declare a variable for the input brep and box;
            Brep Axis = null;
            Brep Gear = null;
            Brep bp = null;
            Int32 speed = 3;
            Vector3d direct = Vector3d.Unset;
            double initialVelocity = 0;
            //Use the DA object to retrieve the data inside the first input parameter.
            //If the retrieval fails we need to abort.
            if (!DA.GetData(0, ref Axis)) { return; }
            if (!DA.GetData(1, ref Gear)) { return; }
            if (!DA.GetData(2, ref bp)) { return; }
            if (!DA.GetData(3, ref speed)) { return; }
            if(!DA.GetData(4,ref direct)) { return; }
            if (!DA.GetData(5, ref initialVelocity)) { return; }
            //Abort on invalid inputs
            if (!Gear.IsValid || !Gear.IsSolid) { return; }
            if (!Axis.IsValid || !Axis.IsSolid) { return; }
             //Output: breps
            List<Brep> gears = new List<Brep>();
            List<Double> angleVelocity = new List<Double>();
            List<Brep> axises = new List<Brep>();
            double teethHeight = 4.0;
            //Rotate all the objects to X direction
            var xrotate = Transform.Rotation(direct, Vector3d.XAxis, Point3d.Origin);
            var xrotateBack = Transform.Rotation(Vector3d.XAxis, direct, Point3d.Origin);
            Gear.Transform(xrotate);
            Axis.Transform(xrotate);
            bp.Transform(xrotate);
            //Compute the center, radius and plane of the gear
            BoundingBox GearBox = Gear.GetBoundingBox(true);
            Point3d Center = GearBox.Center;
            double BoxX = GearBox.Max.X - GearBox.Min.X;
            double BoxY = GearBox.Max.Y - GearBox.Min.Y;
            double BoxZ = GearBox.Max.Z - GearBox.Min.Z;
            double thickness = Math.Min(Math.Min(BoxX, BoxY), BoxZ);
            double radius = Math.Max(Math.Max(BoxX, BoxY), BoxZ) / 2 - 1.0;
            //Interprete Distance to Gear Min and Max
            double max = radius;
            double minFinal = 9;
            double min=radius-(radius-minFinal)/4*(speed-1);            
            //if thickness==BoxY
            Plane GearPlane = new Plane(Center, Vector3d.XAxis, Vector3d.ZAxis);
            Vector3d GearNormal = Vector3d.YAxis;
            /*
            if (thickness == BoxX)
            {
                GearPlane = new Plane(Center, Vector3d.YAxis, Vector3d.ZAxis);
                GearNormal = Vector3d.XAxis;
            }
            */
            if (thickness == BoxZ)
            {
                GearPlane = new Plane(Center, Vector3d.XAxis, Vector3d.YAxis);
                GearNormal = Vector3d.ZAxis;
            }
            //Output
            //bool hasSteer = false;
            List<Brep> steeringGear = new List<Brep>();
            //Test the direction of Axis
            BoundingBox bA = Axis.GetBoundingBox(false);
            double bX = bA.Max.X - bA.Min.X;
            double bY = bA.Max.Y - bA.Min.Y;
            Vector3d AxisDirection = Vector3d.XAxis;
            if (bX < bY)
                AxisDirection = Vector3d.YAxis;
            //The dimension of the bp
            BoundingBox box = bp.GetBoundingBox(false);
            double length = box.Max.DistanceTo(box.Min);
            //brepOutLine is used for further calculation of whether the axis is valid and maxGearRadius
            Curve[] brepOutLine;
            Point3d[] brepOutPoints;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(bp, GearPlane, RhinoMath.SqrtEpsilon, out brepOutLine, out brepOutPoints);
            brepOutLine = Curve.JoinCurves(brepOutLine);
            Point3d AxisCenter;
            if (GearNormal == AxisDirection)
            {
                Curve[] curves;
                Point3d[] points;                
                if (Rhino.Geometry.Intersect.Intersection.BrepPlane(Axis, GearPlane, 0, out curves, out points) && Brep.CreatePlanarBreps(curves, RhinoMath.SqrtEpsilon) != null)
                {
                    Brep surface = Brep.CreatePlanarBreps(curves, RhinoMath.SqrtEpsilon)[0];
                    BoundingBox b = surface.GetBoundingBox(false);
                    AxisCenter = b.Center;
                }
                else
                {
                    RhinoApp.Write("The axis is invalid.");
                    return;
                    /*
                    AxisCenter = NearestPointToGear(Center, bA.Center, bX, bY, AxisDirection);
                    Vector3d GearDirection = MoveGear(AxisCenter, Center, GearNormal, 0);
                    var xf = Transform.Translation(GearDirection);
                    Gear.Transform(xf);
                    Center.Transform(xf);
                    GearPlane.Transform(xf);
                    */
                }
                
            }
            else
            {
               // hasSteer = true;
                Point3d AxisNear = NearestPointToGear(Center, bA.Center, bX, bY, AxisDirection);
                double dist = GearPlane.DistanceTo(AxisNear);
                double AxisRadius;
                if (dist < minFinal || max < dist)
                {
                    RhinoApp.Write("The axis in invalid.");
                    return;
                    /*
                    AxisRadius = 10;
                    Vector3d GearDirection = MoveGear(AxisCenter, Center, GearNormal, AxisRadius);
                    var xf = Transform.Translation(GearDirection);
                    Gear.Transform(xf);
                    Center.Transform(xf);
                    GearPlane.Transform(xf);
                    */
                }
                else
                    AxisRadius = dist;
                AxisCenter = FindLineEnd(AxisNear, Center, GearNormal);
            }
            //Curve c = new Line(Center, AxisCenter).ToNurbsCurve();
            /*
            if (!TestAxisValid(brepOutLine, c, GearPlane, min))
            {
                RhinoApp.WriteLine("The brep is too small to contain all the gears.");
                return;
            }
            */
            #region
            //generate Gears on Axis
            angleVelocity.Add(initialVelocity);
            Line l = new Line(Center, Vector3d.XAxis, Center.DistanceTo(AxisCenter));
            var rotateGear = Transform.Rotation(AxisCenter - Center, Vector3d.XAxis, Center);
            var rotateGearBack= Transform.Rotation(Vector3d.XAxis,AxisCenter - Center,Center);
            bp.Transform(rotateGear);
            //Gear.Transform(rotateGear);
            gears.Add(Gear);
            //myDoc.Objects.AddBrep(gears[0]);
            foreach (Curve outline in brepOutLine)
                outline.Transform(rotateGear);
            l.From = l.PointAtLength(radius);
            Point3d MidCenter;
            Point3d blockCenter;
            double angleRadius = radius - 3;
            double perVelocity=initialVelocity;
            double MidCenterNormal = 0;
            int n = 4;
            double blockRadius = 4;
            //double angle = Vector3d.VectorAngle(Vector3d.XAxis,AxisCenter-Center);
            while (min + max + minFinal < l.Length)
            {
                MidCenter = l.PointAtLength(min);
                if (GearNormal == Vector3d.YAxis)
                {
                    MidCenter.Y -= MidCenterNormal;
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y + thickness/2+2, MidCenter.Z);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                }                                 
                else
                {
                    MidCenter.Z -= MidCenterNormal;
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y, MidCenter.Z+ thickness / 2 + 2);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                }
                double r = min - 3;
                //int divide = (int)r * n;
                int divide = ((int)r * 3 % 4 == 0 || (int)r * 3 % 4 == 1) ? ((int)r * 3 / 4) * 4 : ((int)r * 3 / 4 + 1) * 4;
                //int divide = ((int)r % 2 == 0) ? (int)r * 2 : ((int)r + 1) * 2;
                gears.Add(CreateGear(GearPlane, MidCenter, r, divide, teethHeight, thickness+2));
                perVelocity = -angleRadius / r * perVelocity;
                angleVelocity.Add(perVelocity);
                //Update MidCenterNormal
                MidCenterNormal += thickness + 1;
                if (GearNormal == Vector3d.YAxis)
                {
                    MidCenter.Y -= (thickness + 1);
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y - thickness/2-2, MidCenter.Z);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                }                    
                else
                {
                    MidCenter.Z -= (thickness + 1);
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y, MidCenter.Z - thickness / 2 - 2);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                }                    
                r = maxGearRadius(MidCenter, brepOutLine, max);
                l.From = l.PointAtLength(min + r);
                r -= 3;
                angleRadius = r;
                divide = ((int)r * 3 % 4 == 0 || (int)r * 3 % 4 == 1) ? ((int)r * 3 / 4) * 4 : ((int)r * 3 / 4 + 1) * 4;
                //divide = ((int)r % 2 == 0) ? (int)r * 2 : ((int)r + 1) * 2;
                //divide = (int)r * n;
                gears.Add(CreateGear(GearPlane, MidCenter, r, divide, teethHeight, thickness));
                angleVelocity.Add(perVelocity);
                axises.Add(CreateGearAxis(MidCenter, GearNormal, GearPlane, bp, length));
            }
            if (l.Length < max)
            {
                MidCenter = l.To;
                double r = l.Length - 3.0;
                int divide = ((int)r * 3 % 4 == 0 || (int)r * 3 % 4 == 1) ? ((int)r * 3 / 4) * 4 : ((int)r * 3 / 4 + 1) * 4;
                //int divide = ((int)r % 2 == 0) ? (int)r * 2 : ((int)r + 1) * 2;
                //int divide = (int)r * n;
                if (GearNormal == Vector3d.YAxis)
                {
                    MidCenter.Y -= MidCenterNormal;
                    if (AxisDirection != GearNormal)
                    {
                        angleRadius = r;
                        blockCenter = new Point3d(MidCenter.X, MidCenter.Y + thickness / 2 + 1, MidCenter.Z);
                        axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                        blockCenter = new Point3d(MidCenter.X, MidCenter.Y - thickness / 2 - 2, MidCenter.Z);
                        axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                        axises.Add(CreateGearAxis(MidCenter, GearNormal, GearPlane, bp, length));
                    }
                }                    
                else
                {
                    MidCenter.Z -= MidCenterNormal;
                    angleRadius = r;
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y, MidCenter.Z + thickness / 2 + 1);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y, MidCenter.Z - thickness / 2 - 2);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                    axises.Add(CreateGearAxis(MidCenter, GearNormal, GearPlane, bp, length));
                }                                  
                gears.Add(CreateGear(GearPlane, MidCenter, r, divide, teethHeight, thickness));
                perVelocity =-angleRadius / r * perVelocity;
                angleVelocity.Add(perVelocity);              
            }
            else
            {
                MidCenter = l.PointAtLength((l.Length - minFinal) / 2);
                if (GearNormal == Vector3d.YAxis)
                {
                    MidCenter.Y -= MidCenterNormal;
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y + thickness/2+1, MidCenter.Z);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y - thickness/2 - 2, MidCenter.Z);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                }
                else
                {
                    MidCenter.Z -= MidCenterNormal;
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y, MidCenter.Z + thickness / 2+1);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y, MidCenter.Z - thickness / 2 - 2);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                }
                double r = (l.Length - minFinal) / 2 - 3.0;
                //int divide = (int)r * n;
                int divide = ((int)r * 3 % 4 == 0 || (int)r * 3 % 4 == 1) ? ((int)r * 3 / 4) * 4 : ((int)r * 3 / 4 + 1) * 4;
                //int divide = ((int)r % 2 == 0) ? (int)r * 2 : ((int)r + 1) * 2;
                gears.Add(CreateGear(GearPlane, MidCenter, r, divide, teethHeight, thickness));
                perVelocity = -angleRadius / r * perVelocity;
                angleVelocity.Add(perVelocity);
                axises.Add(CreateGearAxis(MidCenter, GearNormal, GearPlane, bp, length));
                angleRadius = r;
                //final gear
                MidCenter = l.To;
                r = minFinal - 3.0;
                //divide = (int)r * n;
                divide = ((int)r * 3 % 4 == 0 || (int)r * 3 % 4 == 1) ? ((int)r * 3 / 4) * 4 : ((int)r * 3 / 4 + 1) * 4;
                //divide = ((int)r % 2 == 0) ? (int)r * 2 : ((int)r + 1) * 2;
                if (GearNormal == Vector3d.YAxis)
                {
                    MidCenter.Y -= MidCenterNormal;
                    if (AxisDirection != GearNormal)
                    {
                        angleRadius = r;
                        blockCenter = new Point3d(MidCenter.X, MidCenter.Y + thickness / 2 + 1, MidCenter.Z);
                        axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                        blockCenter = new Point3d(MidCenter.X, MidCenter.Y - thickness / 2 - 2, MidCenter.Z);
                        axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                        axises.Add(CreateGearAxis(MidCenter, GearNormal, GearPlane, bp, length));
                    }
                }
                else
                {
                    MidCenter.Z -= MidCenterNormal;
                    angleRadius = r;
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y, MidCenter.Z + thickness / 2 + 1);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                    blockCenter = new Point3d(MidCenter.X, MidCenter.Y, MidCenter.Z - thickness / 2 - 2);
                    axises.Add(CreateAxis(blockCenter, blockRadius, GearPlane, GearNormal));
                    axises.Add(CreateGearAxis(MidCenter, GearNormal, GearPlane, bp, length));
                }               
                gears.Add(CreateGear(GearPlane, MidCenter, r, divide, teethHeight, thickness));
                perVelocity = -angleRadius / r * perVelocity;
                angleVelocity.Add(perVelocity);
                angleRadius = r;
            }
            //myDoc.Objects.AddBrep(Gear);
            foreach (Brep brep in gears)
            {
                brep.Transform(rotateGearBack);
            }
            //Gear.Transform(rotateGearBack);
            //myDoc.Objects.AddBrep(Gear);
            foreach (Brep brep in axises)
            {
                brep.Transform(rotateGearBack);
            }  
            if(AxisDirection!=GearNormal)
            {
                Circle FinalSteer = FindFinalCircle(gears[gears.Count - 1], bA.Center, bX, bY, AxisDirection);
                perVelocity = -angleRadius / FinalSteer.Radius * perVelocity;
                angleVelocity.Add(perVelocity);
                steeringGear=CreateSteeringGear(FinalSteer, thickness, teethHeight);
                //gears.Add(CreateSteeringGear(FinalSteer, thickness, teethHeight));
            }
            foreach (Brep brep in gears)
                brep.Transform(xrotateBack);
            //myDoc.Objects.AddBrep(Gear);
            //Gear.Transform(xrotateBack);
            //myDoc.Objects.AddBrep(Gear);
            foreach (Brep brep in axises)
                brep.Transform(xrotateBack);
            //foreach(Brep brep in steeringGear)
                //brep.Transform(xrotateBack);         
            #endregion
            DA.SetDataList(0, gears);
            DA.SetDataList(1, axises);
            DA.SetDataList(2, angleVelocity);
            DA.SetDataList(3, steeringGear);
            DA.SetData(4, AxisDirection);
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("989a5f62-d291-4718-9cf2-43795d79bc9e"); }
        }
    }
}
