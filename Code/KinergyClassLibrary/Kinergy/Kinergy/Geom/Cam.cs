using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino;
using Rhino.Geometry;
using Kinergy.Generator;
namespace Kinergy.Geom
{
    public class Cam:Component
    {
        int type = 0;
        int camRotateDirection = 0;
        Point3d centerPoint=Point3d.Unset;
        Vector3d normal=Vector3d.Unset;
        Vector3d mainDirection = Vector3d.Unset;
        double radiusMin = 0;
        double radiusMax = 0;
        double degree = 0;
        double thickness = 0;
        Entity connector = null;
        Follower follower;
        /// <summary>
        /// Construct a cam object
        /// </summary>
        /// <param name="CamType">Type of cam, 1 for ellipse cam and 2 for snail cam</param>
        /// <param name="rMin">The minimal radius of cam</param>
        /// <param name="rMax">The maximal radius of cam</param>
        /// <param name="thick">The thickness of cam</param>
        /// <param name="rotateDirection">0 for both, 1 for clockwise only, 2 for anti-clockwise only. Please do define this as 1 or 2 if you are constructing a snail cam.</param>
        public Cam(int CamType,double rMin,double rMax,Point3d CamCenter,Vector3d CamNormal,Vector3d InitialMaxDirection,int rotateDirection=1,double thick=5,double initialAngle=0)
        {
            if(rMin>rMax)
            { rMin = rMax; }
            if(CamType==1)
            {
                //ellipse cam
                type = 1;
                CamRotateDirection = rotateDirection; 
                if(CamRotateDirection<0 || CamRotateDirection>2)
                {
                    throw new Exception("Invalid rotate direction");
                }
                centerPoint = CamCenter;
                normal = CamNormal;
                normal.Unitize();
                thickness = thick;
                mainDirection = InitialMaxDirection;
                mainDirection.Unitize();
                radiusMax = rMax;
                radiusMin = rMin;
                ConstructEllipseCam();
            }
            else if(CamType==2)
            {
                //snail cam
                type = 2;
                CamRotateDirection = rotateDirection;
                if(CamRotateDirection==0 || CamRotateDirection > 2)
                {
                    throw new Exception("Invalid rotate direction");
                }
                centerPoint = CamCenter;
                normal = CamNormal;
                normal.Unitize();
                thickness = thick;
                mainDirection = InitialMaxDirection;
                mainDirection.Unitize();
                radiusMax = rMax;
                radiusMin = rMin;
                ConstructSnailCam();
            }
            else
            {
                throw new Exception("Wrong type value passed to Cam constructor. Currently only 2 types are supported");
            }
            SetAngle(initialAngle);
        }
        private void ConstructEllipseCam()
        {
            //Generate the shape by extruding an ellipse
            Point3d start = centerPoint - normal / normal.Length * thickness / 2;
            Point3d end = centerPoint + normal / normal.Length * thickness / 2;
            Point3d p1 = start + mainDirection / mainDirection.Length * radiusMax;
            Point3d p2 = start + new Plane(centerPoint, normal, mainDirection).Normal * radiusMin;
            Ellipse e = new Ellipse(start, p1, p2);
            Curve c = e.ToNurbsCurve();
            Curve c1 = e.ToNurbsCurve();
            c1.Transform(Transform.Translation(new Vector3d(end) - new Vector3d(start)));
            Curve rail = new Line(start, end).ToNurbsCurve();
            Brep b1 = Brep.CreateFromSweep(rail, c, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            Brep b2=Brep.CreatePatch(new List<Curve> { c}, null, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep b3 = Brep.CreatePatch(new List<Curve> { c1 }, null, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep model = Brep.CreateSolid(new List<Brep> { b1, b2, b3 }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            base.model = model;
        }
        private void ConstructSnailCam()
        {
            //Generate the shape by drawing snail curve and extrude it.
            Point3d start = centerPoint - normal / normal.Length * thickness / 2;
            Point3d end = centerPoint + normal / normal.Length * thickness / 2;
            List<Point3d> pts = new List<Point3d>();
            Vector3d y = mainDirection;
            Vector3d x = new Plane(centerPoint, mainDirection, normal).Normal;
            double rStart=0, rEnd=0, step=0;
            int pointNum = 21;
            if(CamRotateDirection==1)
            {
                rStart = radiusMax;
                rEnd = radiusMin;
            }
            else 
            {
                rStart = radiusMin;
                rEnd = radiusMax;
            }
            step = (rEnd - rStart) / (pointNum - 1);
            for (int i=0;i<pointNum;i++)
            {
                double r = rStart + step * i;
                double rad = 2*Math.PI / (pointNum - 1) * i;
                Point3d pt = start + x * Math.Sin(rad)*r + y * Math.Cos(rad)*r;
                pts.Add(pt);
            }
            Curve snail = Curve.CreateInterpolatedCurve(pts, 3);
            Curve seal = new Line(pts[pointNum - 1], pts[0]).ToNurbsCurve();
            PolyCurve s=new PolyCurve();
            s.Append(snail);
            s.Append(seal);
            //Curve c = Curve.CreatePeriodicCurve(s);
            Curve c = s.ToNurbsCurve();
            //Curve c1 = Curve.CreatePeriodicCurve(s);
            Curve c1= s.ToNurbsCurve();
            c1.Transform(Transform.Translation(new Vector3d(end) - new Vector3d(start)));
            Curve rail = new Line(start, end).ToNurbsCurve();
            Brep b1 = Brep.CreateFromSweep(rail, c, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            Brep b2 = Brep.CreatePatch(new List<Curve> { c }, null, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep b3 = Brep.CreatePatch(new List<Curve> { c1 }, null, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep model = Brep.CreateSolid(new List<Brep> { b1, b2, b3 }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            base.model = model;
        }
        public void GenerateConnector(Entity main)
        {
            connector = new Connector(main, this).ConnectingStructure;//TODO add this situation in Connector code
        }
        public void RegisterConnector(Entity c)
        {
            connector = c;
        }
        public void AdjustSize(double newRmin,double newRmax)
        {
            radiusMin = newRmin;
            radiusMax = newRmax;
        }
        public void RegisterFollower(Follower f)
        {
            follower = f;
        }
        public void SetAngle(double angle)
        {
            double a = angle / 180 * Math.PI;
            //double a = angle;
            double diff = a - degree;
            Movement m = new Movement(this, 2, diff);
            m.Activate();
            degree = a;
            if(follower!=null)
            {
                follower.Follow();
            }
        }
        public override bool AddConstraint(Relationships.Relationship constraint)
        {
            return base.AddConstraint(constraint);
        }
        public override bool Move(Movement move)
        {
            return base.Move(move);
        }
        protected override void ConductMoveAndUpdateParam(Movement move)
        {
            base.offset = Transform.Multiply(base.offset, Transform.Rotation( move.MovementValue, normal,centerPoint));
        }
        public int Type { get => type;private set => type = value; }
        public Point3d CenterPoint { get => centerPoint; set => centerPoint = value; }
        public Vector3d Normal { get => normal; set => normal = value; }
        public Entity Connector { get => connector; set => connector = value; }
        public double RadiusMin { get => radiusMin; set => radiusMin = value; }
        public double RadiusMax { get => radiusMax; set => radiusMax = value; }
        public int CamRotateDirection { get => camRotateDirection;private set => camRotateDirection = value; }
        public double Degree { get => degree;private set => degree = value; }
    }
}
