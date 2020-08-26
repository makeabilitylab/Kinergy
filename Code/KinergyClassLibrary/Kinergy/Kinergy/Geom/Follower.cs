using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinergy.Geom
{
    public class Follower:Component
    {
        int FollowerType = 0;
        Cam cam = null;
        double travel = 0;//positive for downfall and negative for upwars. between 0-360 for type 2 follower.
        Point3d startPoint = Point3d.Unset,endPoint=Point3d.Unset,midPoint=Point3d.Unset;
        double step = 0.2;
        double stepRotation = 0;

        /// <summary>
        /// This is the constructor for both types of follower with model to fix to.
        /// </summary>
        /// <param name="type">Type of follower, 1 for vertical and 2 for horizontal</param>
        /// <param name="Point1">The touching end of follower.or the capsule base point where connecting axis meet the capsule structure</param>
        /// <param name="Point2">The free end of follower</param>
        public Follower(int type,Entity modelToFixTo, Cam c,Point3d Point1,Point3d Point2, double rodRadius, double thick=5)
        {
            c.RegisterFollower(this);
            if (type==1)
            {
                startPoint = Point1;
                endPoint = Point2;
                FollowerType = type;
                cam = c;
                if (type != 1)
                { throw new Exception("This constructor is for vertical follower. Type should be 1"); }
                Line l = new Line(startPoint, endPoint);
                Plane base1 = new Plane(startPoint, new Vector3d(endPoint) - new Vector3d(startPoint));
                Circle c1 = new Circle(base1, rodRadius);
                Cylinder rod1 = new Cylinder(c1, l.Length);
                base.model = rod1.ToBrep(true, true);
                //Cut the model to fix to with slightly thicker rod shape, and leave some tube-like shape.
                Circle c2 = new Circle(base1, rodRadius * 1.1);
                Cylinder rod2 = new Cylinder(c2, l.Length);
                //Point3d[] pts;
                //Intersection.CurveBrep(l.ToNurbsCurve(), modelToFixTo.Model, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out pts);
                modelToFixTo.SetModel(Brep.CreateBooleanDifference(modelToFixTo.Model, rod2.ToBrep(true, true), RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0]);
                //Then move to contact cam
                Follow();
            }
            if(type==2)
            {
                FollowerType = type;
                cam = c;
                midPoint = Point1;
                endPoint = Point2;
                //The type2 follower is formed with an capsule and a rod
                double distance = new Line(c.CenterPoint, midPoint).Length;
                double radius = rodRadius*1.5;
                if (radius <= 0)
                { throw new Exception("Invalid midPoint position! Too close to cam!"); }
                /*Curve capsule = Utilities.GeometryMethods.Capsule(new Plane(midPoint, c.Normal), radius, radius, new Plane(midPoint, c.Normal, new Vector3d(midPoint) - new Vector3d(c.CenterPoint)).Normal);
                Point3d start = midPoint - c.Normal / c.Normal.Length * thick / 2;
                Point3d end = midPoint + c.Normal / c.Normal.Length * thick / 2;
                capsule.Transform(Transform.Translation(-c.Normal / c.Normal.Length * thick / 2));
                Curve rail = new Line(start, end).ToNurbsCurve();
                Brep capsulePart = Brep.CreateFromSweep(rail, capsule, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];*/
                Circle circle1 = new Circle(new Plane(midPoint, c.Normal), radius);
                Cylinder cylinder = new Cylinder(circle1);
                cylinder.Height1 = -rodRadius;
                cylinder.Height2 = rodRadius;
                Brep cylinderPart = cylinder.ToBrep(true, true);
                Plane rodBasePlane = new Plane(midPoint, new Vector3d(endPoint) - new Vector3d(midPoint));
                Circle circle2 = new Circle(rodBasePlane, rodRadius);
                Brep rod = new Cylinder(circle2, (new Vector3d(endPoint) - new Vector3d(midPoint)).Length).ToBrep(true, true);
                //base.model = Brep.CreateBooleanUnion(new List<Brep> { rod, capsulePart }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                Brep b = new Brep();
                b.Append(rod);
                //b.Append(capsulePart);
                b.Append(cylinderPart);
                base.model = b;
                //Tell the step rotation value. Make sure that rotation of strp rotation make the follower closer to cam.
                Vector3d capsuleDirection = new Plane(midPoint, cam.Normal, new Vector3d(midPoint) - new Vector3d(cam.CenterPoint)).Normal;
                Vector3d approachDirection = new Vector3d(cam.CenterPoint) - new Vector3d(midPoint);
                capsuleDirection.Rotate(0.1, cam.Normal);
                if (capsuleDirection * approachDirection > 0)
                { stepRotation = 1 * Math.PI / 180; }
                else
                { stepRotation = -1 * Math.PI / 180; }
                Follow();
            }
            
        }
        public Follower(int type,  Cam c, Point3d Point1, Point3d Point2, double rodRadius, double thick = 5)
        {
            c.RegisterFollower(this);
            if (type == 1)
            {
                startPoint = Point1;
                endPoint = Point2;
                FollowerType = type;
                cam = c;
                if (type != 1)
                { throw new Exception("This constructor is for vertical follower. Type should be 1"); }
                Line l = new Line(startPoint, endPoint);
                Plane base1 = new Plane(startPoint, new Vector3d(endPoint) - new Vector3d(startPoint));
                Circle c1 = new Circle(base1, rodRadius);
                Cylinder rod1 = new Cylinder(c1, l.Length);
                base.model = rod1.ToBrep(true, true);
                //Cut the model to fix to with slightly thicker rod shape, and leave some tube-like shape.
                Circle c2 = new Circle(base1, rodRadius * 1.1);
                Cylinder rod2 = new Cylinder(c2, l.Length);
                //Point3d[] pts;
                //Intersection.CurveBrep(l.ToNurbsCurve(), modelToFixTo.Model, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out pts);
                //Then move to contact cam
                Follow();
            }
            if (type == 2)
            {
                FollowerType = type;
                cam = c;
                midPoint = Point1;
                endPoint = Point2;
                //The type2 follower is formed with an capsule and a rod
                double distance = new Line(c.CenterPoint, midPoint).Length;
                double radius = rodRadius*1.5;
                if (radius < 0)
                { throw new Exception("Invalid midPoint position! Too close to cam!"); }
                /*Curve capsule = Utilities.GeometryMethods.Capsule(new Plane(midPoint, c.Normal), radius, radius, new Plane(midPoint, c.Normal, new Vector3d(midPoint) - new Vector3d(c.CenterPoint)).Normal);
                Point3d start = midPoint - c.Normal / c.Normal.Length * thick / 2;
                Point3d end = midPoint + c.Normal / c.Normal.Length * thick / 2;
                capsule.Transform(Transform.Translation(-c.Normal / c.Normal.Length * thick / 2));
                Curve rail = new Line(start, end).ToNurbsCurve();
                Brep capsulePart = Brep.CreateFromSweep(rail, capsule, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];*/
                Circle circle1 = new Circle(new Plane(midPoint, c.Normal), radius);
                Cylinder cylinder = new Cylinder(circle1);
                cylinder.Height1 = -rodRadius;
                cylinder.Height2 = rodRadius;
                Brep cylinderPart = cylinder.ToBrep(true, true);
                Plane rodBasePlane = new Plane(midPoint, new Vector3d(endPoint) - new Vector3d(midPoint));
                Circle circle2 = new Circle(rodBasePlane, rodRadius);
                Brep rod = new Cylinder(circle2, (new Vector3d(endPoint) - new Vector3d(midPoint)).Length).ToBrep(true, true);
                //base.model = Brep.CreateBooleanUnion(new List<Brep> { rod, capsulePart }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                Brep b = new Brep();
                b.Append(rod);
                //b.Append(capsulePart);
                b.Append(cylinderPart);
                base.model = b;
                //Tell the step rotation value. Make sure that rotation of strp rotation make the follower closer to cam.
                Vector3d capsuleDirection = new Plane(midPoint, cam.Normal, new Vector3d(midPoint) - new Vector3d(cam.CenterPoint)).Normal;
                Vector3d approachDirection = new Vector3d(cam.CenterPoint) - new Vector3d(midPoint);
                capsuleDirection.Rotate(0.1, cam.Normal);
                if (capsuleDirection * approachDirection > 0)
                { stepRotation = 1 * Math.PI / 180; }
                else
                { stepRotation = -1 * Math.PI / 180; }
                Follow();
            }

        }
        /// <summary>
        /// Call this function to make sure follower is contacting cam
        /// </summary>
        public void Follow()
        {
            //Try step by step simulation first. If no intersection at first, downfall until there is. If there is intersection at first, upwards until there isnt.
            if(FollowerType==1)
            {
                Vector3d direction = new Vector3d(startPoint) - new Vector3d(endPoint);
                direction.Unitize();
                Brep f = base.model.DuplicateBrep();
                f.Transform(base.offset);
                bool intersect = Utilities.GeometryMethods.IfBrepsIntersect(f, cam.GetModelinWorldCoordinate());
                if (intersect)//go upwards an keep trying until no intersection
                {
                    double moveValue = 0;
                    int counter = 0;
                    do
                    {
                        Transform m = Transform.Translation(direction * (-step));
                        moveValue -= step;
                        f.Transform(m);
                        intersect = Utilities.GeometryMethods.IfBrepsIntersect(f, cam.GetModelinWorldCoordinate());
                        counter++;
                        if(counter>200)
                        { throw new Exception("Follower failed to contact cam. Please check their positions."); }
                    } while (intersect);
                    Move(new Movement(this, 1, moveValue));
                    
                }
                else
                {
                    double moveValue = 0;
                    int counter = 0;
                    do
                    {
                        Transform m = Transform.Translation(direction * step);
                        moveValue += step;
                        f.Transform(m);
                        intersect =Utilities.GeometryMethods.IfBrepsIntersect(f, cam.GetModelinWorldCoordinate());
                        counter++;
                        if (counter > 200)
                        { throw new Exception("Follower failed to contact cam. Please check their positions."); }
                    } while (!intersect);
                    Move(new Movement(this, 1, moveValue));
                }
            }
            else if(FollowerType==2)
            {
                Vector3d capsuleDirection=new Plane(midPoint,cam.Normal, new Vector3d(midPoint) - new Vector3d(cam.CenterPoint)).Normal;
                Vector3d approachDirection = new Vector3d(cam.CenterPoint) - new Vector3d(midPoint);
                
                Brep f = base.model.DuplicateBrep();
                f.Transform(base.offset);
                bool intersect = Utilities.GeometryMethods.IfBrepsIntersect(f, cam.GetModelinWorldCoordinate());
                if (intersect)//go upwards an keep trying until no intersection
                {
                    double moveValue = 0;
                    int counter = 0;
                    do
                    {
                        Transform m = Transform.Rotation(-stepRotation,cam.Normal, endPoint);
                        moveValue -= step;
                        f.Transform(m);
                        intersect = Utilities.GeometryMethods.IfBrepsIntersect(f, cam.GetModelinWorldCoordinate());
                        counter++;
                        if (counter > 400)
                        { throw new Exception("Follower failed to contact cam. Please check their positions."); }
                    } while (intersect);
                    Move(new Movement(this, 2, moveValue));
                }
                else
                {
                    double moveValue = 0;
                    int counter = 0;
                    do
                    {
                        Transform m = Transform.Rotation(stepRotation, cam.Normal, endPoint);
                        moveValue += stepRotation;
                        f.Transform(m);
                        intersect = Utilities.GeometryMethods.IfBrepsIntersect(f, cam.GetModelinWorldCoordinate());
                        counter++;
                        if (counter > 400)
                        { throw new Exception("Follower failed to contact cam. Please check their positions."); }
                    } while (!intersect);
                    Move(new Movement(this, 2, moveValue));
                    //Move(new Movement(this, 2, 0));
                }
            }
            else
            {
                throw new Exception("Invalid follower type!");
            }
        }
        public override bool Move(Movement move)
        {
            if(FollowerType!=move.Type)
            { throw new Exception("Mismatching movement type!"); }
            if(move.Type==1)
            {
                Vector3d direction = new Vector3d(startPoint) - new Vector3d(endPoint);
                direction.Unitize();
                travel += move.MovementValue;
                Vector3d translation = direction*move.MovementValue;
                base.offset =Transform.Multiply(base.offset, Transform.Translation(translation));
                //TODO maybe there's constraint on follower.
                return true;
            }
            else if(move.Type==2)
            {
                travel += move.MovementValue;
                travel = travel % (Math.PI*2);
                base.offset = Transform.Multiply(base.offset, Transform.Rotation(move.MovementValue, cam.Normal, endPoint));
                //base.offset = Transform.Rotation(travel, cam.Normal, endPoint);
                //TODO propagate this movement?
                return true;
            }
            else
            { throw new Exception("Unexpected movement type"); }
            
        }
    }
}
