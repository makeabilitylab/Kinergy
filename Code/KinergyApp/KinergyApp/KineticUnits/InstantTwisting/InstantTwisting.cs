using System;
using System.Collections.Generic;
using System.IO;
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
using Kinergy.Utilities;
using Kinergy.Geom;
using Kinergy.Relationship;
using Kinergy.KineticUnit;
using Kinergy;
using System.Diagnostics;
using Rhino.Geometry.Intersect;

namespace Kinergy.KineticUnit
{
    public class InstantTwisting:KineticUnit
    {
        //Initial inputs
        Brep model;
        Cylinder innerCylinderForComponents;
        Vector3d direction;
        //Parameters given by user
        double energy, maxDegree;
        Vector3d knobDirection=Vector3d.Unset;
        Brep endEffectorModel;
        Point3d LockPosition;

        //Generated parameters,mostly for spiral spring
        double spiralX = 0, spiralY = 0;
        int roundNum = 0;
        double spiralInnerRadius = 0, spiralOuterRadius = 0;
        double axisRadius = 0;
        double lockDisToAxis = 0;
        Point3d lockClosestPointOnAxis;
        

        //Generated components
        Spiral spiralSpring;
        RodLike centerAxis;
        List<Lock> locks;
        RodLike endEffector;

        public InstantTwisting(Brep InputModel,Vector3d mainDirection,Brep innerCylinder,double KineticStrength,double MaxLoadingDegree)
        {
            model = InputModel;
            direction = mainDirection;
            innerCylinderForComponents = GetCylinder(innerCylinder,mainDirection);
            energy = KineticStrength;
            maxDegree = MaxLoadingDegree;
            GenerateSpiralAndAxis();
            entityList.Add(new Shape(model));
        }
        private void GenerateSpiralAndAxis()
        {
            FixParameter();
            //use the given cylinder to get center
            spiralSpring = new Spiral(innerCylinderForComponents.Center, direction, spiralOuterRadius, spiralInnerRadius, spiralX, spiralY, roundNum);
            BoxLike b = new BoxLike(model, direction);
            BoundingBox box = b.Bbox;
            box.Transform(b.RotateBack);
            if(knobDirection==Vector3d.Unset)
            {
                //Let user do the selection
                Arrow a1 = new Arrow(innerCylinderForComponents.Axis, box.PointAt(1, 0.5, 0.5),1);
                Arrow a2 = new Arrow(-innerCylinderForComponents.Axis, box.PointAt(0, 0.5, 0.5), 1);
                List<Curve> arrowCurves = new List<Curve>();
                arrowCurves.Add(a1.ArrowCurve);
                arrowCurves.Add(a2.ArrowCurve);
                int result=UserSelection.UserSelectCurveInRhino(arrowCurves, RhinoDoc.ActiveDoc);
                if (result == 0)
                    knobDirection = direction;
                else
                    knobDirection = -direction;
            }
            //Then the centerAxis is generated with knob. Be careful with the direction
            BoxLike currB = new BoxLike(model, direction);
            Curve skeleton = currB.Skeleton;
            skeleton.Transform(currB.RotateBack);
            Point3d skStart = skeleton.PointAtNormalizedLength(0);
            Point3d skEnd = skeleton.PointAtNormalizedLength(1);
            double intervalStart = 0, intervalEnd = 0;
            if(knobDirection*direction>0)
            {
                intervalStart =0;
                intervalEnd = (new Vector3d(skEnd) - new Vector3d(innerCylinderForComponents.Center)) * direction / direction.Length+axisRadius*5;
            }
            else
            {
                intervalStart = 0;
                intervalEnd = -(new Vector3d(skStart) - new Vector3d(innerCylinderForComponents.Center)) * direction / direction.Length+axisRadius*5;
            }
            centerAxis = new RodLike(innerCylinderForComponents.Center, axisRadius, knobDirection, new Interval(intervalStart,intervalEnd),false);
            centerAxis.AddKnob(1,axisRadius*2,axisRadius*2);
            entityList.Add(spiralSpring);
            entityList.Add(centerAxis);
            _=new Fixation(centerAxis, spiralSpring);
        }
        private void FixParameter()
        {
            spiralOuterRadius = innerCylinderForComponents.Radius * 0.95;
            spiralInnerRadius = Math.Max(2, innerCylinderForComponents.Radius * 0.1);
            axisRadius = spiralInnerRadius;
            roundNum = (int)Math.Ceiling(maxDegree / 360 / (spiralOuterRadius - spiralInnerRadius) * (2 * spiralOuterRadius + 4 * spiralInnerRadius));
            spiralX = (spiralOuterRadius - spiralInnerRadius) / roundNum * 0.4 * Math.Pow(energy, 0.25);
            spiralY= spiralX * Math.Pow(energy, 0.25)*3;
            spiralY = Math.Min(innerCylinderForComponents.TotalHeight, spiralY);
        }
        public void AdjustParameter(double KineticStrength, double MaxLoadingDegree)
        {
            energy = KineticStrength;
            maxDegree = MaxLoadingDegree;
            GenerateSpiralAndAxis();
        }
        /// <summary>
        /// This method generate the lock pos candidates and let user select
        /// </summary>
        public void SetLockPosition()
        {
            //First calculate lock pos candidates
            //With knob direction given, we could calculate the box that lock pos should be in.
            List<Point3d> posCandidates = CalculateLockPositionCandidate();
            LockPosition = posCandidates[UserSelection.UserSelectPointInRhino(posCandidates, RhinoDoc.ActiveDoc)];
            double t = 0;
            centerAxis.Skeleton.ClosestPoint(LockPosition, out t);
            t = centerAxis.Skeleton.Domain.NormalizedParameterAt(t);
            lockClosestPointOnAxis = centerAxis.Skeleton.PointAtNormalizedLength(t);
            lockDisToAxis = LockPosition.DistanceTo(lockClosestPointOnAxis);
        }
        private List<Point3d> CalculateLockPositionCandidate()
        {
            List<Point3d> lockPosCandidates = new List<Point3d>();
            BoxLike b = new BoxLike(model, direction);
            BoundingBox box = b.Bbox,targetBox=BoundingBox.Unset;

            //First tell the knob direction and get the target box
            if (knobDirection * direction > 0)
                targetBox = new BoundingBox(box.PointAt(0.85, 0, 0), box.PointAt(1, 1, 1));
            else
                targetBox = new BoundingBox(box.PointAt(0, 0, 0), box.PointAt(0.15, 1, 1));
            targetBox.Transform(b.RotateBack);
            box.Transform(b.RotateBack);
            for (double i= 0;i<= 1;i+=0.05)
            {
                for (double j = 0; j <= 1; j += 0.05)
                {
                    Line l = new Line(box.PointAt(0, i, j), box.PointAt(1, i, j));
                    Point3d[] pts;
                    Intersection.CurveBrep(l.ToNurbsCurve(), model, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out pts);
                    foreach(Point3d pt in pts)
                    {
                        if (targetBox.Contains(pt)&&pt.DistanceTo(targetBox.Center)>axisRadius*2)
                            lockPosCandidates.Add(pt+knobDirection*3);
                    }
                }
            }
            return lockPosCandidates;
        }
        public void ConstructLocks()
        {
            locks = new List<Lock>();
            BoxLike b = new BoxLike(model, direction);
            BoundingBox box = b.Bbox;
            //Build the locking structure. ITwould be easy
            Lock LockHead = new Lock(lockClosestPointOnAxis, direction, lockDisToAxis);
            
            Point3d railS = LockPosition - knobDirection * 3 * axisRadius;
            Point3d railE= LockPosition + knobDirection * 3 * axisRadius;
            Line l = new Line(railS, railE);
            Brep LockBaseBrep = Brep.CreatePipe(l.ToNurbsCurve(), lockDisToAxis * 0.1,false,PipeCapMode.Round,true,RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,RhinoDoc.ActiveDoc.ModelAngleToleranceRadians)[0];

            locks.Add(LockHead);
            locks.Add(new Lock(LockBaseBrep, false));
            entityList.Add(LockHead);
            entityList.Add(locks[1]);
            LockHead.RegisterOtherPart(locks[1]);
            _ = new Fixation(centerAxis, LockHead);
        }
        public void SetEndEffector(Brep EEM)
        {
            //First generate some candidate points on model to serve as connecting point.
            Transform rotateX = Transform.Rotation(direction, Vector3d.XAxis, Point3d.Origin);
            Transform rotateBack = Transform.Rotation(Vector3d.XAxis,direction,  Point3d.Origin);
            Brep b = EEM.DuplicateBrep();
            b.Transform(rotateX);
            BoundingBox bbox = b.GetBoundingBox(true);
            bbox.Transform(rotateBack);
            b.Transform(rotateBack);
            List<Point3d> candidates = new List<Point3d>();
            for (double i = 0; i <= 1; i += 0.05)
            {
                for (double j = 0; j <= 1; j += 0.05)
                {
                    Line l = new Line(bbox.PointAt(0, i, j), bbox.PointAt(1, i, j));
                    Point3d[] pts;
                    Intersection.CurveBrep(l.ToNurbsCurve(), b, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out pts);
                    foreach (Point3d pt in pts)
                    {
                        candidates.Add(pt);
                    }
                }
            }
            Point3d ContactPosition = candidates[UserSelection.UserSelectPointInRhino(candidates, RhinoDoc.ActiveDoc)];
            
            b.Transform(Transform.Translation(new Vector3d(centerAxis.Skeleton.PointAtStart) - new Vector3d(ContactPosition)));
            endEffector = new RodLike(b,centerAxis.Direction);
            entityList.Add(endEffector);
            _ = new Fixation(endEffector, centerAxis);
        }
        /// <summary>
        /// Thismethod is for conveniently get component models.
        /// </summary>
        /// <param name="items">The list of items u want. There are "Spiral", "Axis", "BaseModel","Locks","EndEffector" to choose from</param>
        /// <returns></returns>
        public List<Brep> GetModel(List<string> items)
        {
            List<Brep> models = new List<Brep>();
            if (items.Contains("Spiral"))
                models.Add(spiralSpring.GetModelinWorldCoordinate());
            if (items.Contains("Axis"))
                models.Add(centerAxis.GetModelinWorldCoordinate());
            if (items.Contains("BaseModel"))
                models.Add(model);
            if(items.Contains("Locks"))
            {
                models.Add(locks[0].GetModelinWorldCoordinate());
                models.Add(locks[1].GetModelinWorldCoordinate());
            }
            if (items.Contains("EndEffector"))
                models.Add(endEffector.GetModelinWorldCoordinate());
            return models;
        }
        private Cylinder GetCylinder(Brep c,Vector3d d)
        {
            Brep m = c.DuplicateBrep();
            m.Transform(Transform.Rotation(d, Vector3d.XAxis,Point3d.Origin));
            BoundingBox b=m.GetBoundingBox(true);
            double r = (b.Max.Y - b.Min.Y)/2;
            double l = b.Max.X - b.Min.X;
            Point3d startPoint = b.PointAt(0, 0.5, 0.5);
            startPoint.Transform(Transform.Rotation(Vector3d.XAxis, d, Point3d.Origin));
            Plane p = new Plane(startPoint, d);
            Circle circle = new Circle(p, r);
            Cylinder cylinder = new Cylinder(circle, l);
            return cylinder;
        }
        public override bool LoadKineticUnit()
        {
            //Movement twist=new Movement(centerAxis,2,maxDegree/180*Math.PI,Transform.Rotation(maxDegree / 180 * Math.PI,centerAxis.Direction,centerAxis.Center));
            Movement twist = new Movement(centerAxis, 2, maxDegree / 180 * Math.PI, Transform.Rotation(maxDegree / 180 * Math.PI, direction, spiralSpring.CenterPoint));
            twist.Activate();
            locks[0].SetLocked();
            Loaded = true;
            return true;
        }
        public override bool Trigger()
        {
            return locks[0].Activate();//Create point and wait for selection
        }
        public override bool TriggerWithoutInteraction()
        {
            return locks[0].ActivateWithoutInteraction();//Just release locks, no need to wait for selection.
        }
        public override Movement Simulate(double interval = 20, double precision = 0.01)
        {
            Movement m = null;
            m = spiralSpring.Activate(interval);
            m.Activate();
            return m;
        }
    }
}
