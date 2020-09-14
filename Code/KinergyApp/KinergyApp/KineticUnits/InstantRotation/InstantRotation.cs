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
    public class InstantRotation:KineticUnit
    {
        //Initial inputs
        Brep model;
        Cylinder innerCylinderForComponents;
        Vector3d direction;
        //Parameters given by user
        double energy_old, maxDegree;
        int energy;
        double displacement;
        Vector3d knobDirection=Vector3d.Unset;
        int EEDirection = 0;
        Point3d LockPosition;
        Brep b1=null, b2=null, b3=null;
        double t1 = 0, t2 = 0;
        //Generated parameters,mostly for spiral spring
        double spiralX = 0, spiralY = 0;
        int roundNum = 0;
        double spiralInnerRadius = 0, spiralOuterRadius = 0;
        double axisRadius = 0;
        double lockDisToAxis = 0;
        Point3d lockClosestPointOnAxis;
        Curve skeleton;
        double skeletonLen = 0;
        Arrow a;
        //Generated components
        Spiral spiralSpring;
        RodLike centerAxis;
        List<Lock> locks;
        RodLike basePart, MidPart,endEffector;
        List<Point3d> lockPosCandidates;
        /// <summary>
        /// Old constructor
        /// </summary>
        /// <param name="InputModel"></param>
        /// <param name="mainDirection"></param>
        /// <param name="innerCylinder"></param>
        /// <param name="KineticStrength"></param>
        /// <param name="MaxLoadingDegree"></param>
        public InstantRotation(Brep InputModel,Vector3d mainDirection,Brep innerCylinder,double KineticStrength,double MaxLoadingDegree)
        {
            model = InputModel;
            direction = mainDirection;
            innerCylinderForComponents = GetCylinder(innerCylinder,mainDirection);
            energy_old = KineticStrength;
            maxDegree = MaxLoadingDegree;
            BoxLike currB = new BoxLike(model, direction);
            skeleton = currB.Skeleton;
            skeleton.Transform(currB.RotateBack);
            skeletonLen = skeleton.PointAtNormalizedLength(0).DistanceTo(skeleton.PointAtNormalizedLength(1));
            GenerateSpiralAndAxis();
            entityList.Add(new Shape(model));
        }
        public InstantRotation(Brep InputModel,Vector3d mainDirection,  Brep innerCylinder,int e,double disp,bool fornothing)
        {
            model = InputModel;
            direction = mainDirection;
            innerCylinderForComponents = GetCylinder(innerCylinder, mainDirection);
            energy = e;
            displacement = disp;//as degrees
            maxDegree = Math.PI *displacement/180;
            BoxLike currB = new BoxLike(model, direction);
            skeleton = currB.Skeleton;
            skeleton.Transform(currB.RotateBack);
            skeletonLen = skeleton.PointAtNormalizedLength(0).DistanceTo(skeleton.PointAtNormalizedLength(1));
            //GenerateSpiralAndAxis_New();
            //entityList.Add(new Shape(model));
        }
        private void GenerateSpiralAndAxis()
        {
            FixParameter();
            //use the given cylinder to get center
            spiralSpring = new Spiral(innerCylinderForComponents.Center, direction, spiralOuterRadius, spiralInnerRadius, roundNum, spiralX, spiralY);
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
            _ = new Fixation(centerAxis, endEffector);
        }
        public void GenerateSpiralAndAxis(Arrow ar)
        {
            a = ar;
            
            //With given arrow, set 3 parts by cutting, adding.
            if (a.Direction * direction > 0)
                EEDirection = 1;
            else
                EEDirection = 2;
            //First cut the middle part. leave the shell and gap.
            
            Plane pl = Plane.Unset;
            if(EEDirection==1)
            {
                pl = new Plane(skeleton.PointAtNormalizedLength(t2 - 2.5 / skeletonLen), direction);
            }
            else if(EEDirection==2)
            {
                pl = new Plane(skeleton.PointAtNormalizedLength(t1 + 2.5 / skeletonLen), -direction);
            }
            Brep b2Cut1 = b2.Trim(pl, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            Brep b2Inside = Brep.CreateOffsetBrep(b2Cut1, 2, true, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,out _,out _)[0];
            if(b2.GetBoundingBox(true ).Volume<b2Inside.GetBoundingBox(true ).Volume)
            {
                b2Cut1.Flip();
                b2Inside = Brep.CreateOffsetBrep(b2Cut1, 2, true, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out _)[0];
            }
            //b2Inside=b2Inside.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            
            //Brep b2Cut1 = b2.Trim(pl, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            //b2Cut1= b2Cut1.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            
            var s= b2Inside.IsSolid;
            //b2Inside.Flip();
            //var booleanResult= Brep.CreateBooleanDifference(b2Cut1, b2Inside, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep midBrep = b2Inside;
            /*foreach (Brep b in booleanResult)
                midBrep.Append(b);*/
            //Brep midBrep = new Brep();
            /*midBrep.Append(b2Cut1);
            midBrep.Append(b2Inside);*/
            //midBrep = midBrep.Trim(pl, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            //midBrep.CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            if (EEDirection == 1)
            {
                MidPart = new RodLike(midBrep, direction);
                basePart = new RodLike(b1, direction);
                endEffector= new RodLike(b3, direction);
                //entityList.Add(new Socket(skeleton.PointAtNormalizedLength(t1), direction,5));
                BoundingBox bbox = MidPart.Model.GetBoundingBox(true);
                //Point3d axisStart = skeleton.PointAtNormalizedLength(t1 + 2 / skeletonLen);
                //Point3d axisStart = skeleton.PointAtNormalizedLength(t1 ), axisEnd = skeleton.PointAtNormalizedLength(t2);
                Point3d axisStart = bbox.Center - direction / direction.Length * (bbox.Max.X - bbox.Min.X) / 2;
                Point3d axisEnd = bbox.Center + direction / direction.Length * ((bbox.Max.X -bbox.Min.X) / 2+2.5);
                entityList.Add(new Socket(axisStart, direction, 8));
                Cylinder c1 = new Cylinder(new Circle(new Plane(axisStart, direction), 6));
                c1.Height1 = 1;
                c1.Height2 = 2;
                Cylinder c2 = new Cylinder(new Circle(new Plane(axisStart, direction), 3));
                c2.Height1 = 1;c2.Height2 = axisStart.DistanceTo(axisEnd);
                Brep axisB = c1.ToBrep(true, true);
                axisB.Append(c2.ToBrep(true, true));
                centerAxis = new RodLike(axisB, direction);
                centerAxis.Skeleton = new Line(axisStart, axisEnd).ToNurbsCurve();
                entityList.Add(centerAxis);
            }
            else
            { 
                MidPart = new RodLike(midBrep, -direction);
                basePart = new RodLike(b3, -direction);
                endEffector = new RodLike(b1, -direction);
                //entityList.Add(new Socket(skeleton.PointAtNormalizedLength(t2), -direction, 5));
                //Point3d axisStart = skeleton.PointAtNormalizedLength(t2 - 2 / skeletonLen);
                //Point3d axisStart = skeleton.PointAtNormalizedLength(t2), axisEnd = skeleton.PointAtNormalizedLength(t1);
                BoundingBox bbox = MidPart.Model.GetBoundingBox(true);
                Point3d axisStart = bbox.Center + direction / direction.Length * (bbox.Max.X - bbox.Min.X) / 2;
                Point3d axisEnd = bbox.Center - direction / direction.Length * ((bbox.Max.X -bbox.Min.X) / 2+2.5);
                entityList.Add(new Socket(axisStart, -direction, 8));
                Cylinder c1 = new Cylinder(new Circle(new Plane(axisStart, -direction), 6));
                c1.Height1 = 1;
                c1.Height2 = 2;
                Cylinder c2 = new Cylinder(new Circle(new Plane(axisStart, -direction), 3));
                c2.Height1 = 1; c2.Height2 = axisStart.DistanceTo(axisEnd);
                Brep axisB = c1.ToBrep(true, true);
                axisB.Append(c2.ToBrep(true, true));
                centerAxis = new RodLike(axisB, -direction);
                centerAxis.Skeleton = new Line(axisStart, axisEnd).ToNurbsCurve();
                entityList.Add(centerAxis);
            }
            entityList.Add(MidPart);
            entityList.Add(basePart);
            entityList.Add(endEffector);
            //Then generate spiral
            spiralSpring = new Spiral(direction, MidPart.Model.GetBoundingBox(true).Center, innerCylinderForComponents.Radius - 0.5, maxDegree, energy);
            //spiralSpring = new Spiral(direction, skeleton.PointAtNormalizedLength(t1 / 2 + t2 / 2), innerCylinderForComponents.Radius - 2, maxDegree, energy);
            entityList.Add(spiralSpring);
            //CalculateLockPosCandidates
            lockPosCandidates = new List<Point3d>();
            for(double i=t1+3/skeletonLen; i<=t2-3/skeletonLen;i+=2/skeletonLen)
            {
                Point3d centerP = skeleton.PointAtNormalizedLength(i);
                if(centerP.DistanceTo(spiralSpring.Center)>spiralSpring.ThicknessY/2+2)
                {
                    //Add points on the outer surface
                    Plane plane = new Plane(centerP, direction);
                    Curve[] c;
                    Point3d[] pt;
                    Rhino.Geometry.Intersect.Intersection.BrepPlane(b2, plane, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out c, out pt);
                    for (int j = 0; j < 10; j++)
                    {
                        lockPosCandidates.Add(c[0].PointAtNormalizedLength(j * 0.1));
                    }
                }
            }
        }
        public void Set3Parts(double T1,double T2,Brep B1,Brep B2,Brep B3)
        {
            t1 = T1;
            t2 = T2;
            b1 = B1;
            b2 = B2;
            b3 = B3;
        }
        private void FixParameter()
        {
            spiralOuterRadius = innerCylinderForComponents.Radius * 0.95;
            spiralInnerRadius = Math.Max(2, innerCylinderForComponents.Radius * 0.1);
            axisRadius = spiralInnerRadius;
            roundNum = (int)Math.Ceiling(maxDegree / 360 / (spiralOuterRadius - spiralInnerRadius) * (2 * spiralOuterRadius + 4 * spiralInnerRadius));
            spiralX = (spiralOuterRadius - spiralInnerRadius) / roundNum * 0.4 * Math.Pow(energy_old, 0.25);
            spiralY= spiralX * Math.Pow(energy_old, 0.25)*3;
            spiralY = Math.Min(innerCylinderForComponents.TotalHeight, spiralY);
        }
        public void AdjustParameter(double KineticStrength, double MaxLoadingDegree)
        {
            energy_old = KineticStrength;
            maxDegree = MaxLoadingDegree;
            GenerateSpiralAndAxis();
            
        }
        public void AdjustParameter(int KineticStrength, double MaxLoadingDegree)
        {
            maxDegree = Math.PI * MaxLoadingDegree / 180;
            //GenerateSpiralAndAxis();
            spiralSpring.AdjustParam(KineticStrength, maxDegree);
            energy = KineticStrength;
        }
        public List<Arrow> GetDirectionCandidates()
        {
            Vector3d v1 = direction;
            Vector3d v2 = -v1;
            Point3d p1 =  skeleton.PointAtNormalizedLength(0);
            Point3d p2 = skeleton.PointAtNormalizedLength(1);
            Arrow a1 = new Arrow(v1, p1);
            Arrow a2 = new Arrow(v2, p2);
            return new List<Arrow> { a1, a2 };

        }
        /// <summary>
        /// This method generate the lock pos candidates and let user select
        /// </summary>
        public void SetLockPosition_Old()
        {
            //First calculate lock pos candidates
            //With knob direction given, we could calculate the box that lock pos should be in.
            List<Point3d> posCandidates = CalculateLockPositionCandidate_Old();
            LockPosition = posCandidates[UserSelection.UserSelectPointInRhino(posCandidates, RhinoDoc.ActiveDoc)];
            double t = 0;
            centerAxis.Skeleton.ClosestPoint(LockPosition, out t);
            t = centerAxis.Skeleton.Domain.NormalizedParameterAt(t);
            lockClosestPointOnAxis = centerAxis.Skeleton.PointAtNormalizedLength(t);
            lockDisToAxis = LockPosition.DistanceTo(lockClosestPointOnAxis);
        }
        public void SetLockPosition()
        {
            //First calculate lock pos candidates
            //With knob direction given, we could calculate the box that lock pos should be in.
            if(lockDisToAxis==0)//When lock pos is never set before
            {
                LockPosition = lockPosCandidates[UserSelection.UserSelectPointInRhino(lockPosCandidates, RhinoDoc.ActiveDoc)];
                double t = 0;
                //Use the axis instead!
                centerAxis.Skeleton.ClosestPoint(LockPosition, out t);
                t = centerAxis.Skeleton.Domain.NormalizedParameterAt(t);
                lockClosestPointOnAxis = centerAxis.Skeleton.PointAtNormalizedLength(t);
                lockDisToAxis = LockPosition.DistanceTo(lockClosestPointOnAxis);
            }
        }
        private List<Point3d> CalculateLockPositionCandidate_Old()
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
        public void ConstructLocks_Old()
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
        public void ConstructLocks()
        {
            SetLockPosition();
            locks = new List<Lock>();
            //Build the locking structure. ITwould be easy
            Lock LockHead;
            if (EEDirection==2)
                LockHead = new Lock(lockClosestPointOnAxis, direction, lockDisToAxis*0.6,true);
            else
                LockHead = new Lock(lockClosestPointOnAxis, direction, lockDisToAxis * 0.6, false);
            Vector3d centerLinkDirection = new Vector3d(lockClosestPointOnAxis) - new Vector3d(LockPosition);
            double centerLinkLen = centerLinkDirection.Length;
            centerLinkDirection.Unitize();
            Point3d railS = LockPosition - centerLinkDirection*10;
            Point3d railE = LockPosition + centerLinkDirection* lockDisToAxis * 0.5;
            Point3d railS1 = LockPosition - centerLinkDirection * 0.5;
            Point3d railE1 = LockPosition + centerLinkDirection * 2.5;
            Line l = new Line(railS, railE);
            Brep LockBaseBrep = Brep.CreatePipe(l.ToNurbsCurve(), 1.5, false, PipeCapMode.Flat, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAngleToleranceRadians)[0];
            Brep LockBaseBrepLarge = Brep.CreatePipe(l.ToNurbsCurve(), 2.5, false, PipeCapMode.Flat, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAngleToleranceRadians)[0];
            //LockBaseBrepLarge.Flip();
            MidPart.Model.Flip();
            MidPart.Model = Brep.CreateBooleanDifference(MidPart.Model, LockBaseBrepLarge, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            Plane pl1 = new Plane(LockPosition + centerLinkDirection * (lockDisToAxis * 0.21 + 2), centerLinkDirection);
            Cylinder c1 = new Cylinder(new Circle(pl1, 3.5));
            c1.Height1 = 0;c1.Height2 = 3;
            Plane pl2 = new Plane(LockPosition, centerLinkDirection);
            Cylinder c2 = new Cylinder(new Circle(pl2 , 3.5));
            c2.Height1 = -3; c2.Height2 = 0;
            LockBaseBrep.Append(c1.ToBrep(true, true));
            LockBaseBrep.Append(c2.ToBrep(true, true));
            LockBaseBrep.Append(new Sphere(railS, 2.5).ToBrep());
            LockBaseBrep.Transform(Transform.Translation(-centerLinkDirection * lockDisToAxis * 0.1));
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
