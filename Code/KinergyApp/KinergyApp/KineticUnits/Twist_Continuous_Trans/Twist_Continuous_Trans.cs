using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Kinergy.KineticUnit;
using Kinergy.Geom;
using Kinergy.Utilities;
using Rhino.Geometry;
using Rhino.DocObjects;
using Rhino;
using Kinergy.Relationships;
using Rhino.Geometry.Intersect;
using InstExtension;

namespace HumanUIforKinergy.KineticUnits.Twist_Continuous_Trans
{
    public class Twist_Continuous_Trans:KineticUnit
    {
        Brep model;
        Brep innerCylinder;
        Cylinder innerCylinderForComponents;
        Vector3d mainDirection;
        double strength = 0, maxDegree = 0;
        bool toAddLock = false;
        int camType = 0;

        Vector3d knobDirection = Vector3d.Unset;
        double spiralX = 0, spiralY = 0;
        int roundNum = 0;
        double spiralInnerRadius = 0, spiralOuterRadius = 0;
        double axisRadius = 0;
        double lockDisToAxis = 0;
        Point3d lockClosestPointOnAxis;
        double camMinRadius = 0, camMaxRadius = 0;
        double camWidth = 0;

        Brep endEffectorModel=null;
        Brep inputHandlerModel=null;
        Point3d LockPosition=Point3d.Unset;
        Point3d followerPosition = Point3d.Unset;
        Spiral spiralSpring = null;
        RodLike centerAxis=null;
        List<Lock> locks;
        RodLike endEffector = null;
        RodLike inputHandler = null;
        Cam innerCam = null;
        Follower follower = null;
        public Twist_Continuous_Trans(Brep M,Brep IC,Vector3d D,double Str,double Dis,bool addLock=true,int camTypeValue=1)
        {
            model = M;
            innerCylinder = IC;
            innerCylinderForComponents = Kinergy.Utilities.GeometryMethods.GetCylinder(innerCylinder, mainDirection);
            mainDirection = D;
            strength = Str;
            maxDegree = Dis;
            entityList.Add(new Shape(model));
            toAddLock = addLock;
            camType = camTypeValue; 
            locks = new List<Lock>();
            Generate();
        }
        private void Generate()
        {
            FixParameter();
            //First Generate spiral and axis
            GenerateSpiralAndAxis();
            GenerateCamAndFollower(camType);
            if (toAddLock)
                ConstructLocks();
        }
        private void GenerateSpiralAndAxis()
        {
            //use the given cylinder to get center
            spiralSpring = new Spiral(innerCylinderForComponents.Center, mainDirection, spiralOuterRadius, spiralInnerRadius, spiralX, spiralY, roundNum);
            BoxLike b = new BoxLike(model, mainDirection);
            BoundingBox box = b.Bbox;
            box.Transform(b.RotateBack);
            if (knobDirection == Vector3d.Unset)//Let user do the selection if knob direction is unset
            {
                Arrow a1 = new Arrow(innerCylinderForComponents.Axis, box.PointAt(1, 0.5, 0.5), 1);
                Arrow a2 = new Arrow(-innerCylinderForComponents.Axis, box.PointAt(0, 0.5, 0.5), 1);
                List<Curve> arrowCurves = new List<Curve>();
                arrowCurves.Add(a1.ArrowCurve);
                arrowCurves.Add(a2.ArrowCurve);
                int result = UserSelection.UserSelectCurveInRhino(arrowCurves, RhinoDoc.ActiveDoc);
                if (result == 0)
                    knobDirection = mainDirection;
                else
                    knobDirection = -mainDirection;
            }
            //Then the centerAxis is generated with knob. Be careful with the direction
            BoxLike currB = new BoxLike(model, mainDirection);
            Curve skeleton = currB.Skeleton;
            skeleton.Transform(currB.RotateBack);
            Point3d skStart = skeleton.PointAtNormalizedLength(0);
            Point3d skEnd = skeleton.PointAtNormalizedLength(1);
            double intervalStart = 0, intervalEnd = 0;
            if (knobDirection * mainDirection > 0)
            {
                intervalStart = 0;
                intervalEnd = (new Vector3d(skEnd) - new Vector3d(innerCylinderForComponents.Center)) * mainDirection/ mainDirection.Length + axisRadius * 5;
            }
            else
            {
                intervalStart = 0;
                intervalEnd = -(new Vector3d(skStart) - new Vector3d(innerCylinderForComponents.Center)) * mainDirection/ mainDirection.Length + axisRadius * 5;
            }
            centerAxis = new RodLike(innerCylinderForComponents.Center, axisRadius, knobDirection, new Interval(intervalStart, intervalEnd), false);
            centerAxis.AddKnob(1, axisRadius * 2, axisRadius * 2);
            entityList.Add(spiralSpring);
            entityList.Add(centerAxis);
            _ = new Fixation(centerAxis, spiralSpring);
        }
        public void GenerateCamAndFollower(int Type=1)
        {
            if (Type != camType)
                camType = Type;
            if (innerCam != null)
            {
                entityList.Remove(innerCam);
                entityList.Remove(follower);

            }
            //First calculate center available range with cylinder size and params
            double end = 0;
            double start= spiralY + camWidth / 2 + 5;
            if (innerCylinderForComponents.TotalHeight<spiralY+camWidth+5)
            {
                start = spiralY + camWidth / 2;
                end = spiralY + camWidth / 2 + 5;
                Rhino.RhinoApp.WriteLine("The given region of interest is too small! The following selection of position would exceed it. If it isn't what you want, you could reset the input region of interest.");
            }
            else
            {
                end = innerCylinderForComponents.TotalHeight - spiralY / 2;
            }
            //Calculate follower position candidates, let user pick.
            List<Point3d> followerPosCandidates = new List<Point3d>();
            for(double i=start;i<=end;i+=1)
            {
                Circle circle = innerCylinderForComponents.CircleAt(i);
                for(int j=0;j<24;j++)
                {
                    followerPosCandidates.Add(circle.PointAt(Math.PI / 12 * i));
                }
            }
            followerPosition = followerPosCandidates[UserSelection.UserSelectPointInRhino(followerPosCandidates, RhinoDoc.ActiveDoc)];
            Point3d CamCenter = innerCylinderForComponents.Center + innerCylinderForComponents.Axis * (innerCylinderForComponents.TotalHeight - spiralY / 2);
            //Then let user select the follower end position

            /*Cam c = new Cam(camType, camMinRadius, camMaxRadius, CamCenter, innerCylinderForComponents.Axis,)*/
            //Then set the center point for cam

            //Generate Cam

            //Let user select follower position,  if not selected.

            //
        }
        private void FixParameter()
        {
            spiralOuterRadius = innerCylinderForComponents.Radius * 0.95;
            
            spiralInnerRadius = Math.Max(2, innerCylinderForComponents.Radius * 0.1);
            axisRadius = spiralInnerRadius;
            roundNum = (int)Math.Ceiling(maxDegree / 360 / (spiralOuterRadius - spiralInnerRadius) * (2 * spiralOuterRadius + 4 * spiralInnerRadius));
            spiralX = (spiralOuterRadius - spiralInnerRadius) / roundNum * 0.4 * Math.Pow(strength, 0.25);
            spiralY = spiralX * Math.Pow(strength, 0.25) * 3;
            spiralY = Math.Min(innerCylinderForComponents.TotalHeight, spiralY);
            //Fix the parameters of cam & follower
            camMaxRadius = spiralOuterRadius;
            camMinRadius = spiralOuterRadius / 2;//TODO how could this be adjusted by user?
            camWidth = spiralY;
        }
        public void AdjustParam(double strengthNew,double displacementNew)
        {
            strength = strengthNew;
            maxDegree = displacementNew;
            Generate();
        }
        public void SetInputHandler()
        {

            //First let user select brep
            Rhino.Input.Custom.GetObject gb = new Rhino.Input.Custom.GetObject();
            gb.GeometryFilter = Rhino.DocObjects.ObjectType.Brep;
            gb.SetCommandPrompt("Please select the end effector brep by clicking. Press enter to skip.");
            gb.AcceptNothing(true);
            gb.Get();
            if (gb.CommandResult() != Rhino.Commands.Result.Success)
                return;
            else
            {
                Rhino.DocObjects.ObjRef objref = gb.Object(0);
                // get selected object
                Rhino.DocObjects.RhinoObject obj = objref.Object();
                if (obj == null)
                    return;
                // get selected brep
                Brep brep = objref.Brep();
                if (brep == null)
                    return;
                Transform rotateX = Transform.Rotation(mainDirection, Vector3d.XAxis, Point3d.Origin);
                Transform rotateBack = Transform.Rotation(Vector3d.XAxis, mainDirection, Point3d.Origin);
                Brep b = brep.DuplicateBrep();
                b.Transform(rotateX);
                BoundingBox bbox = b.GetBoundingBox(true);
                //bbox.Transform(rotateBack);
                b.Transform(rotateBack);
                List<Point3d> candidates = new List<Point3d>();
                for (double i = 0; i <= 1; i += 0.05)
                {
                    for (double j = 0; j <= 1; j += 0.05)
                    {
                        Line l = new Line(bbox.PointAt(0, i, j), bbox.PointAt(1, i, j));
                        l.Transform(rotateBack);
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
                inputHandler = new RodLike(b, centerAxis.Direction);
                entityList.Add(inputHandler);
                _ = new Fixation(inputHandler, centerAxis);
            }
        }
        public void SetEndEffector()
        {
            //First let user select brep
            Rhino.Input.Custom.GetObject gb=new Rhino.Input.Custom.GetObject();
            gb.GeometryFilter= Rhino.DocObjects.ObjectType.Brep;
            gb.SetCommandPrompt("Please select the end effector brep by clicking. Press enter to skip.");
            gb.AcceptNothing(true);
            gb.Get();
            if (gb.CommandResult() != Rhino.Commands.Result.Success)//If not successful, do nothing.
                return;
            else
            {
                if (endEffector != null)
                {
                    RemoveEntity(endEffector);
                    endEffector = null;
                }
                Rhino.DocObjects.ObjRef objref = gb.Object(0);
                // get selected object
                Rhino.DocObjects.RhinoObject obj = objref.Object();
                if (obj == null)
                    return;
                // get selected brep
                Brep brep = objref.Brep();
                if (brep == null)
                    return;
                Transform rotateX = Transform.Rotation(mainDirection, Vector3d.XAxis, Point3d.Origin);
                Transform rotateBack = Transform.Rotation(Vector3d.XAxis, mainDirection, Point3d.Origin);
                Brep b = brep.DuplicateBrep();
                b.Transform(rotateX);
                BoundingBox bbox = b.GetBoundingBox(true);
                //bbox.Transform(rotateBack);
                b.Transform(rotateBack);
                List<Point3d> candidates = new List<Point3d>();
                for (double i = 0; i <= 1; i += 0.05)
                {
                    for (double j = 0; j <= 1; j += 0.05)
                    {
                        Line l = new Line(bbox.PointAt(0, i, j), bbox.PointAt(1, i, j));
                        l.Transform(rotateBack);
                        Point3d[] pts;
                        Intersection.CurveBrep(l.ToNurbsCurve(), b, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out pts);
                        foreach (Point3d pt in pts)
                        {
                            candidates.Add(pt);
                        }
                    }
                }
                Point3d ContactPosition = candidates[UserSelection.UserSelectPointInRhino(candidates, RhinoDoc.ActiveDoc)];

                b.Transform(Transform.Translation(new Vector3d(centerAxis.Skeleton.PointAtStart) - new Vector3d(ContactPosition)));//TODO change the target position to follower end
                endEffector = new RodLike(b, centerAxis.Direction);
                entityList.Add(endEffector);
                _ = new Fixation(endEffector, centerAxis);
            }
            
        }
        public void RemoveLocks()
        {
            if (locks.Count == 0)
                return;
            RemoveEntity(locks[0]);
            RemoveEntity(locks[1]);
            locks.Clear();
        }
        public void ConstructLocks()
        {
            if(LockPosition==Point3d.Unset)
                SetLockPosition();
            if (locks.Count > 0)
                RemoveLocks();
            BoxLike b = new BoxLike(model, mainDirection);
            BoundingBox box = b.Bbox;
            //Build the locking structure. ITwould be easy
            Lock LockHead = new Lock(lockClosestPointOnAxis, mainDirection, lockDisToAxis);

            Point3d railS = LockPosition - knobDirection * 3 * axisRadius;
            Point3d railE = LockPosition + knobDirection * 3 * axisRadius;
            Line l = new Line(railS, railE);
            Brep LockBaseBrep = Brep.CreatePipe(l.ToNurbsCurve(), lockDisToAxis * 0.1, false, PipeCapMode.Round, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAngleToleranceRadians)[0];

            locks.Add(LockHead);
            locks.Add(new Lock(LockBaseBrep, false));
            entityList.Add(LockHead);
            entityList.Add(locks[1]);
            LockHead.RegisterOtherPart(locks[1]);
            _ = new Fixation(centerAxis, LockHead);
        }
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
            BoxLike b = new BoxLike(model, mainDirection);
            BoundingBox box = b.Bbox, targetBox = BoundingBox.Unset;
            Brep targetBoxBrep = null;
            //First tell the knob direction and get the target box
            if (knobDirection * mainDirection > 0)
                targetBox = new BoundingBox(box.PointAt(0.85, 0, 0), box.PointAt(1, 1, 1));
            else
                targetBox = new BoundingBox(box.PointAt(0, 0, 0), box.PointAt(0.15, 1, 1));
            targetBoxBrep = targetBox.ToBrep();
            targetBoxBrep.Transform(b.RotateBack);
            for (double i = 0; i <= 1; i += 0.05)
            {
                for (double j = 0; j <= 1; j += 0.05)
                {
                    Line l = new Line(box.PointAt(0, i, j), box.PointAt(1, i, j));
                    l.Transform(b.RotateBack);
                    Point3d[] pts;
                    Intersection.CurveBrep(l.ToNurbsCurve(), model, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out pts);
                    foreach (Point3d pt in pts)
                    {
                        if (targetBoxBrep.IsPointInside(pt, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, true) && pt.DistanceTo(targetBox.Center) > axisRadius * 2)
                            lockPosCandidates.Add(pt + knobDirection * 3);
                    }
                }
            }
            return lockPosCandidates;
        }
        public List<Brep> GetModel(List<string> items)
        {
            List<Brep> models = new List<Brep>();
            if (items.Contains("Spiral"))
                models.Add(spiralSpring.GetModelinWorldCoordinate());
            if (items.Contains("Axis"))
                models.Add(centerAxis.GetModelinWorldCoordinate());
            if (items.Contains("BaseModel"))
                models.Add(model);
            if (items.Contains("Locks"))
            {
                models.Add(locks[0].GetModelinWorldCoordinate());
                models.Add(locks[1].GetModelinWorldCoordinate());
            }
            if (items.Contains("EndEffector"))
                models.Add(endEffector.GetModelinWorldCoordinate());
            return models;
        }
    }
}
