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
using KinergyUtilities;
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
        double energy_old;
        int maxDegree;
        int energy;
        int displacement;
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

        private RhinoDoc myDoc;
        int midPartIdx;
        List<int> lockPartIdx;
        Brep midPartBackup;
        /// <summary>
        /// Old constructor
        /// </summary>
        /// <param name="InputModel"></param>
        /// <param name="mainDirection"></param>
        /// <param name="innerCylinder"></param>
        /// <param name="KineticStrength"></param>
        /// <param name="MaxLoadingDegree"></param>
        //public InstantRotation(Brep InputModel,Vector3d mainDirection,Brep innerCylinder,double KineticStrength,double MaxLoadingDegree)
        //{
        //    model = InputModel;
        //    direction = mainDirection;
        //    innerCylinderForComponents = GetCylinder(innerCylinder,mainDirection);
        //    energy_old = KineticStrength;
        //    maxDegree = MaxLoadingDegree;
        //    BoxLike currB = new BoxLike(model, direction);
        //    skeleton = currB.Skeleton;
        //    skeleton.Transform(currB.RotateBack);
        //    skeletonLen = skeleton.PointAtNormalizedLength(0).DistanceTo(skeleton.PointAtNormalizedLength(1));
        //    GenerateSpiralAndAxis();
        //    entityList.Add(new Shape(model));

        //    myDoc = RhinoDoc.ActiveDoc;
        //}
        public InstantRotation(Brep InputModel,Vector3d mainDirection,  Brep innerCylinder,int e,int disp,bool fornothing)
        {
            model = InputModel;
            direction = mainDirection;
            innerCylinderForComponents = GetCylinder(innerCylinder, mainDirection);
            energy = e;
            displacement = disp;//as degrees
            maxDegree = disp;
            BoxLike currB = new BoxLike(model, direction);
            skeleton = currB.Skeleton;
            skeleton.Transform(currB.RotateBack);
            skeletonLen = skeleton.PointAtNormalizedLength(0).DistanceTo(skeleton.PointAtNormalizedLength(1));

            myDoc = RhinoDoc.ActiveDoc;
            midPartIdx = -1;
            lockPartIdx = new List<int>();
            locks = new List<Lock>();
            midPartBackup = new Brep();
            //GenerateSpiralAndAxis_New();
            //entityList.Add(new Shape(model));
        }
        //private void GenerateSpiralAndAxis()
        //{
        //    FixParameter();
        //    //use the given cylinder to get center
        //    spiralSpring = new Spiral(innerCylinderForComponents.Center, direction, spiralOuterRadius, spiralInnerRadius, roundNum, spiralX, spiralY);
        //    BoxLike b = new BoxLike(model, direction);
        //    BoundingBox box = b.Bbox;
        //    box.Transform(b.RotateBack);
        //    if(knobDirection==Vector3d.Unset)
        //    {
        //        //Let user do the selection
        //        Arrow a1 = new Arrow(innerCylinderForComponents.Axis, box.PointAt(1, 0.5, 0.5),1);
        //        Arrow a2 = new Arrow(-innerCylinderForComponents.Axis, box.PointAt(0, 0.5, 0.5), 1);
        //        List<Curve> arrowCurves = new List<Curve>();
        //        arrowCurves.Add(a1.ArrowCurve);
        //        arrowCurves.Add(a2.ArrowCurve);
        //        int result=UserSelection.UserSelectCurveInRhino(arrowCurves, RhinoDoc.ActiveDoc);
        //        if (result == 0)
        //            knobDirection = direction;
        //        else
        //            knobDirection = -direction;
        //    }
        //    //Then the centerAxis is generated with knob. Be careful with the direction
            
        //    Point3d skStart = skeleton.PointAtNormalizedLength(0);
        //    Point3d skEnd = skeleton.PointAtNormalizedLength(1);
        //    double intervalStart = 0, intervalEnd = 0;
        //    if(knobDirection*direction>0)
        //    {
        //        intervalStart =0;
        //        intervalEnd = (new Vector3d(skEnd) - new Vector3d(innerCylinderForComponents.Center)) * direction / direction.Length+axisRadius*5;
        //    }
        //    else
        //    {
        //        intervalStart = 0;
        //        intervalEnd = -(new Vector3d(skStart) - new Vector3d(innerCylinderForComponents.Center)) * direction / direction.Length+axisRadius*5;
        //    }
        //    centerAxis = new RodLike(innerCylinderForComponents.Center, axisRadius, knobDirection, new Interval(intervalStart,intervalEnd),false);
        //    centerAxis.AddKnob(1,axisRadius*2,axisRadius*2);
        //    entityList.Add(spiralSpring);
        //    entityList.Add(centerAxis);
        //    _=new Fixation(centerAxis, spiralSpring);
        //    _ = new Fixation(centerAxis, endEffector);
        //}
        public void GenerateSpiralAndAxis(Guid eeID, List<Brep> brepCut, bool isSpringCW)
        {
            #region Step 1: Obtain the middle part 

            Point3d midPrtS = skeleton.PointAtNormalizedLength(0);
            Point3d midPrtE = skeleton.PointAtNormalizedLength(1);
            Point3d p_ee = new Point3d();
            Point3d p_stationary = new Point3d();

            Brep stationary_brep = new Brep();
            Brep ee_brep = (Brep)myDoc.Objects.Find(eeID).Geometry;
            Plane pl = Plane.Unset;
            int stationaryIdx = 0;

            int eeFlag = -1;
        
            if (ee_brep.ClosestPoint(midPrtS).DistanceTo(midPrtS) <= ee_brep.ClosestPoint(midPrtE).DistanceTo(midPrtE))
            {
                // the end-effector is near the start point of the middle part
                p_ee = midPrtS;
                p_stationary = midPrtE;

                stationary_brep = brepCut[0];
                stationaryIdx = 0;

                if (t1 >= t2)
                {
                    pl = new Plane(skeleton.PointAtNormalizedLength(t2 + 2 / skeletonLen), -direction);
                }
                else
                {
                    pl = new Plane(skeleton.PointAtNormalizedLength(t1 + 2 / skeletonLen), -direction);
                }

                eeFlag = 2;
            }
            else
            {
                // the end-effector is near the end point of the middle part
                p_ee = midPrtE;
                p_stationary = midPrtS;

                stationary_brep = brepCut[2];
                stationaryIdx = 2;

                if (t1 >= t2)
                {
                    pl = new Plane(skeleton.PointAtNormalizedLength(t1 - 2 / skeletonLen), direction);
                }
                else
                {
                    pl = new Plane(skeleton.PointAtNormalizedLength(t2 - 2 / skeletonLen), direction);
                }

                eeFlag = 1;
            }

            Brep b2Cut1 = b2.Trim(pl, myDoc.ModelAbsoluteTolerance)[0];
            b2Cut1 = b2Cut1.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            b2Cut1.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == b2Cut1.SolidOrientation)
                b2Cut1.Flip();

            Brep b2Outside = b2Cut1.DuplicateBrep();
            Brep b2Inside = Brep.CreateOffsetBrep(b2Cut1, -2, false, true, myDoc.ModelAbsoluteTolerance, out _, out _)[0];

            b2Inside.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == b2Inside.SolidOrientation)
                b2Inside.Flip();

            Point3d axisStart = new Point3d();
            Point3d axisEnd = new Point3d();
            Point3d deductStartPt = new Point3d();
            Point3d deductEndPt = new Point3d();
            Vector3d interPlnDir = new Vector3d();

            if(eeFlag == 2)
            {
                axisStart = skeleton.PointAtNormalizedLength(t2 > t1 ? t2 : t1);
                axisEnd = skeleton.PointAtNormalizedLength(t2 > t1 ? t1 : t2);
                deductStartPt = (axisStart + axisEnd) / 2;
                deductEndPt = deductStartPt - direction * axisStart.DistanceTo(axisEnd);
                interPlnDir = -direction;
            }
            else
            {
                axisStart = skeleton.PointAtNormalizedLength(t2 > t1 ? t1 : t2);
                axisEnd = skeleton.PointAtNormalizedLength(t2 > t1 ? t2 : t1);
                deductStartPt = (axisStart + axisEnd) / 2;
                deductEndPt = deductStartPt + direction * axisStart.DistanceTo(axisEnd);
                interPlnDir = direction;
            }

            Curve deductCrv = new Line(deductStartPt, deductEndPt).ToNurbsCurve();
            Brep deductBrep = Brep.CreatePipe(deductCrv, 2.8, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            deductBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == deductBrep.SolidOrientation)
                deductBrep.Flip();

            Brep b2InsideBig = Brep.CreateOffsetBrep(b2, -2, false, true, myDoc.ModelAbsoluteTolerance, out _, out _)[0];
            b2InsideBig.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == b2InsideBig.SolidOrientation)
                b2InsideBig.Flip();

            Point3d[] interPts;
            Intersection.CurveBrep(deductCrv, b2Inside, myDoc.ModelAbsoluteTolerance, out _, out interPts);
            Point3d interPt = interPts[0];
            Plane interPln = new Plane(interPt, interPlnDir);
            Curve[] interCrvs;
            Intersection.BrepPlane(b2Inside, interPln, myDoc.ModelAbsoluteTolerance, out interCrvs, out _);
            Curve interCrv = interCrvs[0];
            double inter_t = -1;
            interCrv.ClosestPoint(interPt, out inter_t);
            Point3d crvInterPt = interCrv.PointAt(inter_t);

            double moonRad = crvInterPt.DistanceTo(interPt) - 1;

            //Curve moonCylinderCrv = new Line(deductStartPt, interPt + interPlnDir * 5).ToNurbsCurve();
            Curve moonCylinderCrv = deductCrv;
            Brep moonCylinderBrep = Brep.CreatePipe(moonCylinderCrv, moonRad, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            moonCylinderBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == moonCylinderBrep.SolidOrientation)
                moonCylinderBrep.Flip();

            Vector3d moonRectVec = crvInterPt - interPt;
            moonRectVec.Unitize();
            Vector3d moonRectExpVec = moonRectVec;
            Transform tr = Transform.Rotation(Math.PI / 2, interPlnDir, interPt);
            moonRectExpVec.Transform(tr);

            double overhangWidthOffset = 4;

            Point3d moonRect0 = deductStartPt + moonRectVec * (moonRad + 11) + moonRectExpVec * overhangWidthOffset;
            Point3d moonRect1 = deductStartPt + moonRectVec * (moonRad + 11) - moonRectExpVec * overhangWidthOffset;
            Point3d moonRect2 = deductStartPt - moonRectVec * (moonRad + 11) - moonRectExpVec * overhangWidthOffset;
            Point3d moonRect3 = deductStartPt - moonRectVec * (moonRad + 11) + moonRectExpVec * overhangWidthOffset;
            Point3d moonRect4 = moonRect0;

            List<Point3d> moonRectCorners = new List<Point3d>();
            moonRectCorners.Add(moonRect0);
            moonRectCorners.Add(moonRect1);
            moonRectCorners.Add(moonRect2);
            moonRectCorners.Add(moonRect3);
            moonRectCorners.Add(moonRect4);

            Polyline moonRect = new Polyline(moonRectCorners);
            Curve moonRectCrv = moonRect.ToNurbsCurve();

            var sweep = new SweepOneRail();
            sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            sweep.ClosedSweep = false;
            sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            Brep[] moonRectBreps = sweep.PerformSweep(moonCylinderCrv, moonRectCrv);
            Brep moonRectBrep = moonRectBreps[0];
            Brep moonRectDeduct = moonRectBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            moonRectDeduct.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == moonRectDeduct.SolidOrientation)
                moonRectDeduct.Flip();

            Brep[] moonBreps = Brep.CreateBooleanDifference(moonCylinderBrep, moonRectDeduct, myDoc.ModelAbsoluteTolerance);
            Brep moonBrep1 = moonBreps[0];
            Brep moonBrep2 = moonBreps[1];
            moonBrep1.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == moonBrep1.SolidOrientation)
                moonBrep1.Flip();
            moonBrep2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == moonBrep2.SolidOrientation)
                moonBrep2.Flip();
 

            Brep b2DeductBrep = Brep.CreateBooleanUnion(new List<Brep> { b2Inside, deductBrep, moonBrep1, moonBrep2 }, myDoc.ModelAbsoluteTolerance)[0];
            b2DeductBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == b2DeductBrep.SolidOrientation)
                b2DeductBrep.Flip();

            b2Inside = Brep.CreateBooleanDifference(b2Outside, b2DeductBrep, myDoc.ModelAbsoluteTolerance)[0];
            b2Inside.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == b2Inside.SolidOrientation)
                b2Inside.Flip();

            //myDoc.Objects.AddBrep(b2Inside);
            //myDoc.Views.Redraw();


            #endregion

            #region Step 2: create the central part (the central axis)

            var s = b2Inside.IsSolid;
            Brep midBrep = b2Inside.DuplicateBrep();

            if(eeFlag == 2)
            {
                MidPart = new RodLike(midBrep, -direction);
                basePart = new RodLike(b1, -direction);
                endEffector = new RodLike(b3, -direction);

                BoundingBox bbox = MidPart.Model.GetBoundingBox(true);
                //Point3d axisStart = bbox.Center + direction / direction.Length * (bbox.Max.X - bbox.Min.X) / 2;
                //Point3d axisEnd = bbox.Center - direction / direction.Length * ((bbox.Max.X - bbox.Min.X) / 2 + 2.5);

                entityList.Add(new Socket(axisStart - direction / direction.Length * 2.5, -direction));

                Cylinder c1 = new Cylinder(new Circle(new Plane(axisStart, -direction), 4));
                c1.Height1 = 1;
                c1.Height2 = 2;

                Cylinder c2 = new Cylinder(new Circle(new Plane(axisStart, -direction), 2));
                c2.Height1 = 1;
                c2.Height2 = axisStart.DistanceTo(axisEnd);

                Brep axisB = c1.ToBrep(true, true);
                axisB.Append(c2.ToBrep(true, true));

                centerAxis = new RodLike(axisB, -direction);
                centerAxis.Skeleton = new Line(axisStart, axisEnd).ToNurbsCurve();
                entityList.Add(centerAxis);

                Point3d cavStart = axisStart + direction / direction.Length * 5;
                Point3d cavEnd = axisStart - direction / direction.Length * 10;
                Curve cavPath = new Line(cavStart, cavEnd).ToNurbsCurve();
                Brep cavBrep = Brep.CreatePipe(cavPath, 10, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

                cavBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == cavBrep.SolidOrientation)
                    cavBrep.Flip();

                MidPart.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == MidPart.Model.SolidOrientation)
                    MidPart.Model.Flip();

                Brep newMidModel = Brep.CreateBooleanDifference(MidPart.Model, cavBrep, myDoc.ModelAbsoluteTolerance)[0];
                newMidModel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == newMidModel.SolidOrientation)
                    newMidModel.Flip();

                MidPart.Model = newMidModel;

                //myDoc.Objects.AddBrep(centerAxis.GetModelinWorldCoordinate());
                //myDoc.Views.Redraw();
            }
            else
            {
                MidPart = new RodLike(midBrep, direction);
                basePart = new RodLike(b3, direction);
                endEffector = new RodLike(b1, direction);

                BoundingBox bbox = MidPart.Model.GetBoundingBox(true);
                //Point3d axisStart = bbox.Center - direction / direction.Length * (bbox.Max.X - bbox.Min.X) / 2;
                //Point3d axisEnd = bbox.Center + direction / direction.Length * ((bbox.Max.X - bbox.Min.X) / 2 + 2.5);

                entityList.Add(new Socket(axisStart + direction/ direction.Length * 2.5, direction));

                Cylinder c1 = new Cylinder(new Circle(new Plane(axisStart, direction), 4));
                c1.Height1 = 1;
                c1.Height2 = 2;

                Cylinder c2 = new Cylinder(new Circle(new Plane(axisStart, direction), 2));
                c2.Height1 = 1;
                c2.Height2 = axisStart.DistanceTo(axisEnd);

                Brep axisB = c1.ToBrep(true, true);
                axisB.Append(c2.ToBrep(true, true));
                centerAxis = new RodLike(axisB, direction);
                centerAxis.Skeleton = new Line(axisStart, axisEnd).ToNurbsCurve();
                entityList.Add(centerAxis);

                Point3d cavStart = axisStart - direction / direction.Length * 5;
                Point3d cavEnd = axisStart + direction / direction.Length * 10;
                Curve cavPath = new Line(cavStart, cavEnd).ToNurbsCurve();
                Brep cavBrep = Brep.CreatePipe(cavPath, 10, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

                cavBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == cavBrep.SolidOrientation)
                    cavBrep.Flip();

                MidPart.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == MidPart.Model.SolidOrientation)
                    MidPart.Model.Flip();

                Brep newMidModel = Brep.CreateBooleanDifference(MidPart.Model, cavBrep, myDoc.ModelAbsoluteTolerance)[0];
                newMidModel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == newMidModel.SolidOrientation)
                    newMidModel.Flip();

                MidPart.Model = newMidModel;

                //myDoc.Objects.AddBrep(centerAxis.GetModelinWorldCoordinate());
                //myDoc.Views.Redraw();
            }

            
            entityList.Add(basePart);
            entityList.Add(endEffector);
            entityList.Add(MidPart);
            midPartIdx = entityList.Count - 1;
            #endregion

            #region Step 3: create the central part (the spiral spring)

            Point3d springCen = new Point3d();
            Point3d pt_temp = new Point3d();
            Point3d pt_far = new Point3d();
            if (eeFlag == 2)
            {
                axisStart = skeleton.PointAtNormalizedLength(t2 > t1 ? t2 : t1);

                pt_temp = skeleton.PointAtNormalizedLength(t2 > t1 ? t2 : t1);
                pt_far = skeleton.PointAtNormalizedLength(t2 > t1 ? t1 : t2);
                springCen = (pt_temp - direction / direction.Length * 10).DistanceTo(pt_temp) < ((pt_temp + pt_far) / 2).DistanceTo(pt_temp)?
                                (pt_temp - direction / direction.Length * 10) : ((pt_temp + pt_far) / 2);
                
                spiralSpring = new Spiral(axisStart, -direction, springCen, innerCylinderForComponents.Radius - 0.5, isSpringCW, maxDegree, energy);

                // generate the spring end connector

            }
            else
            {
                axisStart = skeleton.PointAtNormalizedLength(t2 > t1 ? t1 : t2);

                pt_temp = skeleton.PointAtNormalizedLength(t2 > t1 ? t1 : t2);
                pt_far = skeleton.PointAtNormalizedLength(t2 > t1 ? t2 : t1);
                springCen = (pt_temp + direction / direction.Length * 10).DistanceTo(pt_temp) < ((pt_temp + pt_far) / 2).DistanceTo(pt_temp)?
                               (pt_temp + direction / direction.Length * 10) : ((pt_temp + pt_far) / 2);
                spiralSpring = new Spiral(axisStart, direction, springCen, innerCylinderForComponents.Radius - 0.5, isSpringCW, maxDegree, energy);
            }
            entityList.Add(spiralSpring);

            //Register relation
            _ = new Fixation(spiralSpring, centerAxis);
            _ = new Fixation(spiralSpring, MidPart);
            _ = new Fixation(centerAxis, endEffector);
            _ = new Fixation(MidPart, basePart);

            #endregion

            #region Step 4: prepare the lock position candidates

            lockPosCandidates = new List<Point3d>();
            Point3d centerP = (pt_far + springCen) / 2.0;
            Plane plane = new Plane(centerP, direction);
            Curve[] c;
            Point3d[] pt;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(b2, plane, myDoc.ModelAbsoluteTolerance, out c, out pt);
            for (int j = 0; j < 10; j++)
            {
                lockPosCandidates.Add(c[0].PointAtNormalizedLength(j * 0.1));
            }

            #endregion

        }
        public void Set3Parts(double T1,double T2,Brep B1,Brep B2,Brep B3)
        {
            t1 = T1;
            t2 = T2;
            b1 = B1;
            b2 = B2;
            b3 = B3;
        }
        //private void FixParameter()
        //{
        //    spiralOuterRadius = innerCylinderForComponents.Radius * 0.95;
        //    spiralInnerRadius = Math.Max(2, innerCylinderForComponents.Radius * 0.1);
        //    axisRadius = spiralInnerRadius;
        //    roundNum = (int)Math.Ceiling(maxDegree / 360 / (spiralOuterRadius - spiralInnerRadius) * (2 * spiralOuterRadius + 4 * spiralInnerRadius));
        //    spiralX = (spiralOuterRadius - spiralInnerRadius) / roundNum * 0.4 * Math.Pow(energy_old, 0.25);
        //    spiralY= spiralX * Math.Pow(energy_old, 0.25)*3;
        //    spiralY = Math.Min(innerCylinderForComponents.TotalHeight, spiralY);
        //}
        //public void AdjustParameter(double KineticStrength, double MaxLoadingDegree)
        //{
        //    energy_old = KineticStrength;
        //    maxDegree = MaxLoadingDegree;
        //    GenerateSpiralAndAxis();
            
        //}
        public void AdjustParameter(int KineticStrength, int MaxLoadingDegree, bool isSpringCW)
        {
            maxDegree = MaxLoadingDegree;
            //GenerateSpiralAndAxis(); 
            List<Point3d> lockPos = new List<Point3d>();
            bool spiralLockNorm = false;
            Vector3d spiralLockDir = new Vector3d();
            spiralSpring.AdjustParam(direction, null, model, 0, KineticStrength, maxDegree, isSpringCW, ref lockPos, ref spiralLockNorm, ref spiralLockDir);
            energy = KineticStrength;
        }
      
        public void SetLockPosition()
        {
            //First calculate lock pos candidates
            //With knob direction given, we could calculate the box that lock pos should be in.
            if(lockDisToAxis==0)//When lock pos is never set before
            {
                LockPosition = lockPosCandidates[UserSelection.UserSelectPointInRhino(lockPosCandidates, myDoc)];
                double t = 0;
                //Use the axis instead!
                centerAxis.Skeleton.ClosestPoint(LockPosition, out t);
                t = centerAxis.Skeleton.Domain.NormalizedParameterAt(t);
                lockClosestPointOnAxis = centerAxis.Skeleton.PointAtNormalizedLength(t);
                lockDisToAxis = LockPosition.DistanceTo(lockClosestPointOnAxis);
            }
        }

        public void RemoveLocks()
        {
            if(lockPartIdx.Count > 0)
            {
                // retrack the midpart
                MidPart.Model = midPartBackup;

                // delete all the entities that have already been registered
                for (int i = lockPartIdx.Count-1; i>=0; i--)
                {
                    int pos = lockPartIdx.ElementAt(i);
                    Lock l = (Lock) entityList.ElementAt(pos);
                    l.ClearConstraints();
                    entityList.RemoveAt(pos);
                }
                lockPartIdx.Clear();
                locks.Clear();
                lockDisToAxis = 0;

                // re-registered the new entities
                entityList.Add(MidPart);
                midPartIdx = entityList.Count - 1;

            }

        }
        public void updateLock(bool dir)
        {
            Lock LockHead;
            double ratchetRadius = lockDisToAxis * 0.5;

            if (dir)
                LockHead = new Lock(lockClosestPointOnAxis, direction, ratchetRadius, false);
            else
                LockHead = new Lock(lockClosestPointOnAxis, direction, ratchetRadius, true);

            // update entitylist
            int delPos = entityList.Count - 2;
            entityList.RemoveAt(delPos);
            entityList.Insert(delPos, LockHead);

            // update locks
            locks.RemoveAt(0);
            locks.Insert(0, LockHead);
            LockHead.RegisterOtherPart(locks.ElementAt(1));
            _ = new Fixation(centerAxis, LockHead);

        }
        public void ConstructLocks(bool dir)
        {
            SetLockPosition();
            
            if (lockPartIdx.Count > 0)
            {
                for (int i = lockPartIdx.Count - 1; i >= 0; i--)
                {
                    entityList.RemoveAt(i);
                }
                midPartIdx = -1;
                lockPartIdx.Clear();
            }
                
            if (locks.Count > 0)
                locks.Clear();

            Lock LockHead;
            double ratchetRadius = lockDisToAxis * 0.5;
            //if (EEDirection==2)
            if(dir)
                LockHead = new Lock(lockClosestPointOnAxis, direction, ratchetRadius, false);
            else
                LockHead = new Lock(lockClosestPointOnAxis, direction, ratchetRadius, true);

            //myDoc.Objects.AddBrep(LockHead.GetModelinWorldCoordinate());
            //myDoc.Views.Redraw();

            Vector3d centerLinkDirection = new Vector3d(lockClosestPointOnAxis) - new Vector3d(LockPosition);
            double centerLinkLen = centerLinkDirection.Length;
            centerLinkDirection.Unitize();
            #region generate lock handle -old code, now moved inside lock constructor
            //#region create the handler

            //double handlerDis = Math.Max(20, ratchetRadius);
            //double handlerThickness = 3;
            //double handlerRadius = 5;
            //Point3d handlerPos = LockPosition - centerLinkDirection * handlerDis;
            //Point3d handlerEndPos = handlerPos - centerLinkDirection * handlerThickness;
            //Curve handlerTraj = new Line(handlerPos, handlerEndPos).ToNurbsCurve();
            //Brep handler = Brep.CreatePipe(handlerTraj, handlerRadius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            //handler.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == handler.SolidOrientation)
            //    handler.Flip();

            //#endregion

            //#region create the central axis and the piston

            //double axisLen = 0;
            //double latchClearance = 4;
            //double tipLen = 2;
            //if (ratchetRadius < (tipLen + latchClearance)) return;
            //axisLen = ratchetRadius - (tipLen + latchClearance) + handlerDis;

            //double axisRadius = 1;
            //Point3d axisEnd = handlerPos;
            //Point3d axisStart = axisEnd + centerLinkDirection * axisLen;
            //Curve axisTraj = new Line(axisStart, axisEnd).ToNurbsCurve();
            //Brep axisRodBrep = Brep.CreatePipe(axisTraj, axisRadius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            //axisRodBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == axisRodBrep.SolidOrientation)
            //    axisRodBrep.Flip();

            //Transform rot = Transform.Rotation(Math.PI / 2, centerLinkDirection, axisStart);
            //Vector3d tipDir = direction;
            //tipDir.Transform(rot);
            //tipDir.Unitize();
            //double tipBig = 2.5;
            //double tipSmall = 1;

            //var sweep = new SweepOneRail();
            //sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            //sweep.ClosedSweep = false;
            //sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            //Point3d tipPt0 = axisStart + tipDir * axisRadius + direction/direction.Length * tipBig;
            //Point3d tipPt1 = axisStart + tipDir * axisRadius + centerLinkDirection * tipLen + direction / direction.Length * tipSmall;
            //Point3d tipPt2 = axisStart + tipDir * axisRadius + centerLinkDirection * tipLen - direction / direction.Length * tipSmall;
            //Point3d tipPt3 = axisStart + tipDir * axisRadius - direction / direction.Length * tipBig;
            //Point3d tipPt4 = tipPt0;

            //List<Point3d> tipCorners = new List<Point3d>();
            //tipCorners.Add(tipPt0);
            //tipCorners.Add(tipPt1);
            //tipCorners.Add(tipPt2);
            //tipCorners.Add(tipPt3);
            //tipCorners.Add(tipPt4);

            //Polyline tipRect = new Polyline(tipCorners);
            //Curve tipRectCrv = tipRect.ToNurbsCurve();

            //Point3d tipPathStart = axisStart + tipDir * axisRadius;
            //Point3d tipPathEnd = axisStart - tipDir * axisRadius;
            //Curve tipTraj = new Line(tipPathStart, tipPathEnd).ToNurbsCurve();

            //Brep[] tipBreps = sweep.PerformSweep(tipTraj, tipRectCrv);
            //Brep tipBrep = tipBreps[0];
            //Brep tip = tipBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //tip.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == tip.SolidOrientation)
            //    tip.Flip();

            //Brep centralAxis = Brep.CreateBooleanUnion(new List<Brep> { axisRodBrep, tip }, myDoc.ModelAbsoluteTolerance)[0];

            //centralAxis.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == centralAxis.SolidOrientation)
            //    centralAxis.Flip();

            //#endregion

            //#region create the beams

            //double clearance = 0.6;

            //Point3d axisMidPoint = (axisStart + axisEnd) / 2.0;
            //double beamDis = axisLen / 4.0;
            //double beamOffset = 3;
            //double beamWidthOffset = 1;
            //double beamThickness = 2;
            //double beamLeverThickness = 1;
            //Point3d beamPos = axisMidPoint - centerLinkDirection * beamDis;

            //Point3d beamHolderPt0 = beamPos + tipDir * beamOffset + direction / direction.Length * beamWidthOffset;
            //Point3d beamHolderPt1 = beamPos + tipDir * beamOffset - direction / direction.Length * beamWidthOffset;
            //Point3d beamHolderPt2 = beamPos + tipDir * beamOffset - direction / direction.Length * beamWidthOffset - centerLinkDirection * beamThickness;
            //Point3d beamHolderPt3 = beamPos + tipDir * beamOffset + direction / direction.Length * beamWidthOffset - centerLinkDirection * beamThickness;
            //Point3d beamHolderPt4 = beamHolderPt0;

            //List<Point3d> beamCorners = new List<Point3d>();
            //beamCorners.Add(beamHolderPt0);
            //beamCorners.Add(beamHolderPt1);
            //beamCorners.Add(beamHolderPt2);
            //beamCorners.Add(beamHolderPt3);
            //beamCorners.Add(beamHolderPt4);

            //Polyline beamRect = new Polyline(beamCorners);
            //Curve beamRectCrv = beamRect.ToNurbsCurve();

            //Point3d beamPathStart = beamPos + tipDir * beamOffset;
            //Point3d beamPathEnd = beamPos - tipDir * beamOffset;
            //Curve beamTraj = new Line(beamPathStart, beamPathEnd).ToNurbsCurve();

            //Brep[] beamHolderBreps = sweep.PerformSweep(beamTraj, beamRectCrv);
            //Brep beamHolderBrep = beamHolderBreps[0];
            //Brep beamHolder = beamHolderBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //beamHolder.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == beamHolder.SolidOrientation)
            //    beamHolder.Flip();

            //double hookRadius = 1;
            //double notchRadius = 1.6;
            //double moveDis = tipLen + latchClearance + 2 * 0.6 - 2 * hookRadius;

            //Point3d notchCenFirst = axisMidPoint + tipDir * beamOffset + centerLinkDirection * hookRadius;
            //Point3d notchCenSecond = notchCenFirst + centerLinkDirection * (2 * hookRadius + moveDis);
            //double beamLeverLen = axisLen / 4.0 + 2 * hookRadius;

            //Point3d beamLeverStart = beamHolderPt0;
            //Point3d beamLeverEnd = beamHolderPt0 + centerLinkDirection * beamLeverLen;
            //Curve beamLeverTraj = new Line(beamLeverStart, beamLeverEnd).ToNurbsCurve();

            //Point3d beamDetentStart = beamHolderPt0 + centerLinkDirection * (axisLen / 4.0);
            //Point3d beamDetentEnd = beamDetentStart + centerLinkDirection * 2 * hookRadius;
            //Curve beamDetentTraj = new Line(beamDetentStart, beamDetentEnd).ToNurbsCurve();

            //Point3d beamLeverRPt0 = beamPos + tipDir * beamOffset + direction / direction.Length * beamWidthOffset;
            //Point3d beamLeverRPt1 = beamPos + tipDir * beamOffset - direction / direction.Length * beamWidthOffset;
            //Point3d beamLeverRPt2 = beamPos + tipDir * (beamOffset - beamLeverThickness) - direction / direction.Length * beamWidthOffset;
            //Point3d beamLeverRPt3 = beamPos + tipDir * (beamOffset - beamLeverThickness) + direction / direction.Length * beamWidthOffset;
            //Point3d beamLeverRPt4 = beamLeverRPt0;

            //Point3d beamDetentRPt0 = beamLeverRPt0 + centerLinkDirection * (axisLen / 4.0);
            //Point3d beamDetentRPt1 = beamLeverRPt1 + centerLinkDirection * (axisLen / 4.0);
            //Point3d beamDetentRPt2 = beamPos + tipDir * (beamOffset - hookRadius) - direction / direction.Length * beamWidthOffset + centerLinkDirection * (axisLen / 4.0);
            //Point3d beamDetentRPt3 = beamPos + tipDir * (beamOffset - hookRadius) + direction / direction.Length * beamWidthOffset + centerLinkDirection * (axisLen / 4.0);
            //Point3d beamDetentRPt4 = beamDetentRPt0;

            //List<Point3d> beamLeverRCorners = new List<Point3d>();
            //beamLeverRCorners.Add(beamLeverRPt0);
            //beamLeverRCorners.Add(beamLeverRPt1);
            //beamLeverRCorners.Add(beamLeverRPt2);
            //beamLeverRCorners.Add(beamLeverRPt3);
            //beamLeverRCorners.Add(beamLeverRPt4);

            //List<Point3d> beamDetentRCorners = new List<Point3d>();
            //beamDetentRCorners.Add(beamDetentRPt0);
            //beamDetentRCorners.Add(beamDetentRPt1);
            //beamDetentRCorners.Add(beamDetentRPt2);
            //beamDetentRCorners.Add(beamDetentRPt3);
            //beamDetentRCorners.Add(beamDetentRPt4);

            //Polyline beamLeverRRect = new Polyline(beamLeverRCorners);
            //Curve beamLeverRRectCrv = beamLeverRRect.ToNurbsCurve();

            //Polyline beamDetentRRect = new Polyline(beamDetentRCorners);
            //Curve beamDetentRRectCrv = beamDetentRRect.ToNurbsCurve();

            //Brep[] beamLeverRBreps = sweep.PerformSweep(beamLeverTraj, beamLeverRRectCrv);
            //Brep beamLeverRBrep = beamLeverRBreps[0];
            //Brep beamLeverR = beamLeverRBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //beamLeverR.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == beamLeverR.SolidOrientation)
            //    beamLeverR.Flip();

            //Brep[] beamDetentRBreps = sweep.PerformSweep(beamDetentTraj, beamDetentRRectCrv);
            //Brep beamDetentRBrep = beamDetentRBreps[0];
            //Brep beamDetentR = beamDetentRBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //beamDetentR.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == beamDetentR.SolidOrientation)
            //    beamDetentR.Flip();

            //Sphere beamDetentRSphere = new Sphere(notchCenFirst, hookRadius);
            //Brep beamDetentRSphereBrep = beamDetentRSphere.ToBrep();
            //beamDetentRSphereBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == beamDetentRSphereBrep.SolidOrientation)
            //    beamDetentRSphereBrep.Flip();

            //Brep beamDetentRNotch = Brep.CreateBooleanDifference(beamDetentRSphereBrep, beamDetentR, myDoc.ModelAbsoluteTolerance)[0];
            //beamDetentRNotch.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == beamDetentRNotch.SolidOrientation)
            //    beamDetentRNotch.Flip();

            //Brep beamRight = Brep.CreateBooleanUnion(new List<Brep> { beamLeverR, beamDetentRNotch }, myDoc.ModelAbsoluteTolerance)[0];
            //beamRight.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == beamRight.SolidOrientation)
            //    beamRight.Flip();

            //Brep beamLeft = beamRight.DuplicateBrep();
            //Transform mirrorTrans = Transform.Mirror(LockPosition, tipDir);
            //beamLeft.Transform(mirrorTrans);
            //beamLeft.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == beamLeft.SolidOrientation)
            //    beamLeft.Flip();

            //Brep beam = Brep.CreateBooleanUnion(new List<Brep> { beamHolder, beamRight, beamLeft }, myDoc.ModelAbsoluteTolerance)[0];

            //#endregion

            //#region create the barrel

            //Point3d barrelStart = (axisMidPoint + beamPos) / 2.0;
            //Point3d barrelEnd = barrelStart + centerLinkDirection * (barrelStart.DistanceTo(axisMidPoint) + 4 * hookRadius + moveDis + 2);
            //Curve barrelTraj = new Line(barrelStart, barrelEnd).ToNurbsCurve();

            //Point3d barrelHollowStart = barrelStart + centerLinkDirection * barrelStart.DistanceTo(axisMidPoint);
            //Point3d barrelHollowEnd = barrelHollowStart + centerLinkDirection * (4 * hookRadius + moveDis);
            //Curve barrelHollowTraj = new Line(barrelHollowStart, barrelHollowEnd).ToNurbsCurve();

            //double barrelThickness = 1.2;
            //double barrelRadius = beamOffset + clearance + barrelThickness;

            //double barrelCylinderRadius = barrelRadius - 0.2;

            //Brep barrelBrep = Brep.CreatePipe(barrelTraj, barrelRadius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            //barrelBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == barrelBrep.SolidOrientation)
            //    barrelBrep.Flip();

            //#region deduct the lock from the middle part

            //Brep barrelCylinderDeduct = Brep.CreatePipe(barrelTraj, barrelCylinderRadius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            //barrelCylinderDeduct.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == barrelCylinderDeduct.SolidOrientation)
            //    barrelCylinderDeduct.Flip();

            //midPartBackup = MidPart.Model.DuplicateBrep();

            //MidPart.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == MidPart.Model.SolidOrientation)
            //    MidPart.Model.Flip();
            //MidPart.Model = Brep.CreateBooleanDifference(MidPart.Model, barrelCylinderDeduct, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            //MidPart.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == MidPart.Model.SolidOrientation)
            //    MidPart.Model.Flip();

            //#endregion

            //Point3d barrelTunnelPt0 = barrelStart + tipDir * (beamOffset + clearance) + direction / direction.Length * (beamWidthOffset + clearance);
            //Point3d barrelTunnelPt1 = barrelStart + tipDir * (beamOffset + clearance) - direction / direction.Length * (beamWidthOffset + clearance);
            //Point3d barrelTunnelPt2 = barrelStart - tipDir * (beamOffset + clearance) - direction / direction.Length * (beamWidthOffset + clearance);
            //Point3d barrelTunnelPt3 = barrelStart - tipDir * (beamOffset + clearance) + direction / direction.Length * (beamWidthOffset + clearance);
            //Point3d barrelTunnelPt4 = barrelTunnelPt0;

            //List<Point3d> barrelTunnelCorners = new List<Point3d>();
            //barrelTunnelCorners.Add(barrelTunnelPt0);
            //barrelTunnelCorners.Add(barrelTunnelPt1);
            //barrelTunnelCorners.Add(barrelTunnelPt2);
            //barrelTunnelCorners.Add(barrelTunnelPt3);
            //barrelTunnelCorners.Add(barrelTunnelPt4);

            //Polyline barrelTunnelRect = new Polyline(barrelTunnelCorners);
            //Curve barrelTunnelRectCrv = barrelTunnelRect.ToNurbsCurve();

            //Brep[] barrelTunnelBreps = sweep.PerformSweep(barrelTraj, barrelTunnelRectCrv);
            //Brep barrelTunnelBrep = barrelTunnelBreps[0];
            //Brep barrelTunnel = barrelTunnelBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //barrelTunnel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == barrelTunnel.SolidOrientation)
            //    barrelTunnel.Flip();


            //Point3d barrelHollowPt0 = barrelHollowStart + tipDir * (beamOffset + clearance) + direction / direction.Length * 2 * (beamWidthOffset + clearance + barrelThickness);
            //Point3d barrelHollowPt1 = barrelHollowStart + tipDir * (beamOffset + clearance) - direction / direction.Length * 2 * (beamWidthOffset + clearance + barrelThickness);
            //Point3d barrelHollowPt2 = barrelHollowStart - tipDir * (beamOffset + clearance) - direction / direction.Length * 2 * (beamWidthOffset + clearance + barrelThickness);
            //Point3d barrelHollowPt3 = barrelHollowStart - tipDir * (beamOffset + clearance) + direction / direction.Length * 2 * (beamWidthOffset + clearance + barrelThickness);
            //Point3d barrelHollowPt4 = barrelHollowPt0;

            //List<Point3d> barrelHollowCorners = new List<Point3d>();
            //barrelHollowCorners.Add(barrelHollowPt0);
            //barrelHollowCorners.Add(barrelHollowPt1);
            //barrelHollowCorners.Add(barrelHollowPt2);
            //barrelHollowCorners.Add(barrelHollowPt3);
            //barrelHollowCorners.Add(barrelHollowPt4);

            //Polyline barrelHollowRect = new Polyline(barrelHollowCorners);
            //Curve barrelHollowRectCrv = barrelHollowRect.ToNurbsCurve();

            //Brep[] barrelHollowBreps = sweep.PerformSweep(barrelHollowTraj, barrelHollowRectCrv);
            //Brep barrelHollowBrep = barrelHollowBreps[0];
            //Brep barrelHollow = barrelHollowBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //barrelHollow.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == barrelHollow.SolidOrientation)
            //    barrelHollow.Flip();

            //List<Point3d> sphereShells = new List<Point3d>();
            //sphereShells.Add(notchCenFirst);
            //sphereShells.Add(notchCenSecond);
            //sphereShells.Add(notchCenFirst - tipDir * beamOffset * 2);
            //sphereShells.Add(notchCenSecond - tipDir * beamOffset * 2);

            //List<Brep> deductBreps = new List<Brep>();
            //deductBreps.Add(barrelTunnel);
            //deductBreps.Add(barrelHollow);

            //foreach (Point3d pt in sphereShells)
            //{
            //    Sphere notchSphere = new Sphere(pt, notchRadius);
            //    Brep notchShellBrep = notchSphere.ToBrep();

            //    notchShellBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //    if (BrepSolidOrientation.Inward == notchShellBrep.SolidOrientation)
            //        notchShellBrep.Flip();

            //    deductBreps.Add(notchShellBrep);
            //}

            //Brep barrelDeductBrep = Brep.CreateBooleanUnion(deductBreps, myDoc.ModelAbsoluteTolerance)[0];
            //barrelDeductBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == barrelDeductBrep.SolidOrientation)
            //    barrelDeductBrep.Flip();


            //Brep barrel = Brep.CreateBooleanDifference(barrelBrep, barrelDeductBrep, myDoc.ModelAbsoluteTolerance)[0];
            //barrel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == barrel.SolidOrientation)
            //    barrel.Flip();

            //#endregion

            //#region combine the beam, the axis, and the handler

            //Brep lockBasedBrep = Brep.CreateBooleanUnion(new List<Brep> { beam, handler, centralAxis }, myDoc.ModelAbsoluteTolerance)[0];
            //lockBasedBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == lockBasedBrep.SolidOrientation)
            //    lockBasedBrep.Flip();

            //MidPart.Model = Brep.CreateBooleanUnion(new List<Brep> { MidPart.Model, barrel }, myDoc.ModelAbsoluteTolerance)[0];
            //MidPart.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == MidPart.Model.SolidOrientation)
            //    MidPart.Model.Flip();

            //#endregion
            #endregion - old
            //myDoc.Objects.AddBrep(barrel);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(lockBasedBrep);
            //myDoc.Views.Redraw();

            #region add lock parts to the entitylist

            //Lock lockBase = new Lock(lockBasedBrep, false);
            Brep cutBarrel=null,addBarrel=null;
            Lock lockBase = new Lock(direction, lockClosestPointOnAxis, LockPosition, ratchetRadius,false, myDoc,ref cutBarrel,ref addBarrel,"lockbase");
            MidPart.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == MidPart.Model.SolidOrientation)
                MidPart.Model.Flip();
            MidPart.Model = Brep.CreateBooleanDifference(MidPart.Model, cutBarrel, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            MidPart.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == MidPart.Model.SolidOrientation)
                MidPart.Model.Flip();
            MidPart.Model = Brep.CreateBooleanUnion(new List<Brep> { MidPart.Model, addBarrel }, myDoc.ModelAbsoluteTolerance)[0];
            MidPart.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == MidPart.Model.SolidOrientation)
                MidPart.Model.Flip();
            if (midPartIdx != -1)
            {
                // midpart has been registered in the entity list
                entityList.RemoveAt(midPartIdx);
            }
            
            entityList.Add(MidPart); // the MidPart's model is updated with an extra barrel
            lockPartIdx.Add(entityList.Count - 1);
            entityList.Add(LockHead); // the ratchet gear
            lockPartIdx.Add(entityList.Count - 1);
            entityList.Add(lockBase); // the latch
            lockPartIdx.Add(entityList.Count - 1);
            locks.Add(LockHead);
            locks.Add(lockBase);
            LockHead.RegisterOtherPart(lockBase);
            _ = new Fixation(centerAxis, LockHead);

            #endregion

            #region old method

            //Point3d railS = LockPosition - centerLinkDirection * 10;
            //Point3d railE = LockPosition + centerLinkDirection * lockDisToAxis * 0.5;
            //Point3d railS1 = LockPosition - centerLinkDirection * 0.5;
            //Point3d railE1 = LockPosition + centerLinkDirection * 2.5;
            //Line l = new Line(railS, railE);
            //Brep LockBaseBrep = Brep.CreatePipe(l.ToNurbsCurve(), 1.5, false, PipeCapMode.Flat, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAngleToleranceRadians)[0];
            //Brep LockBaseBrepLarge = Brep.CreatePipe(l.ToNurbsCurve(), 2.5, false, PipeCapMode.Flat, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, RhinoDoc.ActiveDoc.ModelAngleToleranceRadians)[0];
            ////LockBaseBrepLarge.Flip();
            //MidPart.Model.Flip();
            //MidPart.Model = Brep.CreateBooleanDifference(MidPart.Model, LockBaseBrepLarge, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            //Plane pl1 = new Plane(LockPosition + centerLinkDirection * (lockDisToAxis * 0.21 + 2), centerLinkDirection);
            //Cylinder c1 = new Cylinder(new Circle(pl1, 3.5));
            //c1.Height1 = 0; c1.Height2 = 3;
            //Plane pl2 = new Plane(LockPosition, centerLinkDirection);
            //Cylinder c2 = new Cylinder(new Circle(pl2, 3.5));
            //c2.Height1 = -3; c2.Height2 = 0;
            //LockBaseBrep.Append(c1.ToBrep(true, true));
            //LockBaseBrep.Append(c2.ToBrep(true, true));
            //LockBaseBrep.Append(new Sphere(railS, 2.5).ToBrep());
            //LockBaseBrep.Transform(Transform.Translation(-centerLinkDirection * lockDisToAxis * 0.1));
            //locks.Add(LockHead);
            //locks.Add(new Lock(LockBaseBrep, false));
            //entityList.Add(LockHead);
            //entityList.Add(locks[1]);
            //LockHead.RegisterOtherPart(locks[1]);
            //_ = new Fixation(centerAxis, LockHead);

            #endregion
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
            double degree = maxDegree / 10.0 * 2 * Math.PI;
            Movement twist = new Movement(centerAxis, 2, degree, Transform.Rotation(degree, direction, spiralSpring.CenterPoint));
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
