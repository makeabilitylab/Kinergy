using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.DocObjects;
using Rhino.Collections;
using Rhino.Input.Custom;
using Rhino;
using Kinergy.Relationship;
using KinergyUtilities;

namespace Kinergy.Geom
{
    public class Lock : Entity
    {
        private bool headOrBase;
        private Lock otherPart = null;
        private bool locked = false;
        private Point3d releasePosition = Point3d.Unset;
        private bool isRachetLockHead = false;
        private Brep _handler, _beam, _axis;
        Vector3d _lockMoveVector;
        /// <summary>
        /// Lock entity.This constructor is for lock head.
        /// </summary>
        /// <param name="brep"></param>
        /// <param name="LockHeadOrLockBase">True for head and false for base</param>
        /// <param name="stat">Whether the entity is static</param>
        /// <param name="n"></param>
        public Lock(Brep brep, bool LockHeadOrLockBase, bool stat = false, string n = "") : base(brep, false, n)
        {
            headOrBase = LockHeadOrLockBase;
        }
        public Lock(Brep brep, bool LockHeadOrLockBase, Point3d Release, bool stat = false, string n = "") : base(brep, false, n)
        {

            headOrBase = LockHeadOrLockBase;
            releasePosition = Release;
        }
        public Lock(Point3d CenterPoint, Vector3d direction, double radius, bool reverse = false, bool stat = false, string n = "", double thickness = 4) : base(null, false, n)
        {
            model = BuildRatchetLockPlate(CenterPoint, direction, radius, reverse, thickness);
            headOrBase = false;
        }
        public Lock(Vector3d direction, Point3d lockClosestPointOnAxis, Point3d LockPosition, double ratchetRadius ,bool isLocked,RhinoDoc myDoc,ref Brep cutBarrel,ref Brep addBarrel,string name="" )
        {
            isRachetLockHead = true;
            headOrBase = true;
            model = BuildRachetLockHead(direction, lockClosestPointOnAxis,LockPosition,ratchetRadius,isLocked, myDoc,ref cutBarrel,ref addBarrel);
            Locked = isLocked;

        }
        public bool HeadOrBase { get => headOrBase;private set => headOrBase = value; }
        public Lock OtherPart { get => otherPart;private set => otherPart = value; }
        public bool Locked { get => locked;private set => locked = value; }
        public Point3d ReleasePosition { get => releasePosition;private set => releasePosition = value; }
        public Brep BuildRachetLockHead(Vector3d direction, Point3d lockClosestPointOnAxis, Point3d LockPosition, double ratchetRadius, bool isLocked, RhinoDoc myDoc, ref Brep cutBarrel,ref Brep addBarrel)
        {
            Vector3d centerLinkDirection = new Vector3d(lockClosestPointOnAxis) - new Vector3d(LockPosition);
            double centerLinkLen = centerLinkDirection.Length;
            centerLinkDirection.Unitize();

            #region create the handler

            double handlerDis = Math.Max(20, ratchetRadius);
            double handlerThickness = 3;
            double handlerRadius = 5;
            Point3d handlerPos = LockPosition - centerLinkDirection * handlerDis;
            Point3d handlerEndPos = handlerPos - centerLinkDirection * handlerThickness;
            Curve handlerTraj = new Line(handlerPos, handlerEndPos).ToNurbsCurve();
            Brep handler = Brep.CreatePipe(handlerTraj, handlerRadius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            handler.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == handler.SolidOrientation)
                handler.Flip();

            #endregion

            #region create the central axis and the piston

            double axisLen = 0;
            double latchClearance = 4;
            double tipLen = 2;
            if (ratchetRadius < (tipLen + latchClearance)) return null;
            axisLen = ratchetRadius - (tipLen + latchClearance) + handlerDis;

            double axisRadius = 1;
            Point3d axisEnd = handlerPos;
            Point3d axisStart = axisEnd + centerLinkDirection * axisLen;
            Curve axisTraj = new Line(axisStart, axisEnd).ToNurbsCurve();
            Brep axisRodBrep = Brep.CreatePipe(axisTraj, axisRadius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            axisRodBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == axisRodBrep.SolidOrientation)
                axisRodBrep.Flip();

            Transform rot = Transform.Rotation(Math.PI / 2, centerLinkDirection, axisStart);
            Vector3d tipDir = direction;
            tipDir.Transform(rot);
            tipDir.Unitize();
            double tipBig = 2.5;
            double tipSmall = 1;

            var sweep = new SweepOneRail();
            sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            sweep.ClosedSweep = false;
            sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            Point3d tipPt0 = axisStart + tipDir * axisRadius + direction / direction.Length * tipBig;
            Point3d tipPt1 = axisStart + tipDir * axisRadius + centerLinkDirection * tipLen + direction / direction.Length * tipSmall;
            Point3d tipPt2 = axisStart + tipDir * axisRadius + centerLinkDirection * tipLen - direction / direction.Length * tipSmall;
            Point3d tipPt3 = axisStart + tipDir * axisRadius - direction / direction.Length * tipBig;
            Point3d tipPt4 = tipPt0;

            List<Point3d> tipCorners = new List<Point3d>();
            tipCorners.Add(tipPt0);
            tipCorners.Add(tipPt1);
            tipCorners.Add(tipPt2);
            tipCorners.Add(tipPt3);
            tipCorners.Add(tipPt4);

            Polyline tipRect = new Polyline(tipCorners);
            Curve tipRectCrv = tipRect.ToNurbsCurve();

            Point3d tipPathStart = axisStart + tipDir * axisRadius;
            Point3d tipPathEnd = axisStart - tipDir * axisRadius;
            Curve tipTraj = new Line(tipPathStart, tipPathEnd).ToNurbsCurve();

            Brep[] tipBreps = sweep.PerformSweep(tipTraj, tipRectCrv);
            Brep tipBrep = tipBreps[0];
            Brep tip = tipBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            tip.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == tip.SolidOrientation)
                tip.Flip();

            Brep centralAxis = Brep.CreateBooleanUnion(new List<Brep> { axisRodBrep, tip }, myDoc.ModelAbsoluteTolerance)[0];

            centralAxis.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == centralAxis.SolidOrientation)
                centralAxis.Flip();

            myDoc.Objects.AddBrep(centralAxis);
            myDoc.Views.Redraw();

            #endregion

            #region create the beams

            double clearance = 0.6;

            Point3d axisMidPoint = (axisStart + axisEnd) / 2.0;
            double beamDis = axisLen / 4.0;
            double beamOffset = 3;
            double beamWidthOffset = 1;
            double beamThickness = 2;
            double beamLeverThickness = 1;
            Point3d beamPos = axisMidPoint - centerLinkDirection * beamDis;

            Point3d beamHolderPt0 = beamPos + tipDir * beamOffset + direction / direction.Length * beamWidthOffset;
            Point3d beamHolderPt1 = beamPos + tipDir * beamOffset - direction / direction.Length * beamWidthOffset;
            Point3d beamHolderPt2 = beamPos + tipDir * beamOffset - direction / direction.Length * beamWidthOffset - centerLinkDirection * beamThickness;
            Point3d beamHolderPt3 = beamPos + tipDir * beamOffset + direction / direction.Length * beamWidthOffset - centerLinkDirection * beamThickness;
            Point3d beamHolderPt4 = beamHolderPt0;

            List<Point3d> beamCorners = new List<Point3d>();
            beamCorners.Add(beamHolderPt0);
            beamCorners.Add(beamHolderPt1);
            beamCorners.Add(beamHolderPt2);
            beamCorners.Add(beamHolderPt3);
            beamCorners.Add(beamHolderPt4);

            Polyline beamRect = new Polyline(beamCorners);
            Curve beamRectCrv = beamRect.ToNurbsCurve();

            Point3d beamPathStart = beamPos + tipDir * beamOffset;
            Point3d beamPathEnd = beamPos - tipDir * beamOffset;
            Curve beamTraj = new Line(beamPathStart, beamPathEnd).ToNurbsCurve();

            Brep[] beamHolderBreps = sweep.PerformSweep(beamTraj, beamRectCrv);
            Brep beamHolderBrep = beamHolderBreps[0];
            Brep beamHolder = beamHolderBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            beamHolder.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == beamHolder.SolidOrientation)
                beamHolder.Flip();

            double hookRadius = 1;
            double notchRadius = 1.6;
            double moveDis = tipLen + latchClearance + 2 * 0.6 - 2 * hookRadius;

            Point3d notchCenFirst = axisMidPoint + tipDir * beamOffset + centerLinkDirection * hookRadius;
            Point3d notchCenSecond = notchCenFirst + centerLinkDirection * (2 * hookRadius + moveDis);
            double beamLeverLen = axisLen / 4.0 + 2 * hookRadius;

            Point3d beamLeverStart = beamHolderPt0;
            Point3d beamLeverEnd = beamHolderPt0 + centerLinkDirection * beamLeverLen;
            Curve beamLeverTraj = new Line(beamLeverStart, beamLeverEnd).ToNurbsCurve();

            Point3d beamDetentStart = beamHolderPt0 + centerLinkDirection * (axisLen / 4.0);
            Point3d beamDetentEnd = beamDetentStart + centerLinkDirection * 2 * hookRadius;
            Curve beamDetentTraj = new Line(beamDetentStart, beamDetentEnd).ToNurbsCurve();

            Point3d beamLeverRPt0 = beamPos + tipDir * beamOffset + direction / direction.Length * beamWidthOffset;
            Point3d beamLeverRPt1 = beamPos + tipDir * beamOffset - direction / direction.Length * beamWidthOffset;
            Point3d beamLeverRPt2 = beamPos + tipDir * (beamOffset - beamLeverThickness) - direction / direction.Length * beamWidthOffset;
            Point3d beamLeverRPt3 = beamPos + tipDir * (beamOffset - beamLeverThickness) + direction / direction.Length * beamWidthOffset;
            Point3d beamLeverRPt4 = beamLeverRPt0;

            Point3d beamDetentRPt0 = beamLeverRPt0 + centerLinkDirection * (axisLen / 4.0);
            Point3d beamDetentRPt1 = beamLeverRPt1 + centerLinkDirection * (axisLen / 4.0);
            Point3d beamDetentRPt2 = beamPos + tipDir * (beamOffset - hookRadius) - direction / direction.Length * beamWidthOffset + centerLinkDirection * (axisLen / 4.0);
            Point3d beamDetentRPt3 = beamPos + tipDir * (beamOffset - hookRadius) + direction / direction.Length * beamWidthOffset + centerLinkDirection * (axisLen / 4.0);
            Point3d beamDetentRPt4 = beamDetentRPt0;

            List<Point3d> beamLeverRCorners = new List<Point3d>();
            beamLeverRCorners.Add(beamLeverRPt0);
            beamLeverRCorners.Add(beamLeverRPt1);
            beamLeverRCorners.Add(beamLeverRPt2);
            beamLeverRCorners.Add(beamLeverRPt3);
            beamLeverRCorners.Add(beamLeverRPt4);

            List<Point3d> beamDetentRCorners = new List<Point3d>();
            beamDetentRCorners.Add(beamDetentRPt0);
            beamDetentRCorners.Add(beamDetentRPt1);
            beamDetentRCorners.Add(beamDetentRPt2);
            beamDetentRCorners.Add(beamDetentRPt3);
            beamDetentRCorners.Add(beamDetentRPt4);

            Polyline beamLeverRRect = new Polyline(beamLeverRCorners);
            Curve beamLeverRRectCrv = beamLeverRRect.ToNurbsCurve();

            Polyline beamDetentRRect = new Polyline(beamDetentRCorners);
            Curve beamDetentRRectCrv = beamDetentRRect.ToNurbsCurve();

            Brep[] beamLeverRBreps = sweep.PerformSweep(beamLeverTraj, beamLeverRRectCrv);
            Brep beamLeverRBrep = beamLeverRBreps[0];
            Brep beamLeverR = beamLeverRBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            beamLeverR.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == beamLeverR.SolidOrientation)
                beamLeverR.Flip();

            Brep[] beamDetentRBreps = sweep.PerformSweep(beamDetentTraj, beamDetentRRectCrv);
            Brep beamDetentRBrep = beamDetentRBreps[0];
            Brep beamDetentR = beamDetentRBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            beamDetentR.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == beamDetentR.SolidOrientation)
                beamDetentR.Flip();

            Sphere beamDetentRSphere = new Sphere(notchCenFirst, hookRadius);
            Brep beamDetentRSphereBrep = beamDetentRSphere.ToBrep();
            beamDetentRSphereBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == beamDetentRSphereBrep.SolidOrientation)
                beamDetentRSphereBrep.Flip();

     

            Brep beamDetentRNotch = Brep.CreateBooleanDifference(beamDetentRSphereBrep, beamDetentR, myDoc.ModelAbsoluteTolerance)[0];
            beamDetentRNotch.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == beamDetentRNotch.SolidOrientation)
                beamDetentRNotch.Flip();

            Brep beamRight = Brep.CreateBooleanUnion(new List<Brep> { beamLeverR, beamDetentRNotch }, myDoc.ModelAbsoluteTolerance)[0];
            beamRight.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == beamRight.SolidOrientation)
                beamRight.Flip();

            Brep beamLeft = beamRight.DuplicateBrep();
            Transform mirrorTrans = Transform.Mirror(LockPosition, tipDir);
            beamLeft.Transform(mirrorTrans);
            beamLeft.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == beamLeft.SolidOrientation)
                beamLeft.Flip();

            Brep beam = Brep.CreateBooleanUnion(new List<Brep> { beamHolder, beamRight, beamLeft }, myDoc.ModelAbsoluteTolerance)[0];

            #endregion

            #region create the barrel

            Point3d barrelStart = (axisMidPoint + beamPos) / 2.0;
            Point3d barrelEnd = barrelStart + centerLinkDirection * (barrelStart.DistanceTo(axisMidPoint) + 4 * hookRadius + moveDis + 2);
            Curve barrelTraj = new Line(barrelStart, barrelEnd).ToNurbsCurve();

            Point3d barrelHollowStart = barrelStart + centerLinkDirection * barrelStart.DistanceTo(axisMidPoint);
            Point3d barrelHollowEnd = barrelHollowStart + centerLinkDirection * (4 * hookRadius + moveDis);
            Curve barrelHollowTraj = new Line(barrelHollowStart, barrelHollowEnd).ToNurbsCurve();

            double barrelThickness = 1.2;
            double barrelRadius = beamOffset + clearance + barrelThickness;

            double barrelCylinderRadius = barrelRadius - 0.2;

            Brep barrelBrep = Brep.CreatePipe(barrelTraj, barrelRadius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            barrelBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == barrelBrep.SolidOrientation)
                barrelBrep.Flip();

            #region deduct the lock from the middle part

            Brep barrelCylinderDeduct = Brep.CreatePipe(barrelTraj, barrelCylinderRadius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            barrelCylinderDeduct.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == barrelCylinderDeduct.SolidOrientation)
                barrelCylinderDeduct.Flip();

            cutBarrel = barrelCylinderDeduct;
            #endregion

            Point3d barrelTunnelPt0 = barrelStart + tipDir * (beamOffset + clearance) + direction / direction.Length * (beamWidthOffset + clearance);
            Point3d barrelTunnelPt1 = barrelStart + tipDir * (beamOffset + clearance) - direction / direction.Length * (beamWidthOffset + clearance);
            Point3d barrelTunnelPt2 = barrelStart - tipDir * (beamOffset + clearance) - direction / direction.Length * (beamWidthOffset + clearance);
            Point3d barrelTunnelPt3 = barrelStart - tipDir * (beamOffset + clearance) + direction / direction.Length * (beamWidthOffset + clearance);
            Point3d barrelTunnelPt4 = barrelTunnelPt0;

            List<Point3d> barrelTunnelCorners = new List<Point3d>();
            barrelTunnelCorners.Add(barrelTunnelPt0);
            barrelTunnelCorners.Add(barrelTunnelPt1);
            barrelTunnelCorners.Add(barrelTunnelPt2);
            barrelTunnelCorners.Add(barrelTunnelPt3);
            barrelTunnelCorners.Add(barrelTunnelPt4);

            Polyline barrelTunnelRect = new Polyline(barrelTunnelCorners);
            Curve barrelTunnelRectCrv = barrelTunnelRect.ToNurbsCurve();

            Brep[] barrelTunnelBreps = sweep.PerformSweep(barrelTraj, barrelTunnelRectCrv);
            Brep barrelTunnelBrep = barrelTunnelBreps[0];
            Brep barrelTunnel = barrelTunnelBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            barrelTunnel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == barrelTunnel.SolidOrientation)
                barrelTunnel.Flip();

            Point3d barrelHollowPt0 = barrelHollowStart + tipDir * (beamOffset + clearance) + direction / direction.Length * 2 * (beamWidthOffset + clearance + barrelThickness);
            Point3d barrelHollowPt1 = barrelHollowStart + tipDir * (beamOffset + clearance) - direction / direction.Length * 2 * (beamWidthOffset + clearance + barrelThickness);
            Point3d barrelHollowPt2 = barrelHollowStart - tipDir * (beamOffset + clearance) - direction / direction.Length * 2 * (beamWidthOffset + clearance + barrelThickness);
            Point3d barrelHollowPt3 = barrelHollowStart - tipDir * (beamOffset + clearance) + direction / direction.Length * 2 * (beamWidthOffset + clearance + barrelThickness);
            Point3d barrelHollowPt4 = barrelHollowPt0;

            List<Point3d> barrelHollowCorners = new List<Point3d>();
            barrelHollowCorners.Add(barrelHollowPt0);
            barrelHollowCorners.Add(barrelHollowPt1);
            barrelHollowCorners.Add(barrelHollowPt2);
            barrelHollowCorners.Add(barrelHollowPt3);
            barrelHollowCorners.Add(barrelHollowPt4);

            Polyline barrelHollowRect = new Polyline(barrelHollowCorners);
            Curve barrelHollowRectCrv = barrelHollowRect.ToNurbsCurve();

            Brep[] barrelHollowBreps = sweep.PerformSweep(barrelHollowTraj, barrelHollowRectCrv);
            Brep barrelHollowBrep = barrelHollowBreps[0];
            Brep barrelHollow = barrelHollowBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            barrelHollow.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == barrelHollow.SolidOrientation)
                barrelHollow.Flip();

            List<Point3d> sphereShells = new List<Point3d>();
            sphereShells.Add(notchCenFirst);
            sphereShells.Add(notchCenSecond);
            sphereShells.Add(notchCenFirst - tipDir * beamOffset * 2);
            sphereShells.Add(notchCenSecond - tipDir * beamOffset * 2);

            List<Brep> deductBreps = new List<Brep>();
            deductBreps.Add(barrelTunnel);
            deductBreps.Add(barrelHollow);

            foreach (Point3d pt in sphereShells)
            {
                Sphere notchSphere = new Sphere(pt, notchRadius);
                Brep notchShellBrep = notchSphere.ToBrep();

                notchShellBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == notchShellBrep.SolidOrientation)
                    notchShellBrep.Flip();

                deductBreps.Add(notchShellBrep);
            }

            Brep barrelDeductBrep = Brep.CreateBooleanUnion(deductBreps, myDoc.ModelAbsoluteTolerance)[0];
            barrelDeductBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == barrelDeductBrep.SolidOrientation)
                barrelDeductBrep.Flip();


            Brep barrel = Brep.CreateBooleanDifference(barrelBrep, barrelDeductBrep, myDoc.ModelAbsoluteTolerance)[0];
            barrel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == barrel.SolidOrientation)
                barrel.Flip();
            addBarrel = barrel;

            #endregion

            #region combine the beam, the axis, and the handler
            _beam = beam;
            _axis = centralAxis;
            _handler = handler;
            _lockMoveVector = centerLinkDirection * (2 * hookRadius + moveDis);
            
            #endregion
            return JoinRachetLockHeadBrep(isLocked,myDoc);
        }
        private Brep JoinRachetLockHeadBrep(bool isLocked,RhinoDoc myDoc)
        {
           
            Brep lockBasedBrep = Brep.CreateBooleanUnion(new List<Brep> { _beam, _handler, _axis }, myDoc.ModelAbsoluteTolerance)[0];
            lockBasedBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == lockBasedBrep.SolidOrientation)
                lockBasedBrep.Flip();

            if (isLocked)
                lockBasedBrep.Transform(Transform.Translation(_lockMoveVector));
            return lockBasedBrep;
        }
        public static Brep BuildRatchetLockPlate(Point3d CenterPoint, Vector3d direction, double radius,bool reverse=false, double thickness=4)
        {
            List<Point3d> outerPts = new List<Point3d>();
            List<Point3d> innerPts = new List<Point3d>();
            Plane p = new Plane(CenterPoint, direction);
            PolyCurve s = new PolyCurve();
            for (int i=0;i<12;i++)
            {
                double angle = Math.PI / 6 * i;
                
                Vector3d x = p.XAxis, y = p.YAxis;
                outerPts.Add(CenterPoint + x * Math.Sin(angle) * radius * 1.1 + y * Math.Cos(angle) * radius * 1.1);
                innerPts.Add(CenterPoint + x * Math.Sin(angle) * radius * 0.75 + y * Math.Cos(angle) * radius * 0.75);
            }
            for(int i=0;i<12;i++)
            {
                if(reverse)
                {
                    
                    s.Append(new Line(innerPts[i],outerPts[i]).ToNurbsCurve());
                    s.Append(new Line(outerPts[i],innerPts[(i + 1) % 12]).ToNurbsCurve());
                }
                else
                {
                    s.Append(new Line(outerPts[i], innerPts[i]).ToNurbsCurve());
                    s.Append(new Line(innerPts[i], outerPts[(i + 1) % 12]).ToNurbsCurve());
                }
                
            }
            Point3d start = CenterPoint - direction * thickness/2;
            Point3d end = CenterPoint + direction * thickness/2;
            Curve c = s.ToNurbsCurve();
            Curve c1 = s.ToNurbsCurve();
            c.Transform(Transform.Translation((new Vector3d(start) - new Vector3d(end)) / 2));
            c1.Transform(Transform.Translation((new Vector3d(end) - new Vector3d(start))/2));
            Curve rail = new Line(start, end).ToNurbsCurve();
            Brep b1 = Brep.CreateFromSweep(rail, c, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            Brep b2 = Brep.CreatePatch(new List<Curve> { c }, null, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep b3 = Brep.CreatePatch(new List<Curve> { c1 }, null, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            //Brep m = Brep.CreateSolid(new List<Brep> { b1, b2, b3 }, 0.001)[0];
            Brep m = new Brep();
            m.Append(b1);
            m.Append(b2);
            m.Append(b3);
            return m;
        }
        
        protected override void ConductMoveAndUpdateParam(Movement move)
        {
            /*base.model.Transform(move.Trans);
            UpdateBasicParams();*/
            base.ConductMoveAndUpdateParam(move);
        }
        public bool RegisterOtherPart(Lock other)
        {
            if(other.OtherPart==null || other.OtherPart==this)
            { 
                if(this.headOrBase==other.HeadOrBase)
                {
                    return false;
                }
                otherPart = other;
                if(other.OtherPart == null)
                {other.RegisterOtherPart(this); }
                return true;
            }
            return false;
        }
        public override bool AddConstraint(Relationship.Relationship constraint)
        {
            if(constraint.GetType()==typeof(Locking))
            {
                //Check type
                if(constraint.TheOtherEntity(this).GetType()!=typeof(Lock))
                {
                    return false;
                }
                Lock other = (Lock)constraint.TheOtherEntity(this);
                //check head or base
                if (this.headOrBase==other.HeadOrBase)
                {
                    return false;
                }
                if (constraint.IsValid() && constraint.Include(this))
                {
                    constraints.Add(constraint);
                    return true;
                }
                else { return false; }
            }
            else 
            {
                return base.AddConstraint(constraint);
            }
        }
        public bool SetLocked()
        {
            if(otherPart==null)
            {
                return false;
            }
            if(locked==true)
            {
                return false;
            }
            if (locked == false)
            {
                locked = true;
                otherPart.SetLocked();
                if (isRachetLockHead)
                    model = JoinRachetLockHeadBrep(true, RhinoDoc.ActiveDoc);
            }
            if(headOrBase)
                _ = new Locking(this, otherPart);
            //AddConstraint(l);
            return true;
        }
        public bool Activate()
        {
            //First check if this is lock base. only base could generate pointing position
            if(otherPart==null)
            {
                return false;
            }
            if(headOrBase==true)
            {
                if (isRachetLockHead)
                    model = JoinRachetLockHeadBrep(false, RhinoDoc.ActiveDoc);
                return otherPart.Activate();
            }
            /*if(releasePosition==Point3d.Unset)
            {
                return false;
            }*/
            //generate Point at releasePosition and let user select it
            if(UserSelection.UserSelectPointInRhino(new List<Point3d> { releasePosition }, RhinoDoc.ActiveDoc)==0)
            {
                //Release();
                Unlock();
            }
            return true;
        }
        public bool ActivateWithoutInteraction()
        {
            //First check if this is lock base. only base could generate pointing position
            if (otherPart == null)
            {
                return false;
            }
            /*if (headOrBase == true)
            {
                return otherPart.ActivateWithoutInteraction();
            }*/
            /*
            if (releasePosition == Point3d.Unset)
            {
                return false;
            }*/
            //generate Point at releasePosition and let user select it
            //Release();
            if (headOrBase == false)
                otherPart.ActivateWithoutInteraction();
            else
                Unlock();
            //Unlock();
            return true;
        }
        /// <summary>
        /// This method deletes the locking relationship between locks. Don't use it unless necessary
        /// </summary>
        private void Release()
        {
            int count = constraints.Count();
            string body = string.Format("Release method called");
            Rhino.RhinoApp.WriteLine(body);
            for (int i= 0; i < count; i++)
            {
                if(constraints[i].GetType()==typeof(Locking))
                {
                    Locking l = (Locking)constraints[i];
                    l.Release();
                    i -= 1;
                    count -= 1;
                }
            }
        }
        /// <summary>
        /// This method make sure that both part of the lock is not set locked
        /// </summary>
        private void Unlock()
        {
            locked = false;
            otherPart.Locked = false;
            if (isRachetLockHead)
                model = JoinRachetLockHeadBrep(false, RhinoDoc.ActiveDoc);
            Rhino.RhinoApp.WriteLine("Unlock executed!");
        }
        public override bool Move(Movement move)
        {
            if (this.Equals(move.Obj) == false)
            {
                throw new Exception("Wrong movement parameter given to entity!");
            }
            if (staticEntity)
            {
                return false;
            }
            DfsMark = true;
            bool CanIMove = true;
            //Then move all other constraints to know if this movement can be operated
            foreach (Relationship.Relationship c in constraints)
            {
                
                if (c.TheOtherEntity(this).DfsMark == true)//Skip the already visited component to avoid cycle triggering.
                { continue; }
                if (c.Move(move) == false)
                {
                    CanIMove = false;
                    string body = string.Format("A movement on {0} typed {1} with value {2} is stopped by {3} to {4}", this.GetType(), move.Type, move.MovementValue, c.GetType(), c.TheOtherEntity(this).GetType());
                    Rhino.RhinoApp.WriteLine(body);
                    break;
                }
            }
            if (CanIMove)
            {
                this.ConductMoveAndUpdateParam(move);
            }
            DfsMark = false;
            return CanIMove;
        }
    }
}
