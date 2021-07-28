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

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace Kinergy.KineticUnit
{
    public class InstantTranslation : KineticUnit
    {
        //The initial inputs
        private Brep model;
        private bool curved = false;
        private double energy;//energy is above 0 and less than 10,indicating the total energy stored in fully compressed spring
        private double distance;//distance is between 0.2 and 0.8,indicating the ratio of spring travel out of length;
        private Vector3d direction = Vector3d.Unset;
        private bool addLock;
        private RhinoDoc myDoc;

        //The later selected parameters
        private Point3d springPosition = Point3d.Unset;
        private double springT, springStart, springEnd;
        private int lockDirection = 0;//0:unset;1:lock at left;2:lock at right;
        private Point3d lockPosition = Point3d.Unset;
        private double lockT;

        //The calculated geometry and parameters
        Vector3d skeletonVector;//start to end;
        //private Transform xrotate;
        //private Transform xrotateBack;
        private double springRadius = 0;
        private double wireRadius = 0;
        private double springLength = 0;
        private int roundNum = 0;
        private Interval skeletonAvailableRange;

        private Curve skeleton = null;
        private List<Shape> modelCut;
        private List<Lock> locks;
        private Helix spring;
        private List<Point3d> LockPositionCandidates=new List<Point3d>();
        private List<double> LockTCandidates = new List<double>();
        public Helix Spring { get => spring; private set => spring = value; }
        public bool Curved { get => curved; set => curved = value; }
        public Curve Skeleton { get => skeleton; protected set => skeleton = value; }
        public Brep Model { get => model; set => model = value; }
        public List<Shape> ModelCut { get => modelCut; set => modelCut = value; }

        List<int> lockShapeIndexes = new List<int>(); 

        /// <summary> Default constructor without basic input parameter </summary>
        /// <returns> Returns empty instance</returns>

        public InstantTranslation(Brep Model, bool Curved ,Vector3d Direction, double Energy, double Distance)
        {
            model = Model;
            energy = Energy;
            distance = Distance;
            direction = Direction;
            curved = Curved;
            //xrotate = Transform.Rotation(direction, Vector3d.XAxis, Point3d.Origin);//Deprecated. Too ambiguous and indirect.
            //xrotateBack = Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin);
            modelCut = new List<Shape>();
            myDoc = RhinoDoc.ActiveDoc;
            locks = new List<Lock>();
        }
       
        public bool SetSpringPosition(Point3d pos)
        {
            springPosition = pos;
            double T = 0;
            skeleton.ClosestPoint(springPosition, out T,1);
            springT = skeleton.Domain.NormalizedParameterAt(T);
            // springLength= GetSectionRadius(springT) / 7.5 * 25;
            springStart = (springT - springLength / 2 / skeleton.GetLength()) < 0? 0 : springT - springLength / 2 / skeleton.GetLength();
            springEnd = (springT + springLength / 2 / skeleton.GetLength()) > 1 ? 1: springT + springLength / 2 / skeleton.GetLength();

            return true;
        }

        public bool SetLockDirection(Arrow d)
        {
            Point3d p1 = skeleton.PointAtNormalizedLength(0);
            Point3d p2 = skeleton.PointAtNormalizedLength(1);
            direction  = new Vector3d(p2) - new Vector3d(p1);
            if (direction * d.Direction > 0.5)
            {
                lockDirection = 1;
            }
            else
            {
                lockDirection = 2;
            }
            return true;
        }
        public List<Point3d> GetLockPositionCandidates(Guid endEffectorID)
        {
            List<Point3d> candidates = new List<Point3d>();

            Point3d springS = skeleton.PointAtNormalizedLength(0);
            Point3d springE = skeleton.PointAtNormalizedLength(1);
            Point3d p_ee = new Point3d();
            Point3d p_stationary = new Point3d();

            Brep ee_brep = (Brep)myDoc.Objects.Find(endEffectorID).Geometry;
            Brep stationary_brep = new Brep();
            if(ee_brep.ClosestPoint(springS).DistanceTo(springS) <= ee_brep.ClosestPoint(springE).DistanceTo(springE)){
                // the end-effector is near the start point of the spring
                p_ee = springS;
                p_stationary = springE;
                p_stationary = p_stationary + (p_stationary - p_ee) / (p_stationary - p_ee).Length * 5;

                stationary_brep = modelCut[1].GetModelinWorldCoordinate();
            }
            else
            {
                // the end-effector is near the end point of the spring
                p_ee = springE;
                p_stationary = springS;
                p_stationary = p_stationary + (p_stationary - p_ee) / (p_stationary - p_ee).Length * 5;

                stationary_brep = modelCut[0].GetModelinWorldCoordinate();
            }

            Point3d p = p_stationary;
            Vector3d v = p_stationary - p_ee;
            Plane plane = new Plane(p, v);
            Curve[] c;
            Point3d[] pt;

            Rhino.Geometry.Intersect.Intersection.BrepPlane(stationary_brep, plane, myDoc.ModelAbsoluteTolerance, out c, out pt);
            if(c != null)
            {
                for (int j = 0; j < 12; j++)
                {
                    // take 12 samples (every 30 degree)
                    double interval = 1.0 / 12.0;
                    candidates.Add(c[0].PointAtNormalizedLength(j * interval));
                    LockPositionCandidates.Add(c[0].PointAtNormalizedLength(j * interval));
                    //LockTCandidates.Add(start + span * i / count);
                }
            }
            
            if (candidates.Count == 0)
            {
                RhinoApp.WriteLine("fail to generate the candidate positions for the lock!");
            }
            return candidates;

        }
        //public void SetLockPosition(Point3d p)
        //{
        //    lockPosition = p;
        //    for (int i = 0; i < LockPositionCandidates.Count(); i++)
        //    {
        //        Point3d pt = LockPositionCandidates[i];
        //        if (pt.X == p.X && pt.Y == p.Y && pt.Z == p.Z)
        //        {
        //            lockT = LockTCandidates[i];
        //            break;
        //        }
        //    }
        //    //double t;
        //    //skeleton.ClosestPoint(p, out t);
        //    //lockT= skeleton.Domain.NormalizedParameterAt(t);
        //}

        /// <summary>
        /// Compute the helical spring skeleton
        /// </summary>
        /// <param name="ptS">start point of the spring</param>
        /// <param name="ptE">end point of the spring</param>
        /// <param name="selectedModel">body to be converted</param>
        /// <returns></returns>
        public bool CalculateStraightSkeleton(Point3d ptS, Point3d ptE, Brep selectedModel)
        {
            Line l = new Line(ptS, ptE);
            skeleton = l.ToNurbsCurve();

            springLength = ptS.DistanceTo(ptE);

            //springRadius = Math.Min(box_sel.Max.Y - box_sel.Min.Y, box_sel.Max.Z - box_sel.Min.Z) * 0.9;
            //springRadius = Math.Min(model.ClosestPoint(stPt).DistanceTo(stPt), model.ClosestPoint(endPt).DistanceTo(endPt)) * 2;
            Vector3d planeNormal= direction;
            Plane firstPtPlane = new Plane(ptS, planeNormal);
            Plane secondPtPlane = new Plane(ptE, planeNormal);

            Curve[] intersectStart;
            Point3d[] intersectStartPts;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(model, firstPtPlane, myDoc.ModelAbsoluteTolerance, out intersectStart, out intersectStartPts);
            Curve strCrv = intersectStart[0];

            #region test by LH
            //myDoc.Objects.AddCurve(strCrv);
            //myDoc.Views.Redraw();
            #endregion

            Curve[] intersectEnd;
            Point3d[] intersectEndPts;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(model, secondPtPlane, myDoc.ModelAbsoluteTolerance, out intersectEnd, out intersectEndPts);
            Curve endCrv = intersectEnd[0];

            #region test by LH
            //myDoc.Objects.AddCurve(endCrv);
            //myDoc.Views.Redraw();
            #endregion

            double pos1, pos2;
            strCrv.ClosestPoint(ptS, out pos1);
            endCrv.ClosestPoint(ptE, out pos2);

            springRadius = Math.Min(strCrv.PointAt(pos1).DistanceTo(ptS), endCrv.PointAt(pos2).DistanceTo(ptE)) * 1.5;
            //wireRadius = springRadius / 7.5 * 1;
            wireRadius = 3.0; // by default and it should be constant

            skeletonVector = new Vector3d(skeleton.PointAtEnd) - new Vector3d(skeleton.PointAtStart);
            if (skeleton.GetLength() > 0)
            {
                return true;
            }
            return false;
        }

        public bool CalculateCurvedSkeleton()
        {
            if (curved)
            {
                SkeletonGen();
                
                double springLength1 = GetSectionRadius(0.1) / 7.5 * 25;
                double springLength2 = GetSectionRadius(0.9) / 7.5 * 25;
                if (springLength1 > skeleton.GetLength() * 0.4)//the model is too short,so decrease spring radius
                {
                    springLength1 = skeleton.GetLength() * 0.4;
                }
                if (springLength2 > skeleton.GetLength() * 0.4)//the model is too short,so decrease spring radius
                {
                    springLength2 = skeleton.GetLength() * 0.4;
                }
                skeletonAvailableRange = new Interval((springLength1 / 2 + 5) / skeleton.GetLength(), 1 - (springLength2 / 2 + 5) / skeleton.GetLength());
                return true;
            }
            else
                return false;
        }
        public bool CutModelForSpring()
        {
            BoundingBox box = model.GetBoundingBox(true);

            //Vector3d tan1 = skeleton.TangentAt(skeleton.Domain.ParameterAt(springStart));
            Vector3d tan1 = KinergyUtilities.GeometryMethods.AverageTangent(skeleton, springStart);
            //Vector3d tan2 = skeleton.TangentAt(skeleton.Domain.ParameterAt(springEnd));
            Vector3d tan2 = KinergyUtilities.GeometryMethods.AverageTangent(skeleton, springEnd);
            Plane plane2 = new Rhino.Geometry.Plane(skeleton.PointAtNormalizedLength(springEnd), -tan2);
            Plane plane1 = new Rhino.Geometry.Plane(skeleton.PointAtNormalizedLength(springStart), tan1);
            plane1.ExtendThroughBox(box, out Interval s1, out Interval t1);
            plane2.ExtendThroughBox(box, out Interval s2, out Interval t2);

            Brep[] Cut_Brep1 = model.Trim(plane1, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep[] Cut_Brep2 = model.Trim(plane2, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

            #region Test by LH
            //myDoc.Objects.AddBrep(Cut_Brep1[0]);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(Cut_Brep2[0]);
            //myDoc.Views.Redraw();
            #endregion

            Shape mc1 = new Shape(Cut_Brep1[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance), false, "model-rest1");
            Shape mc2 = new Shape(Cut_Brep2[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance), false, "model-rest2");

            modelCut.Add(mc1);
            modelCut.Add(mc2);
            //entityList.Add(mc1);
            //entityList.Add(mc2);
            return true;
        }
        public void ConstructSpring(out double inscribedCircleRadius)
        {
            double result = 0;
            Point3d startPoint = skeleton.PointAtNormalizedLength(springStart);
            Point3d endPoint = skeleton.PointAtNormalizedLength(springEnd);

            #region calculate the biggest inscribed circle radius for calculating the inner space for the spring
            Plane startPln = new Plane(startPoint, direction);
            Plane endPln = new Plane(endPoint, direction);

            Curve[] intersectStart;
            Point3d[] intersectStartPts;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(modelCut[0].GetModelinWorldCoordinate(), startPln, myDoc.ModelAbsoluteTolerance, out intersectStart, out intersectStartPts);
            if(intersectStart == null || intersectStartPts == null)
                Rhino.Geometry.Intersect.Intersection.BrepPlane(modelCut[1].GetModelinWorldCoordinate(), startPln, myDoc.ModelAbsoluteTolerance, out intersectStart, out intersectStartPts);
            Curve strCrv = intersectStart[0];

            Curve[] intersectEnd;
            Point3d[] intersectEndPts;
            Rhino.Geometry.Intersect.Intersection.BrepPlane(modelCut[1].GetModelinWorldCoordinate(), endPln, myDoc.ModelAbsoluteTolerance, out intersectEnd, out intersectEndPts);
            if(intersectEnd == null || intersectEndPts == null)
                Rhino.Geometry.Intersect.Intersection.BrepPlane(modelCut[0].GetModelinWorldCoordinate(), endPln, myDoc.ModelAbsoluteTolerance, out intersectEnd, out intersectEndPts);
            Curve endCrv = intersectEnd[0];

            double pos1, pos2;
            strCrv.ClosestPoint(startPoint, out pos1);
            endCrv.ClosestPoint(endPoint, out pos2);

            result = Math.Min(strCrv.PointAt(pos1).DistanceTo(startPoint) - wireRadius/2, endCrv.PointAt(pos2).DistanceTo(endPoint) - wireRadius/2);

            //result = Math.Max(result - (2 + 2 + 1.4 + 0.8 + 4 + 0.8 + 3), 3.5);
            
            #endregion

            #region calculate the spring diameter so that there is sufficient room for generating the lock

            springRadius = result;
            inscribedCircleRadius = springRadius;

            #endregion

            spring = new Helix(startPoint, endPoint, springRadius, wireRadius, roundNum, distance,energy);
            
            EntityList.Add(spring);
            if (spring.Model != null)
            {
                //Register constraints for spring
                _ = new Fixation(spring, modelCut[0]);
                _ = new Fixation(spring, modelCut[1]);
            }
            else { return; }
        }

        Brep createQuaterSphere(Point3d cen, double r, Vector3d z_dir, Vector3d y_dir, Vector3d x_dir)
        {
            Brep result = new Brep();

            Sphere sph = new Sphere(cen, r);
            Brep sphereBrep = sph.ToBrep();

            Plane firstPln = new Plane(cen, z_dir);
            Plane secondPln = new Plane(cen, y_dir);
            Plane thirdPln = new Plane(cen, x_dir);
            Interval interval = new Interval(-1.5 * r, 1.5 * r);
            PlaneSurface firstPlnSurf = new PlaneSurface(firstPln, interval, interval);
            PlaneSurface secondPlnSurf = new PlaneSurface(secondPln, interval, interval);
            PlaneSurface thirdPlnSurf = new PlaneSurface(thirdPln, interval, interval);
            Brep firstPlaneBrep = firstPlnSurf.ToBrep();
            Brep secondPlaneBrep = secondPlnSurf.ToBrep();
            Brep thirdPlaneBrep = thirdPlnSurf.ToBrep();


            var halfSphereBreps = Brep.CreateBooleanSplit(sphereBrep, firstPlaneBrep, myDoc.ModelAbsoluteTolerance);
            Point3d testPt = cen + z_dir;
            Brep halpSphereBrep = new Brep();
            if(halfSphereBreps[0].IsPointInside(testPt, myDoc.ModelAbsoluteTolerance, true))
            {
                halpSphereBrep = halfSphereBreps[0];
            }
            else
            {
                halpSphereBrep = halfSphereBreps[1];
            }

            var quarterSphereBreps = Brep.CreateBooleanSplit(halpSphereBrep, secondPlaneBrep, myDoc.ModelAbsoluteTolerance);
            Point3d testPt1 = cen + y_dir;
            Brep quarterSphereBrep = new Brep();
            if (quarterSphereBreps[0].IsPointInside(testPt1, myDoc.ModelAbsoluteTolerance, true))
            {
                quarterSphereBrep = quarterSphereBreps[0];
            }
            else
            {
                quarterSphereBrep = quarterSphereBreps[1];
            }

            var finalSphereBreps = Brep.CreateBooleanSplit(quarterSphereBrep, thirdPlaneBrep, myDoc.ModelAbsoluteTolerance);
            Point3d testPt2 = cen + x_dir;
            if (finalSphereBreps[0].IsPointInside(testPt2, myDoc.ModelAbsoluteTolerance, true))
            {
                result = finalSphereBreps[0];
            }
            else
            {
                result = finalSphereBreps[1];
            }

            return result;
        }

        public bool DeleteLock(GH_Document gh_d, Guid eeID)
        {
            Point3d springS = skeleton.PointAtNormalizedLength(0);
            Point3d springE = skeleton.PointAtNormalizedLength(1);

            Brep stationary_brep = new Brep();
            Brep ee_brep = (Brep)myDoc.Objects.Find(eeID).Geometry;
            int stationaryIdx = 0;
            if (ee_brep.ClosestPoint(springS).DistanceTo(springS) <= ee_brep.ClosestPoint(springE).DistanceTo(springE))
            {
                // the end-effector is near the start point of the spring
                stationary_brep = modelCut[1].GetModelinWorldCoordinate();
                stationaryIdx = 1;
            }
            else
            {
                // the end-effector is near the end point of the spring
                stationary_brep = modelCut[0].GetModelinWorldCoordinate();
                stationaryIdx = 0;
            }

            if(lockShapeIndexes.Count > 0)
            {
                lockShapeIndexes.Sort((a, b) => b.CompareTo(a)); // descending sorting
                foreach(int idx in lockShapeIndexes)
                {
                    entityList.RemoveAt(idx);
                }
            }

            Lock lockHeadShape = new Lock(stationary_brep, true, false);
            Lock lockBaseShape = new Lock(ee_brep, false, false);

            lockHeadShape.RegisterOtherPart(lockBaseShape);

            _ = new Fixation(lockBaseShape, modelCut[1 - stationaryIdx]);
            _ = new Fixation(lockHeadShape, modelCut[stationaryIdx]);

            lockShapeIndexes.Add(entityList.Count);
            lockShapeIndexes.Add(entityList.Count + 1);

            entityList.Add(lockHeadShape);
            entityList.Add(lockBaseShape);

            locks.Clear();
            locks.Add(lockHeadShape);
            locks.Add(lockBaseShape);

            return true;
        }
        public bool ConstructLock(Point3d lockButtonPos, GH_Document gh_d, double innerSpringDia, Guid eeID)
        {
            #region new version: compliant mechanism based lock

            #region Step 1: prepare all the variables

            double springStationaryHeight = spring.WireRadius * spring.RoundNum; // spring.WireRadius is actually wire diameter

            Point3d springS = skeleton.PointAtNormalizedLength(0);
            Point3d springE = skeleton.PointAtNormalizedLength(1);
            Point3d p_ee = new Point3d();
            Point3d p_stationary = new Point3d();

            double lockt;
            skeleton.ClosestPoint(lockButtonPos, out lockt);
            lockT= skeleton.Domain.NormalizedParameterAt(lockt);

            Brep stationary_brep = new Brep();
            Brep ee_brep = (Brep)myDoc.Objects.Find(eeID).Geometry;
            int stationaryIdx = 0;
            if (ee_brep.ClosestPoint(springS).DistanceTo(springS) <= ee_brep.ClosestPoint(springE).DistanceTo(springE))
            {
                // the end-effector is near the start point of the spring
                p_ee = springS;
                p_stationary = springE;

                stationary_brep = modelCut[1].GetModelinWorldCoordinate();
                stationaryIdx = 1;

            }
            else
            {
                // the end-effector is near the end point of the spring
                p_ee = springE;
                p_stationary = springS;

                stationary_brep = modelCut[0].GetModelinWorldCoordinate();
                stationaryIdx = 0;
            }

            Vector3d extrusionDir = (p_stationary - p_ee) / (p_stationary - p_ee).Length;
            double gap_spring_hook = 0.8;
            double holderHeight = 3.6;
            double lockBridgeThickness = 1.2;
            double lockQuarterSphereRadius = 3.6;
            double beamThickness = 1.2;
            double gap_beam_spring = 1.4;
            double gap_beam_wall = 2;
            double gap_spring_bridge = 3;
            double bottom_clearance = 0.8;
            double lockOffset = lockQuarterSphereRadius + gap_beam_spring;
            Point3d lockBtnPos = p_stationary  + extrusionDir * lockOffset;
            
            Plane btnPlane = new Plane(lockBtnPos, extrusionDir);
            Point3d projectedLockPos = btnPlane.ClosestPoint(lockButtonPos);
            Vector3d lockBtnDir = (projectedLockPos - lockBtnPos) / (projectedLockPos - lockBtnPos).Length;

            double bridgeGap = spring.SpringRadius + gap_spring_bridge;
            Point3d bridgePos = lockBtnPos + lockBtnDir * spring.SpringRadius;

            double lockBtnThickness = 3.6;
            double lockBtnLength = 10;
            double lockBtnHolderThickness = 2.5;
            double lockBtnRadius = 3.5;

            double lockWallThickness = 2;
            double wallBridgeClearance = 1;
            double smallLen = 1.6;

            var sweep = new SweepOneRail();
            sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            sweep.ClosedSweep = false;
            sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            #endregion

            #region Step 2: construct the button

            Point3d lockBtnHolderPos = bridgePos + lockBtnDir * (lockBridgeThickness + lockBtnLength) + extrusionDir * lockBtnThickness / 2;
            Plane btnHolderPlane = new Plane(lockBtnHolderPos, lockBtnDir);

            // create the button
            Circle btnCir = new Circle(btnHolderPlane, lockBtnRadius);
            Cylinder btnCyl = new Cylinder(btnCir, lockBtnHolderThickness);
            Brep lockBtnBrep = btnCyl.ToBrep(true, true);

            // create the button arm
            Point3d armOneEnd = bridgePos + extrusionDir * lockBtnThickness / 2;
            Line lockBtnArm = new Line(armOneEnd, lockBtnHolderPos);
            Curve lockBtnArmCrv = lockBtnArm.ToNurbsCurve();
            Brep lockBtnPipeArm = Brep.CreatePipe(lockBtnArmCrv, lockBtnThickness / 2, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            #endregion

            #region Step 3: construct the bridge
            Transform bridgeRotation = Transform.Rotation(Math.PI * 90.0 / 180.0, extrusionDir, armOneEnd);
            Transform bridgeRotationOpposite = Transform.Rotation(Math.PI, extrusionDir, armOneEnd);

            Vector3d bridgeLeftDir = lockBtnDir;
            bridgeLeftDir.Transform(bridgeRotation);

            Vector3d bridgeRightDir = bridgeLeftDir;
            bridgeRightDir.Transform(bridgeRotationOpposite);

            Point3d bridgeLeftPt = armOneEnd + lockBtnDir * lockBridgeThickness / 2 + bridgeLeftDir * (spring.SpringRadius + lockWallThickness);   
            Point3d bridgeRightPt = armOneEnd + lockBtnDir * lockBridgeThickness / 2 + bridgeRightDir * (spring.SpringRadius + lockWallThickness);

            Point3d bridgeRightSmallLenStartPt = bridgeRightPt - bridgeRightDir * (lockWallThickness + gap_beam_wall + beamThickness);
            Point3d bridgeRightSmallLenEndPt = bridgeRightSmallLenStartPt - bridgeRightDir * smallLen;
            Point3d bridgeLeftSmallLenStartPt = bridgeLeftPt - bridgeLeftDir * (lockWallThickness + gap_beam_wall + beamThickness + smallLen);
            Point3d bridgeLeftSmallLenEndPt = bridgeLeftSmallLenStartPt + bridgeLeftDir * smallLen;


            Line bridgeLn1 = new Line(bridgeRightPt, bridgeRightSmallLenStartPt);
            Line bridgeLn2 = new Line(bridgeRightSmallLenStartPt, bridgeRightSmallLenEndPt);
            Line bridgeLn3 = new Line(bridgeRightSmallLenEndPt, bridgeLeftSmallLenStartPt);
            Line bridgeLn4 = new Line(bridgeLeftSmallLenStartPt, bridgeLeftSmallLenEndPt);
            Line bridgeLn5 = new Line(bridgeLeftSmallLenEndPt, bridgeLeftPt);

            Curve bridgeCrv1 = bridgeLn1.ToNurbsCurve();
            Curve bridgeCrv2 = bridgeLn2.ToNurbsCurve();
            Curve bridgeCrv3 = bridgeLn3.ToNurbsCurve();
            Curve bridgeCrv4 = bridgeLn4.ToNurbsCurve();
            Curve bridgeCrv5 = bridgeLn5.ToNurbsCurve();

            // create the wall-related 
            Point3d wallEndPt = p_stationary;
            Line wallPath = new Line(p_stationary + extrusionDir * (lockOffset + lockQuarterSphereRadius + 1.4 + bottom_clearance), wallEndPt);
            Curve wallPathCrv = wallPath.ToNurbsCurve();

            Vector3d projWall = extrusionDir * (wallEndPt.DistanceTo(p_stationary) - lockOffset - lockBtnThickness / 2);
            Point3d projWallRightPt = bridgeRightPt + projWall;
            Point3d projWallLeftPt = bridgeLeftPt + projWall;

            Point3d wr0 = projWallRightPt + (lockBridgeThickness / 2 /*+ wallBridgeClearance + lockWallThickness*/) * lockBtnDir - bridgeRightDir * lockWallThickness;
            Point3d wr1 = projWallRightPt + (lockBridgeThickness / 2 /*+ wallBridgeClearance + lockWallThickness*/) * lockBtnDir;
            Point3d wr2 = projWallRightPt - lockBtnDir * (lockBridgeThickness / 2 + spring.SpringRadius);
            Point3d wr3 = projWallRightPt - lockBtnDir * (lockBridgeThickness / 2 + spring.SpringRadius) - bridgeRightDir * lockWallThickness;
            Point3d wr4 = wr0;

            List<Point3d> wallRightCorners = new List<Point3d>();
            wallRightCorners.Add(wr0);
            wallRightCorners.Add(wr1);
            wallRightCorners.Add(wr2);
            wallRightCorners.Add(wr3);
            wallRightCorners.Add(wr4);

            Point3d wl0 = projWallLeftPt + (lockBridgeThickness / 2 /*+ wallBridgeClearance + lockWallThickness*/) * lockBtnDir - bridgeLeftDir * lockWallThickness;
            Point3d wl1 = projWallLeftPt + (lockBridgeThickness / 2 /*+ wallBridgeClearance + lockWallThickness*/) * lockBtnDir;
            Point3d wl2 = projWallLeftPt - lockBtnDir * (lockBridgeThickness / 2 + spring.SpringRadius);
            Point3d wl3 = projWallLeftPt - lockBtnDir * (lockBridgeThickness / 2 + spring.SpringRadius) - bridgeLeftDir * lockWallThickness;
            Point3d wl4 = wl0;

            List<Point3d> wallLeftCorners = new List<Point3d>();
            wallLeftCorners.Add(wl0);
            wallLeftCorners.Add(wl1);
            wallLeftCorners.Add(wl2);
            wallLeftCorners.Add(wl3);
            wallLeftCorners.Add(wl4);

            //Point3d ws0 = projWallLeftPt + (lockBridgeThickness / 2 + wallBridgeClearance + lockWallThickness) * lockBtnDir;
            //Point3d ws1 = projWallRightPt + (lockBridgeThickness / 2 + wallBridgeClearance + lockWallThickness) * lockBtnDir;
            //Point3d ws2 = projWallRightPt + (lockBridgeThickness / 2 + wallBridgeClearance) * lockBtnDir;
            //Point3d ws3 = projWallLeftPt + (lockBridgeThickness / 2 + wallBridgeClearance) * lockBtnDir;
            //Point3d ws4 = ws0;

            //List<Point3d> wallSideCorners = new List<Point3d>();
            //wallSideCorners.Add(ws0);
            //wallSideCorners.Add(ws1);
            //wallSideCorners.Add(ws2);
            //wallSideCorners.Add(ws3);
            //wallSideCorners.Add(ws4);

            Polyline wallRightRect = new Polyline(wallRightCorners);
            Curve wallRightRectCrv = wallRightRect.ToNurbsCurve();

            Polyline wallLeftRect = new Polyline(wallLeftCorners);
            Curve wallLeftRectCrv = wallLeftRect.ToNurbsCurve();

            //Polyline wallSideRect = new Polyline(wallSideCorners);
            //Curve wallSideRectCrv = wallSideRect.ToNurbsCurve();

            Brep[] wallRightBreps = sweep.PerformSweep(wallPathCrv, wallRightRectCrv);
            Brep wallRightBrep = wallRightBreps[0];
            Brep wallRight = wallRightBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            Brep[] wallLeftBreps = sweep.PerformSweep(wallPathCrv, wallLeftRectCrv);
            Brep wallLeftBrep = wallLeftBreps[0];
            Brep wallLeft = wallLeftBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //Brep[] wallSideBreps = sweep.PerformSweep(wallPathCrv, wallSideRectCrv);
            //Brep wallSideBrep = wallSideBreps[0];
            //Brep wallSide = wallSideBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            //// drill a hole on the side wall

            //Point3d holeEnd = bridgePos + lockBtnDir * (lockBridgeThickness * 3 + wallBridgeClearance + lockWallThickness);
            //Point3d holeStart = bridgePos - lockBtnDir * lockBridgeThickness;
            //Line holeLn = new Line(holeStart, holeEnd);
            //Curve holdProjCrv = holeLn.ToNurbsCurve();
            //Brep holeBrep = Brep.CreatePipe(holdProjCrv, lockBtnThickness / 2 + 0.8, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            //var wallSideFinals = Brep.CreateBooleanDifference(wallSide, holeBrep, myDoc.ModelAbsoluteTolerance);
            //if (wallSideFinals == null)
            //{
            //    wallSide.Flip();
            //    wallSideFinals = Brep.CreateBooleanDifference(wallSide, holeBrep, myDoc.ModelAbsoluteTolerance);
            //}
            //Brep wallSideFinal = wallSideFinals[0];

            // the two holders

            Point3d bridgeStopperRight = bridgeRightPt - bridgeRightDir * (lockWallThickness + gap_beam_wall + beamThickness);
            Line rightStopperLn = new Line(bridgeRightPt, bridgeStopperRight);
            Curve rightStopperCrv = rightStopperLn.ToNurbsCurve();
            Point3d bridgeStopperRightOrigin = bridgeRightPt - lockBtnDir * (gap_spring_bridge / 2 + lockBridgeThickness / 2);

            Point3d bridgeStopperLeft = bridgeLeftPt - bridgeLeftDir * (lockWallThickness + gap_beam_wall + beamThickness);
            Line leftStopperLn = new Line(bridgeStopperLeft, bridgeLeftPt);
            Curve leftStopperCrv = leftStopperLn.ToNurbsCurve();
            Point3d bridgeStopperLeftOrigin = bridgeStopperLeft - lockBtnDir * (gap_spring_bridge / 2 + lockBridgeThickness / 2);

            Point3d sr0 = bridgeStopperRightOrigin + holderHeight / 2 * extrusionDir + gap_spring_bridge / 2 * lockBtnDir;
            Point3d sr1 = bridgeStopperRightOrigin + gap_spring_bridge / 2 * lockBtnDir - holderHeight / 2 * extrusionDir;
            Point3d sr2 = bridgeStopperRightOrigin - gap_spring_bridge / 2 * lockBtnDir - holderHeight / 2 * extrusionDir;
            Point3d sr3 = bridgeStopperRightOrigin + holderHeight / 2 * extrusionDir - gap_spring_bridge / 2 * lockBtnDir;
            Point3d sr4 = sr0;

            List<Point3d> bridgeStopperRightCorners = new List<Point3d>();
            bridgeStopperRightCorners.Add(sr0);
            bridgeStopperRightCorners.Add(sr1);
            bridgeStopperRightCorners.Add(sr2);
            bridgeStopperRightCorners.Add(sr3);
            bridgeStopperRightCorners.Add(sr4);

            Polyline bridgeStopperRightRect = new Polyline(bridgeStopperRightCorners);
            Curve bridgeStopperRightRectCrv = bridgeStopperRightRect.ToNurbsCurve();

            Point3d sl0 = bridgeStopperLeftOrigin + holderHeight / 2 * extrusionDir + gap_spring_bridge / 2 * lockBtnDir;
            Point3d sl1 = bridgeStopperLeftOrigin + gap_spring_bridge / 2 * lockBtnDir - holderHeight / 2 * extrusionDir;
            Point3d sl2 = bridgeStopperLeftOrigin - holderHeight / 2 * extrusionDir - gap_spring_bridge / 2 * lockBtnDir;
            Point3d sl3 = bridgeStopperLeftOrigin - gap_spring_bridge / 2 * lockBtnDir + holderHeight / 2 * extrusionDir;
            Point3d sl4 = sl0;
            List<Point3d> bridgeStopperLeftCorners = new List<Point3d>();
            bridgeStopperLeftCorners.Add(sl0);
            bridgeStopperLeftCorners.Add(sl1);
            bridgeStopperLeftCorners.Add(sl2);
            bridgeStopperLeftCorners.Add(sl3);
            bridgeStopperLeftCorners.Add(sl4);

            Polyline bridgeStopperLeftRect = new Polyline(bridgeStopperLeftCorners);
            Curve bridgeStopperLeftRectCrv = bridgeStopperLeftRect.ToNurbsCurve();


            Brep[] bridgeStopperRightBreps = sweep.PerformSweep(rightStopperCrv, bridgeStopperRightRectCrv);
            Brep bridgeStopperRightBrep = bridgeStopperRightBreps[0];
            Brep stopperRight = bridgeStopperRightBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            
            Brep[] bridgeStopperLeftBreps = sweep.PerformSweep(leftStopperCrv, bridgeStopperLeftRectCrv);
            Brep bridgeStopperLeftBrep = bridgeStopperLeftBreps[0];
            Brep stopperLeft = bridgeStopperLeftBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            

            // the cross-sectional area is a rectangle for the bridge

            Point3d o0 = bridgeRightPt + holderHeight / 2 * extrusionDir + lockBridgeThickness / 2 * lockBtnDir;
            Point3d o1 = bridgeRightPt + lockBridgeThickness / 2 * lockBtnDir - holderHeight / 2 * extrusionDir;
            Point3d o2 = bridgeRightPt - holderHeight / 2 * extrusionDir - lockBridgeThickness / 2 * lockBtnDir;
            Point3d o3 = bridgeRightPt - lockBridgeThickness / 2 * lockBtnDir + holderHeight / 2 * extrusionDir;
            Point3d o4 = o0;

            Point3d pl0 = bridgeRightPt + holderHeight / 2 * extrusionDir + lockBridgeThickness / 4 * lockBtnDir;
            Point3d pl1 = bridgeRightPt + lockBridgeThickness / 4 * lockBtnDir - holderHeight / 2 * extrusionDir;
            Point3d pl2 = bridgeRightPt - holderHeight / 2 * extrusionDir - lockBridgeThickness / 4 * lockBtnDir;
            Point3d pl3 = bridgeRightPt - lockBridgeThickness / 4 * lockBtnDir + holderHeight / 2 * extrusionDir;
            Point3d pl4 = pl0;

            List<Point3d> bridgeRectCorners = new List<Point3d>();
            bridgeRectCorners.Add(o0);
            bridgeRectCorners.Add(o1);
            bridgeRectCorners.Add(o2);
            bridgeRectCorners.Add(o3);
            bridgeRectCorners.Add(o4);

            List<Point3d> pivotCorners = new List<Point3d>();
            pivotCorners.Add(pl0);
            pivotCorners.Add(pl1);
            pivotCorners.Add(pl2);
            pivotCorners.Add(pl3);
            pivotCorners.Add(pl4);

            Polyline bridgeRect = new Polyline(bridgeRectCorners);
            Curve bridgeRectCrv = bridgeRect.ToNurbsCurve();

            Polyline bridgePivotRect = new Polyline(pivotCorners);
            Curve bridgePivotRectCrv = bridgePivotRect.ToNurbsCurve();

            // small length segment rect offsets
            Transform trBridgeOffset1 = Transform.Translation(bridgeRightSmallLenStartPt - bridgeRightPt);
            Transform trBridgeOffset2 = Transform.Translation(bridgeLeftSmallLenStartPt - bridgeRightSmallLenStartPt);

            // bridge segment rect offsets
            Transform trBridgeOffset3 = Transform.Translation(bridgeRightSmallLenEndPt - bridgeRightPt);
            Transform trBridgeOffset4 = Transform.Translation(bridgeLeftSmallLenEndPt - bridgeRightSmallLenEndPt);

            Brep bridgeBrep1 = sweep.PerformSweep(bridgeCrv1, bridgeRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            bridgeRectCrv.Transform(trBridgeOffset3);

            bridgePivotRectCrv.Transform(trBridgeOffset1);
            Brep bridgeBrep2 = sweep.PerformSweep(bridgeCrv2, bridgePivotRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            bridgePivotRectCrv.Transform(trBridgeOffset2);

            Brep bridgeBrep3 = sweep.PerformSweep(bridgeCrv3, bridgeRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            bridgeRectCrv.Transform(trBridgeOffset4);

            Brep bridgeBrep4 = sweep.PerformSweep(bridgeCrv4, bridgePivotRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            Brep bridgeBrep5 = sweep.PerformSweep(bridgeCrv5, bridgeRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            Brep lockBridge = Brep.CreateBooleanUnion(new List<Brep> { bridgeBrep1, bridgeBrep2, bridgeBrep3, bridgeBrep4, bridgeBrep5 }, myDoc.ModelAbsoluteTolerance)[0];

            #endregion

            #region Step 4.1: construct the two beams with the 1/4 sphere lock

            Point3d beamRightOrigin = bridgeRightPt + lockBtnDir * lockBridgeThickness / 2;
            Point3d beamRightPivotStartPt = bridgeRightPt - lockBtnDir * (gap_spring_bridge + lockBridgeThickness / 2);
            Point3d beamRightPivotEndPt = beamRightPivotStartPt - lockBtnDir * smallLen;
            Point3d beamRightPt = bridgeRightPt - lockBtnDir * (lockBridgeThickness / 2 + spring.SpringRadius - lockQuarterSphereRadius / 2);

            Transform beamRightOffset1 = Transform.Translation(beamRightPivotStartPt - beamRightOrigin);
            Transform beamRightOffset2 = Transform.Translation(beamRightPivotEndPt - beamRightOrigin);

            Line beamRightLn1 = new Line(beamRightOrigin, beamRightPivotStartPt);
            Line beamRightLn2 = new Line(beamRightPivotStartPt, beamRightPivotEndPt);
            Line beamRightLn3 = new Line(beamRightPivotEndPt, beamRightPt);

            Curve beamRightRail1 = beamRightLn1.ToNurbsCurve();
            Curve beamRightRail2 = beamRightLn2.ToNurbsCurve();
            Curve beamRightRail3 = beamRightLn3.ToNurbsCurve();

            Point3d beamLeftOrigin = bridgeLeftPt + lockBtnDir * lockBridgeThickness / 2;
            Point3d beamLeftPivotStartPt = bridgeLeftPt - lockBtnDir * (gap_spring_bridge + lockBridgeThickness / 2);
            Point3d beamLeftPivotEndPt = beamLeftPivotStartPt - lockBtnDir * smallLen;
            Point3d beamLeftPt = bridgeLeftPt - lockBtnDir * (lockBridgeThickness / 2 + spring.SpringRadius - lockQuarterSphereRadius / 2);

            Transform beamLeftOffset1 = Transform.Translation(beamLeftPivotStartPt - beamLeftOrigin);
            Transform beamLeftOffset2 = Transform.Translation(beamLeftPivotEndPt - beamLeftOrigin);

            Line beamLeftLn1 = new Line(beamLeftOrigin, beamLeftPivotStartPt);
            Line beamLeftLn2 = new Line(beamLeftPivotStartPt, beamLeftPivotEndPt);
            Line beamLeftLn3 = new Line(beamLeftPivotEndPt, beamLeftPt);

            Curve beamLeftRail1 = beamLeftLn1.ToNurbsCurve();
            Curve beamLeftRail2 = beamLeftLn2.ToNurbsCurve();
            Curve beamLeftRail3 = beamLeftLn3.ToNurbsCurve();

            Point3d br0 = beamRightOrigin + (lockWallThickness + gap_beam_wall) * bridgeLeftDir - holderHeight / 2 * extrusionDir;
            Point3d br1 = beamRightOrigin + (lockWallThickness + gap_beam_wall) * bridgeLeftDir + holderHeight / 2 * extrusionDir;
            Point3d br2 = beamRightOrigin + (lockWallThickness + gap_beam_wall + beamThickness) * bridgeLeftDir + holderHeight / 2 * extrusionDir;
            Point3d br3 = beamRightOrigin + (lockWallThickness + gap_beam_wall + beamThickness) * bridgeLeftDir - holderHeight / 2 * extrusionDir;
            Point3d br4 = br0;
            List<Point3d> beamRightRectCorners = new List<Point3d>();
            beamRightRectCorners.Add(br0);
            beamRightRectCorners.Add(br1);
            beamRightRectCorners.Add(br2);
            beamRightRectCorners.Add(br3);
            beamRightRectCorners.Add(br4);

            Point3d bpr0 = beamRightOrigin + (lockWallThickness + gap_beam_wall + beamThickness / 4) * bridgeLeftDir - holderHeight / 2 * extrusionDir;
            Point3d bpr1 = beamRightOrigin + (lockWallThickness + gap_beam_wall + beamThickness / 4) * bridgeLeftDir + holderHeight / 2 * extrusionDir;
            Point3d bpr2 = beamRightOrigin + (lockWallThickness + gap_beam_wall + beamThickness * 3 / 4) * bridgeLeftDir + holderHeight / 2 * extrusionDir;
            Point3d bpr3 = beamRightOrigin + (lockWallThickness + gap_beam_wall + beamThickness * 3 / 4) * bridgeLeftDir - holderHeight / 2 * extrusionDir;
            Point3d bpr4 = bpr0;
            List<Point3d> beamRightPivotRectCorners = new List<Point3d>();
            beamRightPivotRectCorners.Add(bpr0);
            beamRightPivotRectCorners.Add(bpr1);
            beamRightPivotRectCorners.Add(bpr2);
            beamRightPivotRectCorners.Add(bpr3);
            beamRightPivotRectCorners.Add(bpr4);

            Polyline beamRightRect = new Polyline(beamRightRectCorners);
            Curve beamRightRectCrv = beamRightRect.ToNurbsCurve();

            Polyline beamRightPivotRect = new Polyline(beamRightPivotRectCorners);
            Curve beamRightPivotRectCrv = beamRightPivotRect.ToNurbsCurve();

            Brep beamRightBrep1 = sweep.PerformSweep(beamRightRail1, beamRightRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            beamRightRectCrv.Transform(beamRightOffset2);

            beamRightPivotRectCrv.Transform(beamRightOffset1);
            Brep beamRightBrep2 = sweep.PerformSweep(beamRightRail2, beamRightPivotRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            Brep beamRightBrep3 = sweep.PerformSweep(beamRightRail3, beamRightRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            Brep beamRight = Brep.CreateBooleanUnion(new List<Brep> { beamRightBrep1, beamRightBrep2, beamRightBrep3 }, myDoc.ModelAbsoluteTolerance)[0];


            Point3d bl0 = beamLeftOrigin + (lockWallThickness + gap_beam_wall) * bridgeRightDir + holderHeight / 2 * extrusionDir;
            Point3d bl1 = beamLeftOrigin + (lockWallThickness + gap_beam_wall) * bridgeRightDir - holderHeight / 2 * extrusionDir;
            Point3d bl2 = beamLeftOrigin + (lockWallThickness + gap_beam_wall + beamThickness) * bridgeRightDir - holderHeight / 2 * extrusionDir;
            Point3d bl3 = beamLeftOrigin + (lockWallThickness + gap_beam_wall + beamThickness) * bridgeRightDir + holderHeight / 2 * extrusionDir;
            Point3d bl4 = bl0;
            List<Point3d> beamLeftRectCorners = new List<Point3d>();
            beamLeftRectCorners.Add(bl0);
            beamLeftRectCorners.Add(bl1);
            beamLeftRectCorners.Add(bl2);
            beamLeftRectCorners.Add(bl3);
            beamLeftRectCorners.Add(bl4);

            Point3d bpl0 = beamLeftOrigin + (lockWallThickness + gap_beam_wall + beamThickness / 4) * bridgeRightDir + holderHeight / 2 * extrusionDir;
            Point3d bpl1 = beamLeftOrigin + (lockWallThickness + gap_beam_wall + beamThickness / 4) * bridgeRightDir - holderHeight / 2 * extrusionDir;
            Point3d bpl2 = beamLeftOrigin + (lockWallThickness + gap_beam_wall + beamThickness * 3 / 4) * bridgeRightDir - holderHeight / 2 * extrusionDir;
            Point3d bpl3 = beamLeftOrigin + (lockWallThickness + gap_beam_wall + beamThickness * 3 / 4) * bridgeRightDir + holderHeight / 2 * extrusionDir;
            Point3d bpl4 = bpl0;
            List<Point3d> beamLeftPivotRectCorners = new List<Point3d>();
            beamLeftPivotRectCorners.Add(bpl0);
            beamLeftPivotRectCorners.Add(bpl1);
            beamLeftPivotRectCorners.Add(bpl2);
            beamLeftPivotRectCorners.Add(bpl3);
            beamLeftPivotRectCorners.Add(bpl4);

            Polyline beamLeftRect = new Polyline(beamLeftRectCorners);
            Curve beamLeftRectCrv = beamLeftRect.ToNurbsCurve();

            Polyline beamLeftPivotRect = new Polyline(beamLeftPivotRectCorners);
            Curve beamLeftPivotRectCrv = beamLeftPivotRect.ToNurbsCurve();

            Brep beamLeftBrep1 = sweep.PerformSweep(beamLeftRail1, beamLeftRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            beamLeftRectCrv.Transform(beamLeftOffset2);

            beamLeftPivotRectCrv.Transform(beamLeftOffset1);
            Brep beamLeftBrep2 = sweep.PerformSweep(beamLeftRail2, beamLeftPivotRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            Brep beamLeftBrep3 = sweep.PerformSweep(beamLeftRail3, beamLeftRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            Brep beamLeft = Brep.CreateBooleanUnion(new List<Brep> { beamLeftBrep1, beamLeftBrep2, beamLeftBrep3 }, myDoc.ModelAbsoluteTolerance)[0];

            Point3d hookRightCen = beamRightPt  + (lockWallThickness + gap_beam_wall) * bridgeLeftDir + holderHeight / 2 * extrusionDir;
            Point3d hookLeftCen = beamLeftPt + (lockWallThickness + gap_beam_wall) * bridgeRightDir + holderHeight / 2 * extrusionDir;

            Brep hookRight = createQuaterSphere(hookRightCen, lockQuarterSphereRadius, -extrusionDir, bridgeLeftDir, -lockBtnDir);
            Brep hookLeft = createQuaterSphere(hookLeftCen, lockQuarterSphereRadius, -extrusionDir, bridgeRightDir, -lockBtnDir);

            #endregion

            #region Step 4.2: create the links for both beams

            Point3d bridgeMidPt = armOneEnd + lockBtnDir * lockBridgeThickness / 2;
            Point3d linkRightPt = ((beamRightPivotEndPt + beamRightPt) / 2 + beamRightPivotEndPt) / 2 - bridgeRightDir * (lockWallThickness + gap_beam_wall + beamThickness / 2);
            Point3d linkLeftPt = ((beamLeftPivotEndPt + beamLeftPt) / 2 + beamLeftPivotEndPt) / 2 - bridgeLeftDir * (lockWallThickness + gap_beam_wall + beamThickness / 2);

            Line linkRightLn = new Line(bridgeMidPt, linkRightPt);
            Curve linkRightCrv = linkRightLn.ToNurbsCurve();

            Line linkLeftLn = new Line(bridgeMidPt, linkLeftPt);
            Curve linkLeftCrv = linkLeftLn.ToNurbsCurve();

            double linkThickness = 1;

            Point3d lr0 = bridgeMidPt + linkThickness / 2 * bridgeRightDir + holderHeight / 2 * extrusionDir;
            Point3d lr1 = bridgeMidPt + linkThickness / 2 * bridgeRightDir - holderHeight / 2 * extrusionDir;
            Point3d lr2 = bridgeMidPt + linkThickness / 2 * bridgeLeftDir - holderHeight / 2 * extrusionDir;
            Point3d lr3 = bridgeMidPt + linkThickness / 2 * bridgeLeftDir + holderHeight / 2 * extrusionDir;
            Point3d lr4 = lr0;
            List<Point3d> linkRightCorners = new List<Point3d>();
            linkRightCorners.Add(lr0);
            linkRightCorners.Add(lr1);
            linkRightCorners.Add(lr2);
            linkRightCorners.Add(lr3);
            linkRightCorners.Add(lr4);

            Polyline linkRect = new Polyline(linkRightCorners);
            Curve linkRectCrv = linkRect.ToNurbsCurve();

            Brep linkRightBrep = sweep.PerformSweep(linkRightCrv, linkRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            Brep linkLeftBrep = sweep.PerformSweep(linkLeftCrv, linkRectCrv)[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            #endregion

            #region Step 5: construct the notches on the end-effecor

            double clearance = 0.4;

            // ring

            Point3d ringBottomPt = p_ee + extrusionDir * (lockOffset + springStationaryHeight + lockQuarterSphereRadius + 1 + holderHeight + spring.RoundNum * bottom_clearance);
            Point3d ringTopPt = ringBottomPt - extrusionDir * 2;
            Line ringLn = new Line(ringBottomPt, ringTopPt);
            Curve ringCrv = ringLn.ToNurbsCurve();

            double outerR = Math.Sqrt(Math.Pow(lockQuarterSphereRadius / 2 + clearance * 3, 2) + Math.Pow(spring.SpringRadius - gap_beam_wall - beamThickness - gap_beam_spring, 2));

            Brep ringOuter = Brep.CreatePipe(ringCrv, outerR, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Brep ringInner = Brep.CreatePipe(ringCrv, outerR - lockQuarterSphereRadius / 3, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            ringOuter.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == ringOuter.SolidOrientation)
                ringOuter.Flip();

            ringInner.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == ringInner.SolidOrientation)
                ringInner.Flip();

            Brep ring = Brep.CreateBooleanDifference(ringOuter, ringInner, myDoc.ModelAbsoluteTolerance)[0];

            ring.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == ring.SolidOrientation)
                ring.Flip();

            // right notch

            Point3d notchRightStartPt = p_ee + bridgeRightDir * (spring.SpringRadius - gap_beam_wall - beamThickness - gap_beam_spring);
            Point3d notchRightEndPt = notchRightStartPt - bridgeRightDir * lockQuarterSphereRadius;
            double leftoverGap = spring.SpringRadius - gap_beam_wall - beamThickness - gap_beam_spring - lockQuarterSphereRadius;
            Line notchRightProjLn = new Line(notchRightStartPt, notchRightEndPt);
            Curve notchRightProjCrv = notchRightProjLn.ToNurbsCurve();

            Point3d nr0 = notchRightStartPt + lockBtnDir * (lockQuarterSphereRadius / 2 + clearance * 3);
            Point3d nr1 = notchRightStartPt - lockBtnDir * (lockQuarterSphereRadius / 2 + clearance * 3);
            Point3d nr2 = notchRightStartPt - lockBtnDir * (lockQuarterSphereRadius / 2 + clearance * 3) + extrusionDir * (lockOffset + springStationaryHeight + lockQuarterSphereRadius + 1 + holderHeight + spring.RoundNum * bottom_clearance);
            Point3d nr3 = notchRightStartPt + lockBtnDir * (lockQuarterSphereRadius / 2 + clearance * 3) + extrusionDir * (lockOffset + springStationaryHeight + lockQuarterSphereRadius + 1 + holderHeight + spring.RoundNum * bottom_clearance);
            Point3d nr4 = nr0;
            List<Point3d> notchRightRectCorners = new List<Point3d>();
            notchRightRectCorners.Add(nr0);
            notchRightRectCorners.Add(nr1);
            notchRightRectCorners.Add(nr2);
            notchRightRectCorners.Add(nr3);
            notchRightRectCorners.Add(nr4);

            Polyline notchRightRect = new Polyline(notchRightRectCorners);
            Curve notchRightRectCrv = notchRightRect.ToNurbsCurve();

            Brep[] notchRightBreps = sweep.PerformSweep(notchRightProjCrv, notchRightRectCrv);
            Brep notchRightBrep = notchRightBreps[0];
            Brep notchRightFirst = notchRightBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            notchRightFirst.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == notchRightFirst.SolidOrientation)
                notchRightFirst.Flip();

            // left notch

            Point3d notchLeftStartPt = p_ee + bridgeLeftDir * leftoverGap;
            Point3d notchLeftEndPt = notchLeftStartPt + bridgeLeftDir * lockQuarterSphereRadius;
            Line notchLeftProjLn = new Line(notchLeftStartPt, notchLeftEndPt);
            Curve notchLeftProjCrv = notchLeftProjLn.ToNurbsCurve();

            Point3d nl0 = notchLeftStartPt + lockBtnDir * (lockQuarterSphereRadius / 2 + clearance * 3);
            Point3d nl1 = notchLeftStartPt - lockBtnDir * (lockQuarterSphereRadius / 2 + clearance * 3);
            Point3d nl2 = notchLeftStartPt - lockBtnDir * (lockQuarterSphereRadius / 2 + clearance * 3) + extrusionDir * (lockOffset + springStationaryHeight + lockQuarterSphereRadius + 1 + holderHeight + spring.RoundNum * bottom_clearance);
            Point3d nl3 = notchLeftStartPt + lockBtnDir * (lockQuarterSphereRadius / 2 + clearance * 3) + extrusionDir * (lockOffset + springStationaryHeight + lockQuarterSphereRadius + 1 + holderHeight + spring.RoundNum * bottom_clearance);
            Point3d nl4 = nl0;
            List<Point3d> notchLeftRectCorners = new List<Point3d>();
            notchLeftRectCorners.Add(nl0);
            notchLeftRectCorners.Add(nl1);
            notchLeftRectCorners.Add(nl2);
            notchLeftRectCorners.Add(nl3);
            notchLeftRectCorners.Add(nl4);

            Polyline notchLeftRect = new Polyline(notchLeftRectCorners);
            Curve notchLeftRectCrv = notchLeftRect.ToNurbsCurve();

            Brep[] notchLefttBreps = sweep.PerformSweep(notchLeftProjCrv, notchLeftRectCrv);
            Brep notchLeftBrep = notchLefttBreps[0];
            Brep notchLeftFirst = notchLeftBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            notchLeftFirst.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == notchLeftFirst.SolidOrientation)
                notchLeftFirst.Flip();

            // create the notches

            Brep latch = Brep.CreateBooleanUnion(new List<Brep> { notchRightFirst, notchLeftFirst, ring }, myDoc.ModelAbsoluteTolerance)[0];
            latch.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == latch.SolidOrientation)
                latch.Flip();

            Point3d notchRightHookCen = notchRightStartPt + extrusionDir * (lockOffset + springStationaryHeight + lockQuarterSphereRadius + holderHeight + spring.RoundNum * bottom_clearance)
                                       + lockBtnDir * (lockQuarterSphereRadius / 2 + clearance) + bridgeRightDir * gap_beam_spring;
            Brep hookRightOffset = createQuaterSphere(notchRightHookCen, lockQuarterSphereRadius + clearance * 2, -extrusionDir, bridgeLeftDir, -lockBtnDir);

            hookRightOffset.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == hookRightOffset.SolidOrientation)
                hookRightOffset.Flip();

            //var notchRights = Brep.CreateBooleanDifference(notchRightFirst, hookRightOffset, myDoc.ModelAbsoluteTolerance);
            //if (notchRights == null)
            //{
            //    hookRightOffset.Flip();
            //    notchRights = Brep.CreateBooleanDifference(notchRightFirst, hookRightOffset, myDoc.ModelAbsoluteTolerance);
            //}
            //Brep notchRight = notchRights[0];

            Point3d notchLeftHookCen = notchLeftEndPt + extrusionDir * (lockOffset + springStationaryHeight + lockQuarterSphereRadius + holderHeight + spring.RoundNum * bottom_clearance) 
                                            + lockBtnDir * (lockQuarterSphereRadius / 2 + clearance) + bridgeLeftDir * gap_beam_spring;
            Brep hookLeftOffset = createQuaterSphere(notchLeftHookCen, lockQuarterSphereRadius + clearance * 2, -extrusionDir, -bridgeLeftDir, -lockBtnDir);

            hookLeftOffset.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == hookLeftOffset.SolidOrientation)
                hookLeftOffset.Flip();

            //var notchLefts = Brep.CreateBooleanDifference(notchLeftFirst, hookLeftOffset, myDoc.ModelAbsoluteTolerance);
            //if (notchLefts == null)
            //{
            //    hookLeftOffset.Flip();
            //    notchLefts = Brep.CreateBooleanDifference(notchLeftFirst, hookLeftOffset, myDoc.ModelAbsoluteTolerance);
            //}
            //Brep notchLeft = notchLefts[0];

            List<Brep> latches = new List<Brep>();
            latches.Add(latch);
            List<Brep> hooks = new List<Brep>();
            hooks.Add(hookRightOffset);
            hooks.Add(hookLeftOffset);

            Brep latchBrep = Brep.CreateBooleanDifference(latches, hooks, myDoc.ModelAbsoluteTolerance)[0];

            #endregion

            #region Step 7: create the cavity for the locking

            Point3d cavProjEndPt = p_stationary + extrusionDir * 0.8;
            Line cavProjPath = new Line(p_stationary + extrusionDir * (lockOffset + lockQuarterSphereRadius + 1.4 + bottom_clearance), cavProjEndPt);
            Curve capProjCrv = cavProjPath.ToNurbsCurve();

            Point3d cavProjEndPt1 = p_stationary - extrusionDir * lockOffset;
            Line cavProjPath1 = new Line(p_stationary + extrusionDir * (lockOffset + lockQuarterSphereRadius + 1.4 + bottom_clearance), cavProjEndPt1);
            Curve capProjCrv1 = cavProjPath1.ToNurbsCurve();
            Brep cavCylinder = Brep.CreatePipe(capProjCrv1, outerR /*spring.SpringRadius - gap_beam_wall*/ + clearance * 2, 
                                    false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            Point3d cavityCen = p_stationary + extrusionDir * (lockOffset + lockQuarterSphereRadius + 1.4 + bottom_clearance);

            Point3d c0 = cavityCen + lockBtnDir * (spring.SpringRadius + gap_spring_bridge + lockBtnLength + lockBtnHolderThickness) 
                                   + bridgeLeftDir * spring.SpringRadius;
            Point3d c1 = cavityCen + lockBtnDir * (spring.SpringRadius + gap_spring_bridge + lockBtnLength + lockBtnHolderThickness)
                                   + bridgeRightDir * spring.SpringRadius;
            Point3d c2 = cavityCen - lockBtnDir * gap_spring_bridge + bridgeRightDir * spring.SpringRadius;
            Point3d c3 = cavityCen - lockBtnDir * gap_spring_bridge + bridgeLeftDir * spring.SpringRadius;
            Point3d c4 = c0;

            List<Point3d> cavityCorners = new List<Point3d>();
            cavityCorners.Add(c0);
            cavityCorners.Add(c1);
            cavityCorners.Add(c2);
            cavityCorners.Add(c3);
            cavityCorners.Add(c4);

            Polyline cavityRect = new Polyline(cavityCorners);
            Curve cavityRectCrv = cavityRect.ToNurbsCurve();

            Brep[] cavityBreps = sweep.PerformSweep(capProjCrv, cavityRectCrv);
            Brep cavityBrep = cavityBreps[0];
            Brep cavity = cavityBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            var cavTotal = Brep.CreateBooleanUnion(new List<Brep> { cavCylinder, cavity }, myDoc.ModelAbsoluteTolerance)[0];

            var cavFinals = Brep.CreateBooleanDifference(stationary_brep, cavTotal, myDoc.ModelAbsoluteTolerance);
            if(cavFinals == null)
            {
                cavTotal.Flip();
                cavFinals = Brep.CreateBooleanDifference(stationary_brep, cavTotal, myDoc.ModelAbsoluteTolerance);
            }
            Brep cavFinal = cavFinals[0];


            #region show all the components

            //myDoc.Objects.AddBrep(lockBtnBrep);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(lockBtnPipeArm);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(lockBridge);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(stopperRight);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(stopperLeft);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(beamRight);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(beamLeft);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(hookRight);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(hookLeft);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(wallRight);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(wallLeft);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(notchRight);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(notchLeft);
            //myDoc.Views.Redraw();
            //myDoc.Objects.AddBrep(cavFinal);
            //myDoc.Views.Redraw();
            //myDoc.Objects.Add(linkRightBrep);
            //myDoc.Views.Redraw();
            //myDoc.Objects.Add(linkLeftBrep);
            //myDoc.Views.Redraw();

            #endregion

            #endregion

            #region Step 8: register lock heads and bases

            lockBtnBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == lockBtnBrep.SolidOrientation)
                lockBtnBrep.Flip();

            lockBtnPipeArm.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == lockBtnPipeArm.SolidOrientation)
                lockBtnPipeArm.Flip();

            lockBridge.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == lockBridge.SolidOrientation)
                lockBridge.Flip();

            stopperRight.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == stopperRight.SolidOrientation)
                stopperRight.Flip();

            stopperLeft.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == stopperLeft.SolidOrientation)
                stopperLeft.Flip();

            wallRight.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == wallRight.SolidOrientation)
                wallRight.Flip();

            wallLeft.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == wallLeft.SolidOrientation)
                wallLeft.Flip();

            beamRight.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == beamRight.SolidOrientation)
                beamRight.Flip();

            hookRight.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == hookRight.SolidOrientation)
                hookRight.Flip();

            beamLeft.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == beamLeft.SolidOrientation)
                beamLeft.Flip();

            hookLeft.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == hookLeft.SolidOrientation)
                hookLeft.Flip();

            linkRightBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == linkRightBrep.SolidOrientation)
                linkRightBrep.Flip();

            linkLeftBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == linkLeftBrep.SolidOrientation)
                linkLeftBrep.Flip();

            cavFinal.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == cavFinal.SolidOrientation)
                cavFinal.Flip();

            if (lockShapeIndexes.Count > 0)
            {
                lockShapeIndexes.Sort((a, b) => b.CompareTo(a)); // descending sorting
                foreach (int idx in lockShapeIndexes)
                {
                    entityList.RemoveAt(idx);
                }
            }

            Brep[] lockHeads = Brep.CreateBooleanUnion(new List<Brep> { lockBtnBrep, lockBtnPipeArm, lockBridge,
                                                    stopperRight, stopperLeft, wallRight, wallLeft,
                                                    beamRight, hookRight, beamLeft, hookLeft, linkRightBrep,
                                                    linkLeftBrep, cavFinal }, myDoc.ModelAbsoluteTolerance);
            Brep lockHead = lockHeads[0];

            ee_brep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == ee_brep.SolidOrientation)
                ee_brep.Flip();

            latchBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == latchBrep.SolidOrientation)
                latchBrep.Flip();

            //notchLeft.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == notchLeft.SolidOrientation)
            //    notchLeft.Flip();

            //notchRight.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == notchRight.SolidOrientation)
            //    notchRight.Flip();

            Brep[] lockBases = Brep.CreateBooleanUnion(new List<Brep> { ee_brep, latchBrep /*notchRight, notchLeft*/ }, myDoc.ModelAbsoluteTolerance);
            Brep lockBase = lockBases[0];

            Lock lockHeadShape = new Lock(lockHead, true, false);
            Lock lockBaseShape = new Lock(lockBase, false, false);

            lockHeadShape.RegisterOtherPart(lockBaseShape);

            _ = new Fixation(lockBaseShape, modelCut[1-stationaryIdx]);
            _ = new Fixation(lockHeadShape, modelCut[stationaryIdx]);

            lockShapeIndexes.Add(entityList.Count);
            lockShapeIndexes.Add(entityList.Count + 1);

            entityList.Add(lockHeadShape);
            entityList.Add(lockBaseShape);

            locks.Clear();
            locks.Add(lockHeadShape);
            locks.Add(lockBaseShape);

            #endregion
            
            #endregion
            return true;
        }
        private double GetSectionRadius(double t)
        {
            Curve[] crvs;
            Plane midPlane = new Plane(skeleton.PointAtNormalizedLength(t), KinergyUtilities.GeometryMethods.AverageTangent(skeleton,t));
            Rhino.Geometry.Intersect.Intersection.BrepPlane(model, midPlane, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out crvs, out _);
            BoundingBox bbox = crvs[0].GetBoundingBox(true);
            double Radius = bbox.Diagonal.Length / 2/Math.Sqrt(2);
            return Radius*0.7;
        }
        public override bool LoadKineticUnit()
        {
            Movement compression;
            if (lockT < springStart)
            { compression = new Movement(spring, 3, -springLength * distance); }
            else
            { compression = new Movement(spring, 3, springLength * distance);  }
            spring.SetMovement(compression);
            compression.Activate();
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
            m = spring.Activate(interval);
            m.Activate();
            return m;
        }
        private bool SkeletonGen()
        {
            // Limitations: if there are sharp cornners existing on the geometry, the generated medial axis is not accurate. 
            //              In other word, we should fillet the edges of the model if possible.
            // Convert all objects in Rhino to mesh and save as stl files in the current directory

            string dir = @"C:\KinergyTest\";
            //string dir = "\\Mac/Home/Desktop/kinetic_tool/Kinergy-master/Kinergy/Code/MotionSolver/InstantExtension/InstantExtension/bin";
            #region Bake and export the brep model as stl file
            Guid mybrepguid =Guid.Empty;
            string layername = "new_layer";
            // layer to bake the objects to
            InstTranslation.Utilities.create_layer(layername);
            //create a directory to store the stl files
            InstTranslation.Utilities.create_dir(dir);
            var doc = Rhino.RhinoDoc.ActiveDoc;

            //declare the objects attributes
            ObjectAttributes attr = new Rhino.DocObjects.ObjectAttributes();
            //set attributes
            var l = doc.Layers.FindName(layername);
            attr.LayerIndex = l.Index;

            Guid id = Guid.Empty;

            //bake the brep
            if (model.ObjectType == ObjectType.Brep)
            {
                id = doc.Objects.AddBrep(model, attr);
            }
            // add the breps to the guid
            if (id.ToString().Length > 0) mybrepguid = id;

            // select the breps in Rhino to successfully export them
            doc.Objects.Select(mybrepguid, true);
            //where to save
            string oldSTLFile = dir + @"/temp_stl.stl";
            if (File.Exists(oldSTLFile)) File.Delete(oldSTLFile);
            //and export them
            Rhino.RhinoApp.RunScript("-_Export\n\"" + oldSTLFile+"\"\n _Enter\n _Enter", true);

            //delete the breps after exporting them
            doc.Objects.Delete(mybrepguid, true);
            /*ObjRef objSel_ref;
            Guid sufObjId = Guid.Empty;
            var rc = RhinoGet.GetOneObject("Select surface or polysurface to mesh", false, ObjectType.Brep, out objSel_ref);
            if (rc == Rhino.Commands.Result.Success)
            {
                String str1 = "_ExportFileAs=_Binary ";
                String str2 = "_ExportUnfinishedObjects=_Yes ";
                String str3 = "_UseSimpleDialog=_No ";
                String str4 = "_UseSimpleParameters=_Yes ";

                String str5 = "_Enter _DetailedOptions ";
                String str6 = "_JaggedSeams=_No ";
                String str7 = "_PackTextures=_No ";
                String str8 = "_Refine=_Yes ";
                String str9 = "_SimplePlane=_Yes ";
                String str10 = "_Weld=_No ";
                String str11 = "_AdvancedOptions ";
                String str12 = "_Angle=15 ";
                String str13 = "_AspectRatio=0 ";
                String str14 = "_Distance=0.01 ";
                String str15 = "_Grid=16 ";
                String str16 = "_MaxEdgeLength=0 ";
                String str17 = "_MinEdgeLength=0.0001 ";
                String str18 = "_Enter _Enter";

                String str = str1 + str2 + str3 + str4 + str18;
                //String str = str1 + str18;
                //String str = str1 + str2 + str3 + str4 + str5 + str6 + str7 + str8 + str9 + str10 + str11 + str12 +
                //str13 + str14 + str15 + str16 + str17 + str18;
                //String str = str18;

                var stlScript = string.Format("-_Export "+oldSTLFile+str);// _ - Export \\\Mac / Home / Desktop / kinetic tool / Kinergy - master / Kinergy / Code / MotionSolver / InstantExtension / InstantExtension / bin / temp_stl.stl
                success=RhinoApp.RunScript(stlScript, true);
                model = objSel_ref.Brep();
            }
            else
            { return false; }*/
            #endregion

            List<Curve> cvs = new List<Curve>();
            Curve joined = null;

            // clean old files
            string oldFile1 = dir + @"/temp_off_skeleton.txt";
            string oldFile2 = dir + @"/temp_off.off";
            string oldFile3 = dir + @"/temp_off_convert.off";
            string oldFile4 = dir + @"/temp_off_skeleton.off";

            if (File.Exists(oldFile1)) File.Delete(oldFile1);
            if (File.Exists(oldFile2)) File.Delete(oldFile2);
            if (File.Exists(oldFile3)) File.Delete(oldFile3);
            if (File.Exists(oldFile4)) File.Delete(oldFile4);

            //var brep_mesh = Mesh.CreateFromBrep(model, MeshingParameters.FastRenderMesh)[0];

            #region Using meshlab server to convert the mesh into off file
            Process meshCompiler = new Process();
            ProcessStartInfo meshStartInfo = new ProcessStartInfo();
            meshStartInfo.CreateNoWindow = true;
            meshStartInfo.UseShellExecute = false;

            meshStartInfo.FileName = @"meshlabserver/meshlabserver.exe";

            // Note: unifying duplicated vertices is necessary
            meshStartInfo.Arguments = @" -i " + dir + @"/temp_stl.stl -o " + dir + @"/temp_off.off -s " + @"meshlabserver/clean.mlx";

            meshCompiler.StartInfo = meshStartInfo;
            meshCompiler.Start();
            meshCompiler.WaitForExit();
            #endregion

            #region call the medial axis generation cmd
            Process matCompiler = new Process();
            ProcessStartInfo startInfo = new ProcessStartInfo();
            startInfo.CreateNoWindow = true;
            //startInfo.CreateNoWindow = false;
            startInfo.UseShellExecute = false;
            startInfo.FileName = @"skeletonization/skeletonization.exe";

            startInfo.Arguments = dir + @"/temp_off.off --debug";

            matCompiler.StartInfo = startInfo;
            matCompiler.Start();
            matCompiler.WaitForExit();
            //Process.Start(startInfo);


            string curFile = dir + @"/temp_off_skeleton.txt";
            int ctrlPtNum = 0;
            //System.Threading.Thread.Sleep(10000);
            List<Point3d> maPoints = new List<Point3d>();
            string line;

            //Pass the file path and file name to the StreamReader constructor
            StreamReader sr = new StreamReader(curFile);

            //Read the first line of text
            line = sr.ReadLine();
            maPoints.Clear();

            do
            {
                // if there is only one number skip this line,
                // otherwise store those points
                string[] dots = line.Split('\t');
                if (dots.Length == 1 && maPoints.Count != 0)
                {

                    //foreach (Point3d p in maPoints)
                    //{
                    //    myDoc.Objects.AddPoint(p);
                    //}

                    Curve ma = Rhino.Geometry.Curve.CreateControlPointCurve(maPoints, 9);
                    cvs.Add(ma);
                    maPoints.Clear();
                }
                else if (dots.Length == 3)
                {
                    Point3d tempPt = new Point3d();
                    tempPt.X = Convert.ToDouble(dots[0]);
                    tempPt.Y = Convert.ToDouble(dots[1]);
                    tempPt.Z = Convert.ToDouble(dots[2]);
                    maPoints.Add(tempPt);
                    ctrlPtNum++;
                }

                line = sr.ReadLine();
            } while (line != null);
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            if (maPoints.Count != 0)
            {

                //foreach (Point3d p in maPoints)
                //{
                //    myDoc.Objects.AddPoint(p);
                //}

                Curve ma = Curve.CreateControlPointCurve(maPoints, 9);
                cvs.Add(ma);
                joined = Curve.JoinCurves(cvs)[0];

            }
            //close the file
            sr.Close();
            skeleton = joined;
            #endregion
            if (joined.GetLength() > 0)
            { return true; }
            return false;
        }

    }
}




