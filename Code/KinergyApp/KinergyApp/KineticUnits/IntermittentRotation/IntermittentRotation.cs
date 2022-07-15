﻿using System;
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
using HumanUIforKinergy.KinergyUtilities;
namespace Kinergy.KineticUnit
{
    class IntermittentRotation : KineticUnit
    {
        //The initial inputs
        private Brep model;
        private double energy; // the range on the interface is 0.1-1, the ratio of the max energy
        private double angle; // the input step angle
        private int speed; // the type of the speed from the dropdown list
        private Vector3d direction = Vector3d.Unset;
        private bool addLock;
        RhinoDoc myDoc;


        private Curve skeleton = null;
        private List<Shape> modelCut;
        private List<Entity> springPartList;
        //The initial inputs
        Brep conBrep;           // the Brep that is selected and converted
        Brep innerCavity;
        double t1, t2; // the positoins of start point and end point of the segment on the normalized skeleton
        int speedLevel;         // value of the strength slide bar
        int distanceLevel;
        int energyLevel;
        double displacement;   // value of the distance slide bar
        ContinuousTranslation motion;
        List<Arrow> lockDirCandidates;
        Arrow p;
        Helpers helperFun;

        // Variables used for different functions
        bool lockState;
        double min_wire_diamter;
        double min_coil_num;
        double amplitude;
        bool isLockSet;
        Guid selObjId;
        List<Guid> toBeBaked;

        // Region selection related variables
        Point3d center = Point3d.Unset;
        bool PlaneSelected = false;
        Guid selected = Guid.Empty;
        Vector3d skeletonVec = Vector3d.Unset;
        Vector3d selectedAxisVector = Vector3d.Unset;
        ProcessingWin processingwin = new ProcessingWin();
        WarningWin warningwin = new WarningWin();

        bool testBodySelBtn;
        bool testMotionControlPosSetBtn;
        bool testEEPosSetBtn;
        bool testMotionAxisDirSetBtn;
        bool testPreBtn;
        bool testBakeBtn;
        int motionControlMethod; // 1: press; 2: turn

        ObjectAttributes solidAttribute, orangeAttribute, redAttribute, blueAttribute, greenAttribute;
        int selectedAxisIndex;
        Guid convertedPortion;
        List<Brep> brepCut;
        Guid reserveBrepID1;
        Guid reserveBrepID2;
        Point3d motionCtrlPointSelected;

        Point3d eeCircleDotPt = new Point3d();
        Point3d eeLineDotPt = new Point3d();
        Vector3d kineticUnitDir = new Vector3d();
        Vector3d axelDir = new Vector3d();
        Point3d eeCenPt = new Point3d();
        double finalGearPositionRatio;

        Guid mainAxisArrow = Guid.Empty;
        Guid perpAxisArrowUp = Guid.Empty;
        Guid perpAxisArrowDown = Guid.Empty;
        int eeMovingDirectionSelection = 0;//-1 for error, 0 for unset, 1 for main, 2 for perp up, 3 for perp down
        GearTrainParam selectedGearTrainParam;

        double axelSpace = 0;
        double gearSpace = 0;
        public const double clearance1 = 0.3;
        public const double clearance2 = 0.6;

        List<Entity> axel_spacer_entities = new List<Entity>();
        List<Gear> gears = new List<Gear>();
        List<Entity> spring_entities = new List<Entity>();
        //List<GearParameter> gear_info = new List<GearParameter>();
        List<Point3d> lockPos = new List<Point3d>();
        bool spiralLockNorm = false;
        Vector3d spiralLockDir = new Vector3d();
        List<Lock> locks = new List<Lock>();
        double lockDisToAxis = 0;
        Point3d spiralLockCen = new Point3d();
        Shaft firstGearShaft;
        Brep cutBarrel;
        Brep addBarrel;
        private Brep _model;
        private int _selectedAxisIndex = 0;
        private int _speed;  // the range on the interface is 0-9
        private int _distance;    // the range on the interface is 0-9
        private int _energy; //  the range on the interface is 0-9
        private Vector3d _direction = Vector3d.Unset;
        private bool _addLock;
        private RhinoDoc _myDoc;
        private int _inputType;
        private Brep _innerCavity;
        private double _skeletonLen;
        private Point3d _motorRefPt;
        private Vector3d _mainAxis;
        private Vector3d _perpAxis;
        private Vector3d _otherAxis;

        private Curve _skeleton = null;
        private List<Shape> _modelCut;
        private List<Lock> _locks;
        private List<Gear> _gears = new List<Gear>();
        private GearTrainParam _gearParam;
        private Gear drivingGear;
        private Gear lastGear;
        private List<Entity> _axelsStoppers = new List<Entity>();
        List<int> lockPartIdx = new List<int>();

        Brep b1 = null, b2 = null, b3 = null;
        Entity p1 = null, p2 = null, p3 = null;
        bool reversed = false;
        private List<Entity> endEffectors = new List<Entity>();
        private int endEffectorState;
        Brep ee1Model = null, ee2Model = null;
        Entity GDWP = null, GW = null, GS = null;
        DrivingWheel drivingWheel=null;
        GenevaDrivenWheel drivenWheel = null;
        LockSelectionForSpiralSpring lockSelection;
        Helpers _helperFun;
        int old_speedValue = 5;
        int old_energyValue = 5;
        int old_strokeValue = 5;
        bool old_direction = true;

        Point3d helicalSpringLockPos = new Point3d();

        public IntermittentRotation(Brep Model, Brep innerCavity, Vector3d Direction, double Energy, double Angle, int Speed, Helpers helper, int inputType)
        {
            model = Model;
            energy = Energy;
            angle = Angle;
            direction = Direction;
            modelCut = new List<Shape>();
            myDoc = RhinoDoc.ActiveDoc;
            locks = new List<Lock>();
            speed = Speed;
            _innerCavity = innerCavity;

            _helperFun = helper;
            motionControlMethod = inputType;
        }
        public void AddSprings(List<Entity> springControl)
        {
            if (springPartList != null)
            {
                for (int i = springPartList.Count - 1; i > -0; i--)
                {
                    entityList.Remove(springPartList.ElementAt(i));
                }
            }

            foreach (Entity springPart in springControl)
            {
                entityList.Add(springPart);
                if (springPartList == null)
                {
                    springPartList = new List<Entity>();
                }
                springPartList.Add(springPart);
            }
        }

        public void AdjustParameter(Vector3d transVec, int eeMovingDirectionSelection, int speedLevel, int strokeLevel, int energyLevel, List<double> gr_list, List<GearTrainScheme> gear_schemes, bool isSpringCW, Entity spring, int motionControlMethod, ref List<Point3d> lockPos, ref bool spiralLockNorm, ref Vector3d spiralLockDir, Point3d eePos = new Point3d())
        {
            bool isSpeedChange = false;
            bool isEnergyChange = false;
            bool isStrokeChange = false;
            bool isDirChange = false;

            if (speedLevel != old_speedValue)
            {
                isSpeedChange = true;
                old_speedValue = speedLevel;
            }

            if (strokeLevel != old_strokeValue)
            {
                isStrokeChange = true;
                old_strokeValue = strokeLevel;
            }

            if (energyLevel != old_energyValue)
            {
                isEnergyChange = true;
                old_energyValue = energyLevel;
            }

            if (isSpringCW != old_direction)
            {
                old_direction = isSpringCW;
                isDirChange = true;
            }

            if (motionControlMethod == 1)
            {
                // Adjust the parameters for the helical spring control

            }
            else
            {
                // Adjust the parameters for the spiral spring control
                int schemeNum = -1;
                int paramNum = -1;

                _helperFun.mapSpeedToGears(speedLevel, gr_list, gear_schemes, out schemeNum, out paramNum);

                GearTrainParam selectedGearTrainParam = gear_schemes[schemeNum].parameters[paramNum];

                #region re-generate all the shafts, spacers, and gears

                if (isSpeedChange)
                {
                    List<Entity> axel_spacer_entities = _helperFun.genAxelsStoppers(selectedGearTrainParam.parameters, _model, motionControlMethod);
                    List<Gear> gears = _helperFun.genGears(selectedGearTrainParam.parameters, motionControlMethod, 0.4, false);

                    //delete last shaft and replace with needed structure

                    Shaft lastShaft = null;
                    foreach (Entity e in axel_spacer_entities)
                    {
                        if (e.Name == "lastShaft")
                            lastShaft = (Shaft)e;
                    }
                    axel_spacer_entities.Remove(lastShaft);

                    if (endEffectorState == 1)
                    {
                        //Replace the last shaft with a longer one with a revolute joint at end
                        Point3d pt1 = lastShaft.StartPt;
                        Point3d pt2 = pt1 + lastShaft.AxisDir * lastShaft.Len;
                        //Tell which one is closer to ee
                        double dis1 = pt1.DistanceTo(ee1Model.ClosestPoint(pt1));
                        double dis2 = pt2.DistanceTo(ee1Model.ClosestPoint(pt1));
                        //Params of replaced axel, disc, revolute joint
                        Point3d axelStart, axelEnd, discStart, revoluteJointCenter;
                        Vector3d axelDir;
                        double axelLen;
                        if (dis1 < dis2)
                        {
                            axelDir = -lastShaft.AxisDir;
                            axelStart = pt2 + axelDir * 2.75;
                            axelEnd = ee1Model.GetBoundingBox(true).Center;
                            discStart = axelStart;
                            revoluteJointCenter = pt2 + axelDir * 4.5;
                            axelLen = axelStart.DistanceTo(axelEnd);
                        }
                        else
                        {
                            axelDir = lastShaft.AxisDir;
                            axelStart = pt1 + axelDir * 2.75;
                            axelEnd = ee1Model.GetBoundingBox(true).Center;
                            discStart = axelStart;
                            revoluteJointCenter = pt1 + axelDir * 4.5;
                            axelLen = axelStart.DistanceTo(axelEnd);
                        }
                        Socket ShaftSocket = new Socket(revoluteJointCenter, axelDir);
                        Shaft newLastShaft = new Shaft(axelStart, axelLen, 2, axelDir);
                        newLastShaft.SetName("MiddleShellBreakerShaft");
                        Shaft newLastShaftDisc = new Shaft(axelStart, 1.5, 4, axelDir);
                        axel_spacer_entities.Add(ShaftSocket);
                        axel_spacer_entities.Add(newLastShaft);
                        axel_spacer_entities.Add(newLastShaftDisc);
                    }
                    else if (endEffectorState == 2)
                    {
                        //Replace the last shaft with a longer one with 2 holes and spacers
                        Point3d pt1 = lastShaft.StartPt;
                        Point3d pt2 = pt1 + lastShaft.AxisDir * lastShaft.Len;
                        //Add shaft to link 2 ees

                        ee1Model.Transform(Transform.Translation(-transVec));
                        ee2Model.Transform(Transform.Translation(-transVec));

                        Point3d axelStart = ee1Model.GetBoundingBox(true).Center;
                        Point3d axelEnd = ee2Model.GetBoundingBox(true).Center;
                        Vector3d axelDir = axelEnd - axelStart;
                        double axelLen = axelDir.Length;
                        axelDir.Unitize();
                        Shaft newLastShaft = new Shaft(axelStart, axelLen,2, axelDir);
                        newLastShaft.SetName("MiddleShellBreakerShaft");
                        //TODO Add spacer along line within model ? Not adding for now to prevent bug
                        axel_spacer_entities.Add(newLastShaft);

                        ee1Model.Transform(Transform.Translation(transVec));
                        ee2Model.Transform(Transform.Translation(transVec));
                    }
                    else
                    {
                        throw new Exception("Unexpected situation! Invalid end effector state");
                    }
                    AddGears(gears, axel_spacer_entities, selectedGearTrainParam);


                    if (endEffectorState == 2)
                    {
                        foreach (Entity en in gears)
                        {
                            en.Model.Transform(Transform.Translation(transVec));
                        }

                        foreach (Entity en in axel_spacer_entities)
                        {
                            en.Model.Transform(Transform.Translation(transVec));
                        }
                    }
                }

                #endregion

                #region re-generate the spring

                if (isEnergyChange || isStrokeChange || isDirChange || isSpeedChange)
                {
                    Spiral spiralSpring = (Spiral)spring;
                    spiralSpring.AdjustParam(direction, selectedGearTrainParam.parameters, this.Model, eeMovingDirectionSelection, energyLevel, strokeLevel, isSpringCW, ref lockPos, ref spiralLockNorm, ref spiralLockDir);
                    AddSprings(new List<Entity> { spiralSpring });

                    if (endEffectorState == 2)
                        spiralSpring.Model.Transform(Transform.Translation(transVec));
                }
                //else if ((isEnergyChange || isRoundChange || isDirChange) && isSpeedChange)
                //{
                //    Spiral spiralSpring = _helperFun.genSprings(selectedGearTrainParam.parameters, this.Model, skeleton, mainAxis, motionControlMethod, roundLevel, energyLevel, eeMovingDirectionSelection, out lockPos, out spiralLockNorm, out spiralLockDir);
                //}

                #endregion
            }
        }
        public void ConstructLocks(Vector3d transDir, List<Point3d> lockPos, bool spiralLockNorm, Vector3d spiralLockDir, GearTrainParam gtp, List<Entity> spring_entities, int motionControlMethod)
        {
            if (motionControlMethod == 1)
            {
                // add the lock for the helical spring

                double hookHeightDepth = 7;
                double rkTeethHeight = 2.25;
                double gearThickness = 3.6;
                double rackBaseHeight = 2;
                double rackOverflowLen = 4;

                // add the lock for the helical spring

                #region Step 1: decide the lock pos

                Vector3d shaftDir = gtp.parameters.ElementAt(0).center - gtp.parameters.ElementAt(1).center;
                shaftDir.Unitize();
                //Find the vector that is orthogonal to both the mainAxis and the shaftDir
                _mainAxis.Unitize();
                Vector3d rkDir = Vector3d.CrossProduct(shaftDir, _mainAxis);
                rkDir.Unitize();

                if (spiralLockNorm)
                    rkDir = -rkDir;

                Point3d lockBasePosInitial = gtp.parameters.ElementAt(0).center - rkDir * (gtp.parameters.ElementAt(0).radius + 0.6 + rkTeethHeight / 2) + shaftDir * gearThickness / 2;

                Helix helical = (Helix)spring_entities[0];
                double helicalStationaryLen = helical.WireRadius * helical.RoundNum;
                double helicalMoveRange = helical.Length - helicalStationaryLen;

                double t1, t2;
                //_skeleton.ClosestPoint(gtp.parameters.ElementAt(0).center, out t1);
                //Point3d gear1CenOnMain = _skeleton.PointAt(t1);
                Point3d gear1CenOnMain = gtp.parameters.ElementAt(0).center;

                //_skeleton.ClosestPoint(gtp.parameters.ElementAt(gtp.parameters.Count - 1).center, out t2);
                //Point3d gear2CenOnMain = _skeleton.PointAt(t2);
                Point3d gear2CenOnMain = gtp.parameters.ElementAt(gtp.parameters.Count - 1).center;

                double spaceMoveRange = gear1CenOnMain.DistanceTo(gear2CenOnMain) - rackOverflowLen;

                double realLockPosOffset = Math.Min(helicalMoveRange, spaceMoveRange);
                //Point3d lockBasePos = lockBasePosInitial + _mainAxis * realLockPosOffset - rkDir * hookHeightDepth/2;
                Point3d lockBasePos = lockBasePosInitial + _mainAxis * realLockPosOffset;

                #endregion

                #region Step 2: construct the lock head and base

                double thicknessScaler = 1;
                if (helical.SpringRadius > 15)
                    thicknessScaler = Math.Pow(helical.SpringRadius / 15, 0.5);

                //Lock helicalSpringLock = new Lock(lockBasePos, shaftDir, _mainAxis, rkDir, thicknessScaler, realLockPosOffset);
                Lock helicalSpringLockBase = new Lock(lockBasePos, shaftDir, _mainAxis, rkDir, realLockPosOffset);
                helicalSpringLockBase.SetName("lockBase");


                cutBarrel = null;
                addBarrel = null;
                double latchTipOffset = 1;
                Point3d lockHeadPos = lockBasePos - _mainAxis * latchTipOffset;

                helicalSpringLockPos = lockHeadPos;

                Point3d selectedLockPos = new Point3d();
                Curve crossLineCrv = new Line(lockHeadPos - rkDir * int.MaxValue, lockHeadPos + rkDir * int.MaxValue).ToNurbsCurve();
                Curve[] crvs;
                Point3d[] pts;
                double lockradiusdis;
                Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, model, myDoc.ModelAbsoluteTolerance, out crvs, out pts);
                if ((pts[0] - lockHeadPos) / pts[0].DistanceTo(lockHeadPos) == rkDir)
                {
                    selectedLockPos = pts[1];
                }
                else
                {
                    selectedLockPos = pts[0];
                }
                lockradiusdis = selectedLockPos.DistanceTo(lockHeadPos) * 0.7;
                Lock helicalSpringLockHead = new Lock(shaftDir, lockHeadPos, selectedLockPos, lockradiusdis, false, myDoc, ref cutBarrel, ref addBarrel, "lockHead");
                helicalSpringLockHead.SetName("lockHead");

                #endregion

                lockPartIdx.Add(entityList.Count - 1);
                entityList.Add(helicalSpringLockBase); // the detent
                lockPartIdx.Add(entityList.Count - 1);
                entityList.Add(helicalSpringLockHead); // the latch
                lockPartIdx.Add(entityList.Count - 1);
                locks.Add(helicalSpringLockBase);
                locks.Add(helicalSpringLockHead);
                helicalSpringLockBase.RegisterOtherPart(helicalSpringLockHead);
            }
            else
            {
                // add the lock for the spiral spring

                #region ask the user to select the lock position
                lockSelection = new LockSelectionForSpiralSpring(myDoc, blueAttribute, lockPos[0], lockPos[1]);
                lockDisToAxis = gtp.parameters.ElementAt(0).radius + gtp.parameters.ElementAt(1).radius;
                //spiralLockCen = (lockPos.ElementAt(0) + lockPos.ElementAt(1)) / 2;
                spiralLockCen = lockPos.ElementAt(2);
                #endregion

                if (lockPartIdx.Count > 0)
                {
                    for (int i = lockPartIdx.Count - 1; i >= 0; i--)
                    {
                        entityList.RemoveAt(i);
                    }
                    lockPartIdx.Clear();
                }

                if (locks.Count > 0)
                    locks.Clear();

                Lock LockHead;

                Vector3d centerLinkDirection = new Vector3d(spiralLockCen) - new Vector3d(lockSelection.lockCtrlPointSelected);
                double centerLinkLen = centerLinkDirection.Length;
                centerLinkDirection.Unitize();

                double ratchetRadius = Math.Min(centerLinkLen * 0.45, lockDisToAxis * 0.7);

                if (spiralLockNorm)
                {
                    Point3d tempPt = spiralLockCen;

                    if (endEffectorState == 2)
                    {
                        tempPt.Transform(Transform.Translation(transDir));
                    }

                    LockHead = new Lock(tempPt, spiralLockDir, ratchetRadius, true);
                }

                else
                {
                    Point3d tempPt = spiralLockCen;

                    if (endEffectorState == 2)
                    {
                        tempPt.Transform(Transform.Translation(transDir));
                    }

                    LockHead = new Lock(tempPt, spiralLockDir, ratchetRadius, false);
                }

                LockHead.SetName("lockHead");

                #region add lock parts to the entitylist

                cutBarrel = null;
                addBarrel = null;
                Lock lockBase = new Lock(spiralLockDir, spiralLockCen, lockSelection.lockCtrlPointSelected, ratchetRadius, false, myDoc, ref cutBarrel, ref addBarrel, "lockbase");

                if (endEffectorState == 2)
                {
                    lockBase.Model.Transform(Transform.Translation(transDir));
                    addBarrel.Transform(Transform.Translation(transDir));
                    cutBarrel.Transform(Transform.Translation(transDir));
                }

                lockBase.SetName("lockBase");

                //Entity tempAddBarrel = new Entity(addBarrel, false, "lockBarrel");
                //entityList.Add(tempAddBarrel);

                //lockBase.Model.Transform(Transform.Translation(transDir));
                //LockHead.Model.Transform(Transform.Translation(transDir));
                //cutBarrel.Transform(Transform.Translation(transDir));
                //tempAddBarrel.Model.Transform(Transform.Translation(transDir));

                lockPartIdx.Add(entityList.Count - 1);
                entityList.Add(LockHead); // the ratchet gear
                lockPartIdx.Add(entityList.Count - 1);
                entityList.Add(lockBase); // the latch
                lockPartIdx.Add(entityList.Count - 1);
                locks.Add(LockHead);
                locks.Add(lockBase);
                LockHead.RegisterOtherPart(lockBase);

                foreach (Entity en in entityList)
                {
                    if (en.Name.Equals("SpiralShaft"))
                    {
                        firstGearShaft = (Shaft)en;
                        _ = new Fixation(firstGearShaft, LockHead);
                    }
                }
                #endregion

            }
        }

        public void RemoveLocks(int controlMethod)
        {
            if (controlMethod == 1)
            {
                // remove the lock for the helical spring

            }
            else
            {
                // remove the lock for the spiral spring
                if (lockPartIdx.Count > 0)
                {
                    // delete all the entities that have already been registered
                    List<int> toRemoveEntityIndexes = new List<int>();

                    foreach (Entity en in entityList)
                    {
                        if (en.Name.Equals("lockBarrel") || en.Name.Equals("lockBase") || en.Name.Equals("lockHead"))
                        {
                            int idx = entityList.IndexOf(en);
                            toRemoveEntityIndexes.Add(idx);
                        }
                    }

                    toRemoveEntityIndexes.Sort((a, b) => b.CompareTo(a)); //sorting by descending
                    foreach (int idx in toRemoveEntityIndexes)
                    {
                        entityList.RemoveAt(idx);
                    }

                    lockPartIdx.Clear();
                    locks.Clear();
                    lockDisToAxis = 0;
                }

            }

        }


        //public void ConstructLocks(List<Point3d> lockPos, bool spiralLockNorm, Vector3d spiralLockDir, GearTrainParam gtp, int motionControlMethod)
        //{
        //    if (motionControlMethod == 1)
        //    {
        //        // add the lock for the helical spring

        //    }
        //    else
        //    {
        //        // add the lock for the spiral spring

        //        #region ask the user to select the lock position
        //        LockSelectionForSpiralSpring lockSelection = new LockSelectionForSpiralSpring(myDoc, blueAttribute, lockPos[0], lockPos[1]);
        //        lockDisToAxis = gtp.parameters.ElementAt(0).radius + gtp.parameters.ElementAt(1).radius;
        //        spiralLockCen = (lockPos.ElementAt(0) + lockPos.ElementAt(1)) / 2;
        //        #endregion

        //        if (lockPartIdx.Count > 0)
        //        {
        //            for (int i = lockPartIdx.Count - 1; i >= 0; i--)
        //            {
        //                entityList.RemoveAt(i);
        //            }
        //            lockPartIdx.Clear();
        //        }

        //        if (locks.Count > 0)
        //            locks.Clear();

        //        Lock LockHead;
        //        double ratchetRadius = lockDisToAxis * 0.5;

        //        if (spiralLockNorm)
        //            LockHead = new Lock(spiralLockCen, spiralLockDir, ratchetRadius, true);
        //        else
        //            LockHead = new Lock(spiralLockCen, spiralLockDir, ratchetRadius, false);

        //        Vector3d centerLinkDirection = new Vector3d(spiralLockCen) - new Vector3d(lockSelection.lockCtrlPointSelected);
        //        double centerLinkLen = centerLinkDirection.Length;
        //        centerLinkDirection.Unitize();


        //        #region add lock parts to the entitylist

        //        cutBarrel = null;
        //        addBarrel = null;
        //        Lock lockBase = new Lock(spiralLockDir, spiralLockCen, lockSelection.lockCtrlPointSelected, ratchetRadius, false, myDoc, ref cutBarrel, ref addBarrel, "lockbase");
        //        Entity tempAddBarrel = new Entity(addBarrel, false, "");
        //        entityList.Add(tempAddBarrel);

        //        lockPartIdx.Add(entityList.Count - 1);
        //        entityList.Add(LockHead); // the ratchet gear
        //        lockPartIdx.Add(entityList.Count - 1);
        //        entityList.Add(lockBase); // the latch
        //        lockPartIdx.Add(entityList.Count - 1);
        //        locks.Add(LockHead);
        //        locks.Add(lockBase);
        //        LockHead.RegisterOtherPart(lockBase);

        //        foreach (Entity en in entityList)
        //        {
        //            if (en.Name.Equals("MiddleShellBreakerShaft"))
        //            {
        //                firstGearShaft = (Shaft)en;
        //                _ = new Fixation(firstGearShaft, LockHead);
        //            }
        //        }
        //        #endregion

        //    }
        //}
        public void AddGears(List<Gear> gears, List<Entity> axelsStoppers, GearTrainParam gearParam)
        {
            _gearParam = gearParam;
            drivingGear = gears[0];
            lastGear = gears.Last();
            //First remove existing gears before adding new
            foreach (Gear g in _gears)
            {
                entityList.Remove(g);
            }
            _gears.Clear();
            foreach (Entity e in _axelsStoppers)
            {
                entityList.Remove(e);
            }
            _axelsStoppers.Clear();
            foreach (Gear g in gears)
            {
                _gears.Add(g);
                entityList.Add(g);
            }
            foreach (Entity e in axelsStoppers)
            {
                _axelsStoppers.Add(e);
                entityList.Add(e);
            }
            //TODO register connecting relations
        }
        public void Set3Parts(double T1, double T2, Brep B1, Brep B2, Brep B3)
        {
            t1 = T1;
            t2 = T2;
            if (t1 > t2)
            {
                double t = t1;
                t1 = t2;
                t2 = t;
            }
            b1 = B1;
            b2 = B2;
            b3 = B3;
            //Tell if motion control point is closer to B1 or B3. make sure b1 is driving part and b3 is ee part
            double dis1 = _motorRefPt.DistanceTo(b1.ClosestPoint(_motorRefPt));
            double dis3 = _motorRefPt.DistanceTo(b3.ClosestPoint(_motorRefPt));
            if (dis1 > dis3)
            {
                Brep t = b1;
                b1 = b3;
                b3 = t;
            }
            entityList.Remove(p1);
            entityList.Remove(p2);
            entityList.Remove(p3);
        }
        public void Set3Axis(Vector3d main, Vector3d perp, Vector3d shaft)
        {
            _mainAxis = main;
            _perpAxis = perp;
            _otherAxis = shaft;
            //Use main axis to tell if b1 and b3 is reversed
            if ((b3.GetBoundingBox(true).Center - b1.GetBoundingBox(true).Center) * _mainAxis < 0)
            {
                Brep t = b1;
                b1 = b3;
                b3 = t;
            }
        }
        public void SetEndEffectors(int eeState, List<Brep> ees)
        {
            endEffectorState = eeState;
            if (endEffectors.Count > 0)
            {
                foreach (Entity e in endEffectors)
                {
                    entityList.Remove(e);
                }
            }
            endEffectors.Clear();
            foreach (Brep b in ees)
            {
                Entity ee = new Entity(b, false, "endEffector");
                entityList.Add(ee);
                endEffectors.Add(ee);
            }
            if (eeState == 1)
                ee1Model = ees[0];
            else if (eeState == 2)
            {
                ee1Model = ees[0];
                ee2Model = ees[1];
            }
        }
        public void AddGenevaDrive(Entity GenevaDrivingWheelWithPin,Entity  GenevaWheel,Entity  GenevaStopper,GenevaDrive drive)
        {
            entityList.Remove(GDWP);
            entityList.Remove(GW);
            entityList.Remove(GS);
            entityList.Remove(drivingWheel);
            entityList.Remove(drivenWheel);
            GDWP = GenevaDrivingWheelWithPin;
            GW = GenevaWheel;
            GS = GenevaStopper;
            drivingWheel = drive.drivingWheel;
            drivenWheel = drive.drivenWheel;
            _ = new Engagement(drivingWheel, drivenWheel);
            entityList.Add(GDWP);
            entityList.Add(GW);
            entityList.Add(GS);
            entityList.Add(drivingWheel);
            entityList.Add(drivenWheel);
        }
        public void CreateShell(Brep socketBrep)
        {
            double shellThickness = 2;
            Brep part2=(Brep)b2.Duplicate();
            part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                part2.Flip();

            foreach (Entity e in entityList)
            {
                if (e.Name == "MiddleShellBreakerShaft" || e.Name == "SpiralShaft")
                {
                    Shaft s = (Shaft)e;
                    Plane p = new Plane(s.StartPt, s.AxisDir);
                    Circle c = new Circle(p, 3);
                    Cylinder cy = new Cylinder(c, s.Len);
                    part2 = Brep.CreateBooleanDifference(part2, cy.ToBrep(true, true), myDoc.ModelAbsoluteTolerance)[0];
                    part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                    if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                        part2.Flip();
                }
            }
            Brep[] shells = Brep.CreateOffsetBrep(b2, (-1) * shellThickness, false, true, myDoc.ModelRelativeTolerance, out _, out _);
            Brep innerShell = shells[0];
            innerShell.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == innerShell.SolidOrientation)
                innerShell.Flip();
            part2 = Brep.CreateBooleanDifference(part2, innerShell, myDoc.ModelAbsoluteTolerance)[0];
            part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                part2.Flip();
            //Cut b2 with shaft if needed

            //Cut part 2 open
            #region Cut part 2 open with box
            BoundingBox inncerCavityBbox = _innerCavity.GetBoundingBox(true);
            double bboxMainDimension = 0, bboxPerpDimension = 0, bboxOtherDimension = 0;

            if ((Vector3d.XAxis * _mainAxis) == 1 || (Vector3d.XAxis * _mainAxis) == -1)
            {
                bboxMainDimension = inncerCavityBbox.Max.X - inncerCavityBbox.Min.X;

            }
            else if ((Vector3d.YAxis * _mainAxis) == 1 || (Vector3d.YAxis * _mainAxis) == -1)
            {
                bboxMainDimension = inncerCavityBbox.Max.Y - inncerCavityBbox.Min.Y;

            }
            else
            {
                bboxMainDimension = inncerCavityBbox.Max.Z - inncerCavityBbox.Min.Z;

            }
            Plane boxPlane = new Plane(inncerCavityBbox.Center, _mainAxis, _otherAxis);
            Brep cutBox = new Box(boxPlane, new Interval(-bboxMainDimension * 0.4, bboxMainDimension * 0.4), new Interval(-15, 15)
                , new Interval(0, bboxMainDimension * 5)).ToBrep();

            if (motionControlMethod == 2 && lockSelection != null)
            {
                if ((lockSelection.lockCtrlPointSelected - inncerCavityBbox.Center) * boxPlane.Normal > 0)
                    cutBox = new Box(boxPlane, new Interval(-bboxMainDimension * 0.4, bboxMainDimension * 0.4), new Interval(-15, 15)
                    , new Interval(-bboxMainDimension * 5, 0)).ToBrep();
            }
            else if (motionControlMethod == 1)
            {
                if ((helicalSpringLockPos - inncerCavityBbox.Center) * boxPlane.Normal > 0)
                    cutBox = new Box(boxPlane, new Interval(-bboxMainDimension * 0.4, bboxMainDimension * 0.4), new Interval(-15, 15)
                    , new Interval(-bboxMainDimension * 5, 0)).ToBrep();
            }

            cutBox.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == cutBox.SolidOrientation)
                cutBox.Flip();
            part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                part2.Flip();

            //myDoc.Objects.AddBrep(part2);
            //myDoc.Views.Redraw();

            Brep[] cutResult = Brep.CreateBooleanDifference(part2, cutBox, myDoc.ModelAbsoluteTolerance);
            part2 = cutResult[0];

            part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                part2.Flip();
            try
            {
                cutBarrel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == cutBarrel.SolidOrientation)
                    cutBarrel.Flip();

                addBarrel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == addBarrel.SolidOrientation)
                    addBarrel.Flip();

                part2 = Brep.CreateBooleanDifference(part2, cutBarrel, myDoc.ModelAbsoluteTolerance)[0];
                part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                    part2.Flip();

                part2 = Brep.CreateBooleanUnion(new List<Brep> { part2, addBarrel }, myDoc.ModelAbsoluteTolerance)[0];
                part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                    part2.Flip();
            }
            catch { }

            #endregion
            Brep part1=b1,part3 = b3;

            if (socketBrep != null)
            {
                Brep socketBrepDup1 = socketBrep.DuplicateBrep();
                if (Brep.CreateBooleanDifference(part1, socketBrepDup1, myDoc.ModelAbsoluteTolerance) != null)
                {
                    if (Brep.CreateBooleanDifference(part1, socketBrepDup1, myDoc.ModelAbsoluteTolerance).Count() > 0)
                    {
                        part1 = Brep.CreateBooleanDifference(part1, socketBrepDup1, myDoc.ModelAbsoluteTolerance)[0];
                        part1.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                        if (BrepSolidOrientation.Inward == part1.SolidOrientation)
                            part1.Flip();
                    }
                }

                Brep socketBrepDup2 = socketBrep.DuplicateBrep();
                if (Brep.CreateBooleanDifference(part2, socketBrepDup2, myDoc.ModelAbsoluteTolerance) != null)
                {
                    if (Brep.CreateBooleanDifference(part2, socketBrepDup2, myDoc.ModelAbsoluteTolerance).Count() > 0)
                    {
                        part2 = Brep.CreateBooleanDifference(part2, socketBrepDup2, myDoc.ModelAbsoluteTolerance)[0];
                        part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                        if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                            part2.Flip();
                    }

                }

                Brep socketBrepDup3 = socketBrep.DuplicateBrep();
                if (Brep.CreateBooleanDifference(part3, socketBrepDup3, myDoc.ModelAbsoluteTolerance) != null)
                {
                    if (Brep.CreateBooleanDifference(part3, socketBrepDup3, myDoc.ModelAbsoluteTolerance).Count() > 0)
                    {
                        part3 = Brep.CreateBooleanDifference(part3, socketBrepDup3, myDoc.ModelAbsoluteTolerance)[0];
                        part3.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                        if (BrepSolidOrientation.Inward == part3.SolidOrientation)
                            part3.Flip();
                    }
                }
            }

            entityList.Remove(p1);
            entityList.Remove(p2);
            entityList.Remove(p3);
            p1 = new Entity(part1);
            entityList.Add(p1);
            p2 = new Entity(part2);
            entityList.Add(p2);
            p3 = new Entity(part3);
            entityList.Add(p3);
            foreach(Entity e in endEffectors)
            {
                _ = new Fixation(e, drivenWheel);
            }
        }
        public override Movement Simulate(double interval = 20, double precision = 0.01)
        {
            Movement m = null;
            if (motionControlMethod == 1)
            {
                Helix h = null;
                foreach (Entity e in springPartList)
                {
                    if (e.GetType() == typeof(Helix))
                        h = (Helix)e;
                }

                m = h.Activate(interval);
                m.Activate();
            }
            else if (motionControlMethod == 2)
            {
                Spiral s = null;
                foreach (Entity e in springPartList)
                {
                    if (e.GetType() == typeof(Spiral))
                        s = (Spiral)e;
                }
                m = s.Activate(interval);
                m.Activate();
            }
            return m;
        }
        public override bool LoadKineticUnit()
        {
            Movement compression;
            if (motionControlMethod == 1)
            {
                Helix h = null;
                foreach (Entity e in springPartList)
                {
                    if (e.GetType() == typeof(Helix))
                        h = (Helix)e;
                }

                compression = new Movement(h, 3, -h.Length * _distance * 0.9);
                //h.SetMovement(compression);
                compression.Activate();
            }
            else if (motionControlMethod == 2)
            {
                Spiral s = null;
                foreach (Entity e in springPartList)
                {
                    if (e.GetType() == typeof(Spiral))
                        s = (Spiral)e;
                }
                double degree = _distance / 10.0 * 2 * Math.PI;
                compression = new Movement(s, 4, degree);
                //s.SetMovement(compression);
                compression.Activate();
            }
            if (locks.Count > 0)
                locks[0].SetLocked();
            Loaded = true;
            return true;
        }
        public List<Lock> Locks { get => locks; set => locks = value; }
        public List<Shape> ModelCut { get => modelCut; set => modelCut = value; }
        public Curve Skeleton { get => skeleton; set => skeleton = value; }
        public Brep Model { get => model; set => model = value; }
        public double Energy { get => energy; set => energy = value; }
        public int Speed { get => speed; set => speed = value; }
        public double Angle { get => angle; set => angle = value; }
        public bool Old_direction { get => old_direction; set => old_direction = value; }
    }
}
