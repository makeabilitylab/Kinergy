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
using System.Drawing;

namespace Kinergy.KineticUnit
{
    class ContinuousRotation : KineticUnit
    {
        //The initial inputs
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
        private List<Entity> springPartList;
        private List<Entity> _axelsStoppers = new List<Entity>();
        private Helix _spring;
        private Rack endEffectorRack = null;
        private List<Entity> endEffectorRackConfiningStructure = new List<Entity>();
        private Entity endEffectorConnectingStructure = null;
        Brep constrainingStructureSpaceTaken;
        private int _shaftNum;
        private List<List<int>> _r_shaft_num_List;
        private List<List<double>> _shaft_radius_pool;
        private List<Entity> endEffectors = new List<Entity>();
        private int endEffectorState;
        public const double clearance1 = 0.3;
        public const double clearance2 = 0.6;
        public const double gearFaceWidth = 3.6;
        public const double gearModule = 1;
        public const double gearPressureAngle = 20;
        public const double shaftRadius = 2;
        public RhinoDoc myDoc = RhinoDoc.ActiveDoc;

        Brep b1 = null, b2 = null, b3 = null;
        Entity p1 = null, p2 = null, p3 = null;
        bool reversed = false;
        double t1 = 0, t2 = 0;

        Helpers _helperFun;
        int old_speedValue = -1;
        int old_energyValue = -1;
        int old_roundValue = -1;
        bool old_direction = true;
        Brep ee1Model = null, ee2Model = null;


        // New lock approach
        double lockDisToAxis = 0;
        ObjectAttributes blueAttribute;
        Point3d spiralLockCen = new Point3d();
        List<int> lockPartIdx = new List<int>();
        int midPartIdx = -1;
        List<Lock> locks = new List<Lock>();
        Brep cutBarrel;
        Brep addBarrel;
        Shaft firstGearShaft;
        LockSelectionForSpiralSpring lockSelection;
        int motionControlMethod; // 1: press; 2: turn
        Point3d helicalSpringLockPos = new Point3d();

        public ContinuousRotation(Brep Model, int selectedAxisIndex, Vector3d Direction, Brep innerCavity, Point3d motionCtrlPt, int speed, int dis, int eneryg, int InputType, Helpers helper, int inputMethod)
        {
            _model = Model;
            _selectedAxisIndex = selectedAxisIndex;
            _speed = speed;
            _energy = eneryg;
            _distance = dis;
            _direction = Direction;
            _modelCut = new List<Shape>();
            _myDoc = RhinoDoc.ActiveDoc;
            _locks = new List<Lock>();
            _inputType = InputType;
            _motorRefPt = motionCtrlPt;

            _innerCavity = innerCavity;
            BoxLike currB = new BoxLike(_model, _direction);

            _skeleton = currB.Skeleton;
            _skeleton.Transform(currB.RotateBack);
            _skeletonLen = _skeleton.PointAtNormalizedLength(0).DistanceTo(_skeleton.PointAtNormalizedLength(1));

            _shaftNum = 0;
            _r_shaft_num_List = new List<List<int>>();

            _helperFun = helper;

            int blueIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material blueMat = myDoc.Materials[blueIndex];
            blueMat.DiffuseColor = System.Drawing.Color.FromArgb(16, 150, 206);
            blueMat.SpecularColor = System.Drawing.Color.FromArgb(16, 150, 206);
            blueMat.Transparency = 0.7f;
            blueMat.TransparentColor = System.Drawing.Color.FromArgb(16, 150, 206);
            blueMat.CommitChanges();
            blueAttribute = new ObjectAttributes();
            //blueAttribute.LayerIndex = 5;
            blueAttribute.MaterialIndex = blueIndex;
            blueAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            blueAttribute.ObjectColor = Color.FromArgb(16, 150, 206);
            blueAttribute.ColorSource = ObjectColorSource.ColorFromObject;

            motionControlMethod = inputMethod;
        }
        //public ContinuousRotation(Brep Model, Vector3d Direction, double Energy, double Speed, int InputType, Brep inCavity)
        //{
        //    this._model = Model;
        //    this._energy = Energy;
        //    this._speed = Speed;
        //    this._direction = Direction;
        //    this.modelCut = new List<Shape>();
        //    this._myDoc = RhinoDoc.ActiveDoc;
        //    this.locks = new List<Lock>();
        //    this._inputType = InputType;
        //    this._innerCavity = inCavity;
        //    this.skeleton = null;

        //}

        public void AdjustParameter(Vector3d transVec, int eeMovingDirectionSelection, int speedLevel, int roundLevel, int energyLevel, List<double> gr_list, List<GearTrainScheme> gear_schemes, bool isSpringCW, Entity spring, int motionControlMethod, ref List<Point3d> lockPos, ref bool spiralLockNorm, ref Vector3d spiralLockDir, Point3d eePos = new Point3d())
        {
            bool isSpeedChange = false;
            bool isEnergyChange = false;
            bool isRoundChange = false;
            bool isDirChange = false;

            if (speedLevel != old_speedValue)
            {
                isSpeedChange = true;
                old_speedValue = speedLevel;
            }

            if (roundLevel != old_roundValue)
            {
                isRoundChange = true;
                old_roundValue = roundLevel;
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
                    List<Entity> axel_spacer_entities = _helperFun.genAxelsStoppers(selectedGearTrainParam.parameters, _model, motionControlMethod, 0.3);
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
                        Shaft newLastShaftDisc = new Shaft(axelStart, 1.5, 3.8, axelDir);
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
                        Shaft newLastShaft = new Shaft(axelStart, axelLen, 2, axelDir);
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

                if (isEnergyChange || isRoundChange || isDirChange || isSpeedChange)
                {
                    Spiral spiralSpring = (Spiral)spring;
                    spiralSpring.AdjustParam(direction, selectedGearTrainParam.parameters, this.Model, eeMovingDirectionSelection, energyLevel, roundLevel, isSpringCW, ref lockPos, ref spiralLockNorm, ref spiralLockDir);
                    AddSprings(new List<Entity> { spiralSpring });

                    if(endEffectorState == 2)
                        spiralSpring.Model.Transform(Transform.Translation(transVec));
                }
                //else if ((isEnergyChange || isRoundChange || isDirChange) && isSpeedChange)
                //{
                //    Spiral spiralSpring = _helperFun.genSprings(selectedGearTrainParam.parameters, this.Model, skeleton, mainAxis, motionControlMethod, roundLevel, energyLevel, eeMovingDirectionSelection, out lockPos, out spiralLockNorm, out spiralLockDir);
                //}

                #endregion
            }
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
                _skeleton.ClosestPoint(gtp.parameters.ElementAt(0).center, out t1);
                Point3d gear1CenOnMain = _skeleton.PointAt(t1);
                _skeleton.ClosestPoint(gtp.parameters.ElementAt(gtp.parameters.Count - 1).center, out t2);
                Point3d gear2CenOnMain = _skeleton.PointAt(t2);
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
                Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, this._model, myDoc.ModelAbsoluteTolerance, out crvs, out pts);
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

                //ToDo add two end-effector case -- translate the entire lock and the spring
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
                    midPartIdx = -1;
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

                    if(endEffectorState == 2)
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
                    if (en.Name.Equals("MiddleShellBreakerShaft"))
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

        public class gearSetDesign
        {
            private double _module;
            private double _gearRatio;
            private double _pinionRadius;
            private int _numSet;
            private double _grUnit;

            public gearSetDesign(double m, double gr, double pr, int num, double grU) {
                _module = m;
                _gearRatio = gr;
                _pinionRadius = pr;
                _numSet = num;
                _grUnit = grU;
            }

            public double Module { get => _module; set => _module = value; }
            public double GearRatio { get => _gearRatio; set => _gearRatio = value; }
            public double PinionRadius { get => _pinionRadius; set => _pinionRadius = value; }
            public int NumSet { get => _numSet; set => _numSet = value; }
            public double GrUnit { get => _grUnit; set => _grUnit = value; }
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
        public void Set3Axis(Vector3d main,Vector3d perp,Vector3d shaft)
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
        public void SetEndEffectors(int eeState,List<Brep> ees)
        {
            endEffectorState = eeState;
            if(endEffectors.Count>0)
            {
                foreach(Entity e in endEffectors)
                {
                    entityList.Remove(e);
                }
            }
            endEffectors.Clear();
            foreach(Brep b in ees)
            {
                Entity ee = new Entity(b, false, "endEffector");
                entityList.Add(ee);
                endEffectors.Add(ee);
            }
            if (eeState == 1)
                ee1Model = ees[0];
            else if(eeState==2)
            {
                ee1Model = ees[0];
                ee2Model = ees[1];
            }
        }
        public void CreateShell(Brep socketBrep)
        {
            double shellThickness = 2;
            Brep part2=b2.DuplicateBrep();
            part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                part2.Flip();

            b2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == b2.SolidOrientation)
                b2.Flip();

            foreach (Entity e in entityList)
            {
                if (e.Name == "MiddleShellBreakerShaft")
                {
                    Shaft s = (Shaft)e;
                    //Plane p = new Plane(s.StartPt, s.AxisDir);
                    //Circle c = new Circle(p, 2.1);
                    //Cylinder cy = new Cylinder(c, s.Len);
                    //part2 = Brep.CreateBooleanDifference(part2, cy.ToBrep(true, true), myDoc.ModelAbsoluteTolerance)[0];
                    Brep[] shaftEnlargedList = Brep.CreateOffsetBrep(s.Model, clearance2, false, true, myDoc.ModelRelativeTolerance, out _, out _);
                    Brep shaftEnlarged = shaftEnlargedList[0];
                    shaftEnlarged.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                    if (BrepSolidOrientation.Inward == shaftEnlarged.SolidOrientation)
                        shaftEnlarged.Flip();

                    part2 = Brep.CreateBooleanDifference(part2, shaftEnlarged, myDoc.ModelAbsoluteTolerance)[0];
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

            GearParameter lgp = _gearParam.parameters.Last();
            Brep lgCylinder = new Cylinder(new Circle(new Plane(lgp.center, lgp.norm), lgp.radius + 2), lgp.faceWidth + 0.6 + 1.3 * 2).ToBrep(true, true);
            lgCylinder.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == lgCylinder.SolidOrientation)
                lgCylinder.Flip();

            Brep lgCylinderDup = lgCylinder.DuplicateBrep();
            lgCylinder.Transform(Transform.Translation(-lgp.norm * 1.6));
            try
            {
                part2 = Brep.CreateBooleanDifference(part2, lgCylinder, myDoc.ModelAbsoluteTolerance)[0];
                part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                    part2.Flip();
            }
            catch { }
            
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
            if(motionControlMethod == 2 && lockSelection != null)
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

            //Cut b3 with gear cylinder
            Brep part3 = b3;
            part3.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == part3.SolidOrientation)
                part3.Flip();

            try
            {
                part3 = Brep.CreateBooleanDifference(part3, lgCylinderDup, myDoc.ModelAbsoluteTolerance)[0];
                part3.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == part3.SolidOrientation)
                    part3.Flip();
            }
            catch { }
            //Cut b3 with a horizontal rod to let water in
            //BoundingBox part3bbox = part3.GetBoundingBox(true);
            //Brep cuttingRod = new Cylinder(new Circle(new Plane(part3bbox.Center, _otherAxis), 5), bboxMainDimension * 10).ToBrep(true, true);
            //cuttingRod.Transform(Transform.Translation(_otherAxis * (-bboxMainDimension * 5)));
            //cuttingRod.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == cuttingRod.SolidOrientation)
            //    cuttingRod.Flip();
            //try
            //{
            //    part3 = Brep.CreateBooleanDifference(part3, cuttingRod, myDoc.ModelAbsoluteTolerance)[0];
            //}
            //catch { }
            //Cut part 3 with plane to greate a gap
            //Plane cutter;
            //try
            //{
            //    cutter = new Plane(_skeleton.PointAtNormalizedLength(t1), _mainAxis);
            //    cutter.Transform(Transform.Translation(_mainAxis * 0.3));
            //    Brep[] Cut_Brep = part3.Trim(cutter, myDoc.ModelAbsoluteTolerance);
            //    part3 = Cut_Brep[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            //}
            //catch
            //{
            //    cutter = new Plane(_skeleton.PointAtNormalizedLength(t2), _mainAxis);
            //    cutter.Transform(Transform.Translation(_mainAxis * 0.3));
            //    Brep[] Cut_Brep = part3.Trim(cutter, myDoc.ModelAbsoluteTolerance);
            //    part3 = Cut_Brep[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            //}
            //part3.Transform(Transform.Translation(_mainAxis * clearance2));
            //Cut part123 with constraining structure
            Brep part1 = b1;
            part1.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == part1.SolidOrientation)
                part1.Flip();

            //constrainingStructureSpaceTaken.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            //if (BrepSolidOrientation.Inward == constrainingStructureSpaceTaken.SolidOrientation)
            //    constrainingStructureSpaceTaken.Flip();
            //try
            //{
            //    part1 = Brep.CreateBooleanDifference(part1, constrainingStructureSpaceTaken, myDoc.ModelAbsoluteTolerance)[0];
            //}
            //catch { }
            //try
            //{
            //    part2 = Brep.CreateBooleanDifference(part2, constrainingStructureSpaceTaken, myDoc.ModelAbsoluteTolerance)[0];
            //}
            //catch { }
            //try
            //{
            //    part3 = Brep.CreateBooleanDifference(part3, constrainingStructureSpaceTaken, myDoc.ModelAbsoluteTolerance)[0];
            //}
            //catch { }

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
        }
        public override bool LoadKineticUnit()
        {
            if(InputType == 1)
            {
                // helical
                //Movement compression;
                //if (lockT < springStart)
                //{ compression = new Movement(spring, 3, -springLength * distance); }
                //else
                //{ compression = new Movement(spring, 3, springLength * distance); }
                //spring.SetMovement(compression);
                //compression.Activate();
                //locks[0].SetLocked();
                //Loaded = true;
                //return true;
                return true;
            }
            else
            {
                // spiral
                //Movement twist = new Movement(shaftRodShape, 2, 2* Math.PI, Transform.Rotation(2*Math.PI, direction, springSCenter));
                //twist.Activate();
                //locks[0].SetLocked();
                //Loaded = true;
                return true;
            }
           
        }
        public override bool Trigger()
        {
            return true;
            //return locks[0].Activate();//Create point and wait for selection
        }
        public override bool TriggerWithoutInteraction()
        {
            return true;
            //return locks[0].ActivateWithoutInteraction();//Just release locks, no need to wait for selection.
        }
        public override Movement Simulate(double interval = 20, double precision = 0.01)
        {
            Movement m = null;
            //TODO add simulation process
            //m = spring.Activate(interval);
            //m.Activate();
            return m;
        }


        #region Member viarables encapsulations
        
        public Brep Model { get => _model; set => _model = value; }
        public Vector3d Direction { get => _direction; set => _direction = value; }
        public bool AddLock { get => _addLock; set => _addLock = value; }
        public RhinoDoc MyDoc { get => _myDoc; set => _myDoc = value; }
        public int InputType { get => _inputType; set => _inputType = value; }
        public Brep InnerCavity { get => _innerCavity; set => _innerCavity = value; }
        public bool Old_direction { get => old_direction; set => old_direction = value; }
        #endregion
    }
}
