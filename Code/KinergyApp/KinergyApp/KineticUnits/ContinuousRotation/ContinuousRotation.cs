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
        private Entity spring;
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
        public const double shaftRadius = 1.5;
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

        public ContinuousRotation(Brep Model, int selectedAxisIndex, Vector3d Direction, Brep innerCavity, Point3d motionCtrlPt, int speed, int dis, int eneryg, int InputType, Helpers helper)
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

        public void AdjustParameter(int eeMovingDirectionSelection, int speedLevel, int roundLevel, int energyLevel, List<double> gr_list, List<GearTrainScheme> gear_schemes, bool isSpringCW, Entity spring, int motionControlMethod, ref List<Point3d> lockPos, ref bool spiralLockNorm, ref Vector3d spiralLockDir, Point3d eePos = new Point3d())
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
                    List<Gear> gears = _helperFun.genGears(selectedGearTrainParam.parameters, motionControlMethod, 0.4);

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
                        Shaft newLastShaft = new Shaft(axelStart, axelLen, 1.5, axelDir);
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
                        Point3d axelStart = ee1Model.GetBoundingBox(true).Center;
                        Point3d axelEnd = ee2Model.GetBoundingBox(true).Center;
                        Vector3d axelDir = axelEnd - axelStart;
                        double axelLen = axelDir.Length;
                        axelDir.Unitize();
                        Shaft newLastShaft = new Shaft(axelStart, axelLen, 1.5, axelDir);
                        newLastShaft.SetName("MiddleShellBreakerShaft");
                        //TODO Add spacer along line within model ? Not adding for now to prevent bug
                        axel_spacer_entities.Add(newLastShaft);
                    }
                    else
                    {
                        throw new Exception("Unexpected situation! Invalid end effector state");
                    }
                    AddGears(gears, axel_spacer_entities, selectedGearTrainParam);
                }

                #endregion

                #region re-generate the spring

                if (isEnergyChange || isRoundChange || isDirChange || isSpeedChange)
                {
                    Spiral spiralSpring = (Spiral)spring;
                    spiralSpring.AdjustParam(selectedGearTrainParam.parameters, this.Model, eeMovingDirectionSelection, energyLevel, roundLevel, isSpringCW, ref lockPos, ref spiralLockNorm, ref spiralLockDir);
                    AddSprings(spiralSpring);
                }
                //else if ((isEnergyChange || isRoundChange || isDirChange) && isSpeedChange)
                //{
                //    Spiral spiralSpring = _helperFun.genSprings(selectedGearTrainParam.parameters, this.Model, skeleton, mainAxis, motionControlMethod, roundLevel, energyLevel, eeMovingDirectionSelection, out lockPos, out spiralLockNorm, out spiralLockDir);
                //}

                #endregion
            }
        }
        public void AddSprings(Entity springControl)
        {
            entityList.Remove(spring);
            spring = springControl;
            entityList.Add(spring);
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
        //public void ConstructLocks(Transform translationBack, Transform rotationBack, Transform poseRotationBack)
        //{
        //    #region old code
        //    //if(InputType == 1)
        //    //{
        //    //    Vector3d backDir = new Vector3d(-1, 0, 0);


        //    //    List<Brep> bs = new List<Brep>();
        //    //    bs.Add(innerShell);
        //    //    List<Point3d> projPts = new List<Point3d>();
        //    //    projPts.Add(guideEnd);
        //    //    Vector3d projDir = new Vector3d(1, 0, 0);
        //    //    Point3d[] projBrepPts;
        //    //    projBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(bs, projPts, projDir, MyDoc.ModelAbsoluteTolerance);

        //    //    Point3d lockBtnPt = new Point3d();
        //    //    if (guideEnd.X >= Math.Min(projBrepPts[0].X, projBrepPts[1].X) && guideEnd.X <= Math.Max(projBrepPts[0].X, projBrepPts[1].X))
        //    //    {
        //    //        // extend the button to the outside of the innershell
        //    //        double extensionDis = guideEnd.X - Math.Min(projBrepPts[0].X, projBrepPts[1].X);
        //    //        lockBtnPt = guideEnd + backDir * (extensionDis + 2);

        //    //    }
        //    //    else if (guideEnd.X < Math.Min(projBrepPts[0].X, projBrepPts[1].X))
        //    //    {
        //    //        // do nothing
        //    //        lockBtnPt = guideEnd + backDir * 2;
        //    //    }

        //    //    Point3d[] hookGuideRailPts = new Point3d[2];
        //    //    hookGuideRailPts[0] = guideEnd;
        //    //    hookGuideRailPts[1] = lockBtnPt;
        //    //    Curve hookGuideRail = new Polyline(hookGuideRailPts).ToNurbsCurve();

        //    //    Brep hookGuideBrep = Brep.CreatePipe(hookGuideRail, 0.8, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

        //    //    Point3d[] hookGuideTipPts = new Point3d[2];
        //    //    hookGuideTipPts[0] = lockBtnPt;
        //    //    hookGuideTipPts[1] = lockBtnPt + backDir * 1.6;
        //    //    Curve hookGuideTipRail = new Polyline(hookGuideTipPts).ToNurbsCurve();
        //    //    Brep hookGuideTipBrep = Brep.CreatePipe(hookGuideTipRail, 1.6, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

        //    //    Brep guideBtnPart = Brep.CreateBooleanUnion(new List<Brep> { hookGuideBrep, hookGuideTipBrep }, MyDoc.ModelAbsoluteTolerance)[0];

        //    //    guideBtnPart.Transform(translationBack);
        //    //    guideBtnPart.Transform(rotationBack);
        //    //    guideBtnPart.Transform(poseRotationBack);

        //    //    Shape guideBtnShape = new Shape(guideBtnPart, false, "lock");
        //    //    EntityList.Add(guideBtnShape);

        //    //    // Create the hooks
        //    //    Vector3d yPos = new Vector3d(0, 1, 0);
        //    //    Point3d hookStPt = barSt - yPos * 1.2;
        //    //    Point3d hookEndPt = barSt + yPos * 1.2;
        //    //    Point3d[] hookPts = new Point3d[2];
        //    //    hookPts[0] = hookStPt;
        //    //    hookPts[1] = hookEndPt;
        //    //    Curve hookRail = new Polyline(hookPts).ToNurbsCurve();

        //    //    // create sweep function
        //    //    var sweep = new Rhino.Geometry.SweepOneRail();
        //    //    sweep.AngleToleranceRadians = MyDoc.ModelAngleToleranceRadians;
        //    //    sweep.ClosedSweep = false;
        //    //    sweep.SweepTolerance = MyDoc.ModelAbsoluteTolerance;


        //    //    Plane hookPln = new Plane(hookStPt, yPos);

        //    //    Vector3d hk_xp = 0 * hookPln.XAxis;
        //    //    Vector3d hk_xn = (-3) * hookPln.XAxis;
        //    //    Vector3d hk_yp = 4 * hookPln.YAxis;
        //    //    Vector3d hk_yn = 0 * hookPln.YAxis;

        //    //    Point3d[] hookPlatePts = new Point3d[4];
        //    //    hookPlatePts[0] = hookStPt + hk_xp + hk_yn;
        //    //    hookPlatePts[1] = hookStPt + hk_xn + hk_yn;
        //    //    hookPlatePts[2] = hookStPt + hk_xp + hk_yp;
        //    //    hookPlatePts[3] = hookStPt + hk_xp + hk_yn;

        //    //    Curve hookPlateRect = new Polyline(hookPlatePts).ToNurbsCurve();

        //    //    Brep hookPlateBrep = new Brep();

        //    //    hookPlateBrep = sweep.PerformSweep(hookRail, hookPlateRect)[0];
        //    //    hookPlateBrep = hookPlateBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

        //    //    hookPlateBrep.Transform(translationBack);
        //    //    hookPlateBrep.Transform(rotationBack);
        //    //    hookPlateBrep.Transform(poseRotationBack);

        //    //    Shape hookPartShape = new Shape(hookPlateBrep, false, "lock");
        //    //    EntityList.Add(hookPartShape);

        //    //}
        //    //else
        //    //{
        //    //    double ratchetThickness = 1.6;
        //    //    double tolerance = 0.6;
        //    //    double lockRadius = 1.5;

        //    //    Vector3d xPos = new Vector3d(1, 0, 0);
        //    //    Vector3d zPos = new Vector3d(0, 0, 1);
        //    //    Point3d lockPos = keySSidePt + xPos * (lastGearRadius * 0.9) - zPos * (1.732 * lockRadius/2 + tolerance * 3/2);

        //    //    locks = new List<Lock>();

        //    //    //Build the locking structure.

        //    //    Vector3d springDir = (springSSidePt - keySSidePt) / springSSidePt.DistanceTo(keySSidePt);


        //    //    double lockLen = lastGearPos.DistanceTo(keySSidePt) / 2 - tolerance - ratchetThickness;
        //    //    double extensionLen = 10;

        //    //    Lock LockHead = new Lock(keySSidePt + springDir * lockLen, -springDir, lastGearRadius * 0.9,false,false,"", ratchetThickness);

        //    //    Brep lockheadBrep = LockHead.GetModelinWorldCoordinate();

        //    //    lockheadBrep.Transform(translationBack);
        //    //    lockheadBrep.Transform(rotationBack);
        //    //    lockheadBrep.Transform(poseRotationBack);
        //    //    LockHead.SetModel(lockheadBrep);

        //    //    //Vector3d centerLinkDirection = new Vector3d(lockClosestPointOnAxis) - new Vector3d(LockPosition);
        //    //    Point3d lockBarSt = lockPos - springDir * extensionLen;
        //    //    Point3d lockBarEnd = lockPos + springDir * (lockLen - 1);

        //    //    Line lockbarLine = new Line(lockBarSt, lockBarEnd);
        //    //    Curve lockRail = lockbarLine.ToNurbsCurve();
        //    //    Brep lockbarRod = Brep.CreatePipe(lockRail, lockRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];
        //    //    Brep lockbarRodDiff = Brep.CreatePipe(lockRail, lockRadius + tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

        //    //    Sphere lockHandler = new Sphere(lockBarSt, 2 * lockRadius);

        //    //    Point3d lockDisc1PtSt = lockPos - springDir * (1.9 + 1.6 + 1);
        //    //    Point3d lockDisc1PtEnd = lockDisc1PtSt + springDir * 1;

        //    //    Point3d lockDisc2PtSt = lockPos + springDir * (1);
        //    //    Point3d lockDisc2PtEnd = lockDisc2PtSt + springDir * 1;

        //    //    Line lockDisc1Line = new Line(lockDisc1PtSt, lockDisc1PtEnd);
        //    //    Curve lockDisc1Rail = lockDisc1Line.ToNurbsCurve();
        //    //    Brep lockDisc1 = Brep.CreatePipe(lockDisc1Rail, lockRadius + 2 * tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

        //    //    Line lockDisc2Line = new Line(lockDisc2PtSt, lockDisc2PtEnd);
        //    //    Curve lockDisc2Rail = lockDisc2Line.ToNurbsCurve();
        //    //    Brep lockDisc2 = Brep.CreatePipe(lockDisc2Rail, lockRadius + 2 * tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

        //    //    Brep lockPart = Brep.CreateBooleanUnion(new List<Brep> { lockbarRod, lockHandler.ToBrep(), lockDisc1, lockDisc2 }, MyDoc.ModelAbsoluteTolerance)[0];

        //    //    lockPart.Transform(translationBack);
        //    //    lockPart.Transform(rotationBack);
        //    //    lockPart.Transform(poseRotationBack);

        //    //    lockbarRodDiff.Transform(translationBack);
        //    //    lockbarRodDiff.Transform(rotationBack);
        //    //    lockbarRodDiff.Transform(poseRotationBack);


        //    //    if (ModelShape != null)
        //    //    {
        //    //        Brep finalModel = Brep.CreateBooleanDifference(ModelShape.GetModelinWorldCoordinate(), lockbarRodDiff, MyDoc.ModelAbsoluteTolerance)[0];
        //    //        Model = finalModel.DuplicateBrep();
        //    //        ModelShape.SetModel(Model);
        //    //    }


        //    //    locks.Add(LockHead);
        //    //    locks.Add(new Lock(lockPart, false));
        //    //    entityList.Add(LockHead);
        //    //    entityList.Add(locks[1]);
        //    //    LockHead.RegisterOtherPart(locks[1]);
        //    //    _ = new Fixation(shaftRodShape, LockHead);
        //    //}
        //    #endregion
        //}


        public void ConstructLocks(List<Point3d> lockPos, bool spiralLockNorm, Vector3d spiralLockDir, GearTrainParam gtp, int motionControlMethod)
        {
            if (motionControlMethod == 1)
            {
                // add the lock for the helical spring

            }
            else
            {
                // add the lock for the spiral spring

                #region ask the user to select the lock position
                LockSelectionForSpiralSpring lockSelection = new LockSelectionForSpiralSpring(myDoc, blueAttribute, lockPos[0], lockPos[1]);
                lockDisToAxis = gtp.parameters.ElementAt(0).radius + gtp.parameters.ElementAt(1).radius;
                spiralLockCen = (lockPos.ElementAt(0) + lockPos.ElementAt(1)) / 2;
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
                double ratchetRadius = lockDisToAxis * 0.35;

                if (spiralLockNorm)
                    LockHead = new Lock(spiralLockCen, spiralLockDir, ratchetRadius, true);
                else
                    LockHead = new Lock(spiralLockCen, spiralLockDir, ratchetRadius, false);

                LockHead.SetName("lockHead");

                Vector3d centerLinkDirection = new Vector3d(spiralLockCen) - new Vector3d(lockSelection.lockCtrlPointSelected);
                double centerLinkLen = centerLinkDirection.Length;
                centerLinkDirection.Unitize();


                #region add lock parts to the entitylist

                cutBarrel = null;
                addBarrel = null;
                Lock lockBase = new Lock(spiralLockDir, spiralLockCen, lockSelection.lockCtrlPointSelected, ratchetRadius, false, myDoc, ref cutBarrel, ref addBarrel, "lockbase");
                lockBase.SetName("lockBase");

                Entity tempAddBarrel = new Entity(addBarrel, false, "lockBarrel");
                entityList.Add(tempAddBarrel);

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

        public void ConstructSpring(Point3d startPoint, double xEnd, double outDiameter, double totalThickness, double xSpaceEnd, int outputAxle,
            Transform translateBack, Transform rotationBack, Transform postRotationBack)
        {
            #region old code
            //Point3d gearTrainStPos = startPoint + new Vector3d(1, 0, 0) * springWidthThreadshold;
            //double xStart = gearTrainStPos.X;
            //double xLength = xEnd - xStart;
            //double gap = 0.6;
            //int pinionTeethNum = 10; //fixed
            //double minRatio = 1.1; //to ensure that the gear teeth number is integral
            //double module0 = 0.75;
            //double addendum = module0;
            //double pitch = Math.PI * module0;
            ////gear facewidth = min(1.6, pitch*3)  
            //double thickness = Math.Min(2, 3 * pitch);
            //double backlash = 0.6;
            //double backlashGap = 1.374;

            //double pairGearRatio = minRatio;
            //double module = module0;
            //double pressureAngle = 20;
            //double tolerance = 0.4;
            //double discThickness = 1;
            //double shaftRadius = 1.5;
            //double discRadius = 2.5;

            //if (InputType == 1)
            //{
            //    springWidthThreadshold = 15;
            //}
            //else
            //{
            //    springWidthThreadshold = 0;
            //}

            //#region Step 2: construct the spring

            //if (this.InputType == 1)
            //{
            //    #region construct a helical spring 
            //    Vector3d backwardDir = new Vector3d(-1, 0, 0);
            //    Point3d rackTip = lastGearPos + backwardDir * (teethEqualInt * grNew * currModule / 2 + backlash * backlashGap);

            //    // get the spring skeleton
            //    Point3d springPos = rackTip + backwardDir * (2 * currModule + 3 + 1 + springWidthThreadshold / 2);     // 3 is the rack thickness, 1 is the gap
            //    List<Brep> bs = new List<Brep>();
            //    bs.Add(innerShell);
            //    List<Point3d> projPts = new List<Point3d>();
            //    projPts.Add(springPos);
            //    Vector3d projDir = new Vector3d(0, 0, 1);
            //    Point3d[] projBrepPts;
            //    projBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(bs, projPts, projDir, MyDoc.ModelAbsoluteTolerance);

            //    double totalLen = 0;
            //    Point3d SpringSt, SpringEnd;

            //    SpringSt = springPos - projDir * outDiameter / 2.5;
            //    SpringEnd = springPos + projDir * outDiameter / 2;
            //    totalLen = SpringSt.DistanceTo(SpringEnd);

            //    Line skeletonLine = new Line(SpringSt, SpringEnd);
            //    Curve skeletonH = skeletonLine.ToNurbsCurve();

            //    double s_len = SpringSt.DistanceTo(SpringEnd);
            //    double min_wire_diamter = 2.8;
            //    double min_coil_num = 3;
            //    double maxDisp = Math.Max(s_len - min_wire_diamter * min_coil_num, min_coil_num * 0.6);
            //    double displacement = 0.9 * maxDisp / s_len;

            //    springH = new Helix(SpringSt, SpringEnd, springWidthThreadshold, 2.8, 0, displacement, Energy);

            //    Brep brep_s_origi = springH.GetModelinWorldCoordinate();
            //    brep_s_origi.Transform(translateBack);
            //    brep_s_origi.Transform(rotationBack);
            //    brep_s_origi.Transform(postRotationBack);
            //    springH.SetModel(brep_s_origi);

            //    EntityList.Add(springH);
            //    #endregion

            //    #region test by LH
            //    //MyDoc.Objects.AddBrep(springH.GetModelinWorldCoordinate());
            //    //MyDoc.Views.Redraw();
            //    #endregion

            //    #region construct the rack

            //    Point3d rackStPt = SpringEnd + projDir * 1 + (-1) * backwardDir * ((springWidthThreadshold + springH.WireRadius) / 2 + 1);
            //    Point3d rackEndPt = SpringSt + projDir * displacement * s_len + (-1) * backwardDir * ((springWidthThreadshold + springH.WireRadius) / 2 + 1);
            //    Rack rack = new Rack(rackStPt, rackEndPt, new Point3d(1, 0, 0), currModule);

            //    Brep brep_r_origi = rack.GetModelinWorldCoordinate();
            //    brep_r_origi.Transform(translateBack);
            //    brep_r_origi.Transform(rotationBack);
            //    brep_r_origi.Transform(postRotationBack);
            //    rack.SetModel(brep_r_origi);

            //    EntityList.Add(rack);

            //    #endregion

            //    #region test by LH
            //    //MyDoc.Objects.AddBrep(rack.GetModelinWorldCoordinate());
            //    //MyDoc.Views.Redraw();
            //    #endregion


            //    // create sweep function
            //    var sweep = new Rhino.Geometry.SweepOneRail();
            //    sweep.AngleToleranceRadians = MyDoc.ModelAngleToleranceRadians;
            //    sweep.ClosedSweep = false;
            //    sweep.SweepTolerance = MyDoc.ModelAbsoluteTolerance;


            //    #region construct connectors and control button

            //    #region construct the upper plate

            //    Point3d upperPlateSt = SpringEnd;
            //    Point3d upperPlateEnd = SpringEnd + projDir * 2;
            //    Plane upperPlatePln = new Plane(upperPlateSt, projDir);

            //    Vector3d up_xp = ((springWidthThreadshold + springH.WireRadius) / 2 + 1.2) * upperPlatePln.XAxis;
            //    Vector3d up_xn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2 + 1.2) * upperPlatePln.XAxis;
            //    Vector3d up_yp = ((springWidthThreadshold + springH.WireRadius) / 2 + 1) * upperPlatePln.YAxis;
            //    Vector3d up_yn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2 + 1) * upperPlatePln.YAxis;

            //    Point3d[] upperPlatePts = new Point3d[5];
            //    upperPlatePts[0] = upperPlateSt + up_xp + up_yp;
            //    upperPlatePts[1] = upperPlateSt + up_xn + up_yp;
            //    upperPlatePts[2] = upperPlateSt + up_xn + up_yn;
            //    upperPlatePts[3] = upperPlateSt + up_xp + up_yn;
            //    upperPlatePts[4] = upperPlateSt + up_xp + up_yp;
            //    Curve upperPlateRect = new Polyline(upperPlatePts).ToNurbsCurve();


            //    Point3d[] upperRailPts = new Point3d[2];
            //    upperRailPts[0] = upperPlateSt;
            //    upperRailPts[1] = upperPlateEnd;
            //    Curve rail1 = new Polyline(upperRailPts).ToNurbsCurve();

            //    Brep upperPlateBrep = new Brep();

            //    upperPlateBrep = sweep.PerformSweep(rail1, upperPlateRect)[0];
            //    upperPlateBrep = upperPlateBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

            //    #endregion

            //    #region constrcut the base plate

            //    Point3d bottomPlateSt = projBrepPts[0].Z > projBrepPts[1].Z ? projBrepPts[1] : projBrepPts[0];
            //    Point3d bottomPlateEnd = SpringSt;
            //    Plane bottomPlatePln = new Plane(bottomPlateSt, projDir);

            //    Vector3d b_xp = ((springWidthThreadshold + springH.WireRadius) / 2 + 1.2) * bottomPlatePln.XAxis;
            //    Vector3d b_xn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2 + 1.2) * bottomPlatePln.XAxis;
            //    Vector3d b_yp = ((springWidthThreadshold + springH.WireRadius) / 2 + 1) * bottomPlatePln.YAxis;
            //    Vector3d b_yn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2 + 1) * bottomPlatePln.YAxis;

            //    Point3d[] bottomPlatePts = new Point3d[5];
            //    bottomPlatePts[0] = bottomPlateSt + b_xp + b_yp;
            //    bottomPlatePts[1] = bottomPlateSt + b_xn + b_yp;
            //    bottomPlatePts[2] = bottomPlateSt + b_xn + b_yn;
            //    bottomPlatePts[3] = bottomPlateSt + b_xp + b_yn;
            //    bottomPlatePts[4] = bottomPlateSt + b_xp + b_yp;
            //    Curve bottomPlateRect = new Polyline(bottomPlatePts).ToNurbsCurve();


            //    Point3d[] bottomRailPts = new Point3d[2];
            //    bottomRailPts[0] = bottomPlateSt;
            //    bottomRailPts[1] = bottomPlateEnd;
            //    Curve rail2 = new Polyline(bottomRailPts).ToNurbsCurve();

            //    Brep bottomPlateBrep = new Brep();

            //    bottomPlateBrep = sweep.PerformSweep(rail2, bottomPlateRect)[0];
            //    bottomPlateBrep = bottomPlateBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

            //    #endregion

            //    #region constrcut the button

            //    Point3d btnSt = upperPlateEnd;
            //    Point3d btnEnd = projBrepPts[0].Z > projBrepPts[1].Z ? projBrepPts[0] + (1.6 + displacement * s_len) * projDir : projBrepPts[1] + (1.6 + displacement * s_len) * projDir;

            //    Point3d[] btnRailPts = new Point3d[2];
            //    btnRailPts[0] = btnSt;
            //    btnRailPts[1] = btnEnd;
            //    Curve rail3 = new Polyline(btnRailPts).ToNurbsCurve();

            //    Brep btnHandlerBrep = new Brep();
            //    Brep btnHandlerDiffBrep = new Brep();
            //    Brep btnTipBrep = new Brep();

            //    btnHandlerBrep = Brep.CreatePipe(rail3, 2.5, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];
            //    btnHandlerDiffBrep = Brep.CreatePipe(rail3, 3.5, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];


            //    Point3d[] btnTipRailPts = new Point3d[2];
            //    btnTipRailPts[0] = btnEnd;
            //    btnTipRailPts[1] = btnEnd + projDir * 4;
            //    Curve rail4 = new Polyline(btnTipRailPts).ToNurbsCurve();
            //    btnTipBrep = Brep.CreatePipe(rail4, 10, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

            //    var finalModelBreps = Brep.CreateBooleanDifference(Model, btnHandlerDiffBrep, MyDoc.ModelAbsoluteTolerance);
            //    if (finalModelBreps == null)
            //    {
            //        btnHandlerDiffBrep.Flip();
            //        finalModelBreps = Brep.CreateBooleanDifference(Model, btnHandlerDiffBrep, MyDoc.ModelAbsoluteTolerance);
            //    }
            //    Brep finalModelBrep = finalModelBreps[0];
            //    Model = finalModelBrep.DuplicateBrep();

            //    #endregion

            //    #region test by LH

            //    //MyDoc.Objects.AddBrep(upperPlateBrep);
            //    //MyDoc.Views.Redraw();
            //    //MyDoc.Objects.AddBrep(bottomPlateBrep);
            //    //MyDoc.Views.Redraw();
            //    //MyDoc.Objects.AddBrep(btnHandlerBrep);
            //    //MyDoc.Views.Redraw();
            //    //MyDoc.Objects.AddBrep(btnTipBrep);
            //    //MyDoc.Views.Redraw();

            //    #endregion

            //    upperPlateBrep.Transform(translateBack);
            //    upperPlateBrep.Transform(rotationBack);
            //    upperPlateBrep.Transform(postRotationBack);
            //    Shape upperPlateShape = new Shape(upperPlateBrep, false, "connector");

            //    bottomPlateBrep.Transform(translateBack);
            //    bottomPlateBrep.Transform(rotationBack);
            //    bottomPlateBrep.Transform(postRotationBack);
            //    Shape bottomPlateShape = new Shape(bottomPlateBrep, false, "connector");

            //    btnHandlerBrep.Transform(translateBack);
            //    btnHandlerBrep.Transform(rotationBack);
            //    btnHandlerBrep.Transform(postRotationBack);
            //    Shape btnHandlerShape = new Shape(btnHandlerBrep, false, "connector");

            //    btnTipBrep.Transform(translateBack);
            //    btnTipBrep.Transform(rotationBack);
            //    btnTipBrep.Transform(postRotationBack);
            //    Shape btnTipShape = new Shape(btnTipBrep, false, "connector");

            //    EntityList.Add(upperPlateShape);
            //    EntityList.Add(bottomPlateShape);
            //    EntityList.Add(btnHandlerShape);
            //    EntityList.Add(btnTipShape);

            //    #endregion

            //    #region construct the guide for the spring

            //    #region constrcut the hookbar and the hookguide
            //    Point3d barEnd = upperPlateSt + backwardDir * ((springWidthThreadshold + springH.WireRadius) / 2 + 1.5) + projDir * 2;
            //    barSt = barEnd - projDir * (totalLen - displacement * s_len - 4);
            //    Point3d guideSt = barSt + projDir * (4 + 0.2);        // the hook height is 6mm
            //    guideEnd = guideSt + backwardDir * (4 + 1.4);

            //    Plane hookBarPln = new Plane(barSt, projDir);

            //    Vector3d hb_xp = 0.6 * hookBarPln.XAxis;
            //    Vector3d hb_xn = -0.6 * hookBarPln.XAxis;
            //    Vector3d hb_yp = 1.2 * hookBarPln.YAxis;
            //    Vector3d hb_yn = -1.2 * hookBarPln.YAxis;

            //    Point3d[] hookBarPts = new Point3d[5];
            //    hookBarPts[0] = barSt + hb_xp + hb_yp;
            //    hookBarPts[1] = barSt + hb_xn + hb_yp;
            //    hookBarPts[2] = barSt + hb_xn + hb_yn;
            //    hookBarPts[3] = barSt + hb_xp + hb_yn;
            //    hookBarPts[4] = barSt + hb_xp + hb_yp;
            //    Curve hookBarRect = new Polyline(hookBarPts).ToNurbsCurve();


            //    Point3d[] hookBarRailPts = new Point3d[2];
            //    hookBarRailPts[0] = barSt;
            //    hookBarRailPts[1] = barEnd;
            //    Curve rail5 = new Polyline(hookBarRailPts).ToNurbsCurve();

            //    Brep hookbarBrep = new Brep();

            //    hookbarBrep = sweep.PerformSweep(rail5, hookBarRect)[0];
            //    hookbarBrep = hookbarBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);


            //    Point3d[] hookGuideRailPts = new Point3d[2];
            //    hookGuideRailPts[0] = guideSt;
            //    hookGuideRailPts[1] = guideEnd;
            //    Curve rail6 = new Polyline(hookGuideRailPts).ToNurbsCurve();

            //    Brep hookGuideBrep = new Brep();

            //    hookGuideBrep = Brep.CreatePipe(rail6, 0.8, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

            //    #endregion

            //    #region construct the guide
            //    Point3d guideWallPos = SpringSt + backwardDir * ((springWidthThreadshold + springH.WireRadius) / 2 + 1.5 + 4);

            //    List<Brep> bs_wall = new List<Brep>();
            //    bs_wall.Add(innerShell);
            //    List<Point3d> projWallPts = new List<Point3d>();
            //    projWallPts.Add(guideWallPos);
            //    Vector3d projWallDir = new Vector3d(0, 0, 1);
            //    Point3d[] projWallBrepPts;
            //    projWallBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(bs_wall, projWallPts, projWallDir, MyDoc.ModelAbsoluteTolerance);

            //    Point3d guideWallSt, guideWallEnd;

            //    if (projWallBrepPts[0].Z > projWallBrepPts[1].Z)
            //    {
            //        guideWallEnd = projWallBrepPts[0];
            //        guideWallSt = projWallBrepPts[1];
            //    }
            //    else
            //    {
            //        guideWallEnd = projWallBrepPts[1];
            //        guideWallSt = projWallBrepPts[0];
            //    }

            //    Plane guideWallPln = new Plane(guideWallSt, projWallDir);

            //    Vector3d gw_xp = 0.8 * guideWallPln.XAxis;
            //    Vector3d gw_xn = -0.8 * guideWallPln.XAxis;
            //    Vector3d gw_yp = ((springWidthThreadshold + springH.WireRadius) / 2) * guideWallPln.YAxis;
            //    Vector3d gw_yn = (-1) * ((springWidthThreadshold + springH.WireRadius) / 2) * guideWallPln.YAxis;

            //    Point3d[] guideWallPts = new Point3d[5];
            //    guideWallPts[0] = guideWallSt + gw_xp + gw_yp;
            //    guideWallPts[1] = guideWallSt + gw_xn + gw_yp;
            //    guideWallPts[2] = guideWallSt + gw_xn + gw_yn;
            //    guideWallPts[3] = guideWallSt + gw_xp + gw_yn;
            //    guideWallPts[4] = guideWallSt + gw_xp + gw_yp;
            //    Curve guideWallRect = new Polyline(guideWallPts).ToNurbsCurve();


            //    Point3d[] guideWallRailPts = new Point3d[2];
            //    guideWallRailPts[0] = guideWallSt;
            //    guideWallRailPts[1] = guideWallEnd;
            //    Curve rail7 = new Polyline(guideWallRailPts).ToNurbsCurve();

            //    Brep guideWallBrep = new Brep();

            //    guideWallBrep = sweep.PerformSweep(rail7, guideWallRect)[0];
            //    guideWallBrep = guideWallBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

            //    // Open the linear slot 

            //    Point3d slotSt = guideWallPos;
            //    Point3d slotEnd = guideWallPos + projDir * (totalLen + 2);

            //    Plane wallSlotPln = new Plane(slotSt, projDir);

            //    Vector3d ws_xp = 2 * wallSlotPln.XAxis;
            //    Vector3d ws_xn = -2 * wallSlotPln.XAxis;
            //    Vector3d ws_yp = 1.4 * wallSlotPln.YAxis;
            //    Vector3d ws_yn = -1.4 * wallSlotPln.YAxis;

            //    Point3d[] wallSlotPts = new Point3d[5];
            //    wallSlotPts[0] = slotSt + ws_xp + ws_yp;
            //    wallSlotPts[1] = slotSt + ws_xn + ws_yp;
            //    wallSlotPts[2] = slotSt + ws_xn + ws_yn;
            //    wallSlotPts[3] = slotSt + ws_xp + ws_yn;
            //    wallSlotPts[4] = slotSt + ws_xp + ws_yp;
            //    Curve wallSlotRect = new Polyline(wallSlotPts).ToNurbsCurve();


            //    Point3d[] wallSlotRailPts = new Point3d[2];
            //    wallSlotRailPts[0] = slotSt;
            //    wallSlotRailPts[1] = slotEnd;
            //    Curve rail8 = new Polyline(wallSlotRailPts).ToNurbsCurve();

            //    Brep wallSlotBrep = new Brep();

            //    wallSlotBrep = sweep.PerformSweep(rail8, wallSlotRect)[0];
            //    wallSlotBrep = wallSlotBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

            //    var wallGuideFinalBreps = Brep.CreateBooleanDifference(guideWallBrep, wallSlotBrep, MyDoc.ModelAbsoluteTolerance);
            //    if (wallGuideFinalBreps == null)
            //    {
            //        wallSlotBrep.Flip();
            //        wallGuideFinalBreps = Brep.CreateBooleanDifference(guideWallBrep, wallSlotBrep, MyDoc.ModelAbsoluteTolerance);
            //    }
            //    Brep wallGuideFinalBrep = wallGuideFinalBreps[0];

            //    #endregion

            //    #region test by LH

            //    //MyDoc.Objects.AddBrep(hookbarBrep);
            //    //MyDoc.Views.Redraw();
            //    //MyDoc.Objects.AddBrep(hookGuideBrep);
            //    //MyDoc.Views.Redraw();
            //    //MyDoc.Objects.AddBrep(wallGuideFinalBrep);
            //    //MyDoc.Views.Redraw();
            //    //MyDoc.Objects.AddBrep(Model);
            //    //MyDoc.Views.Redraw();

            //    #endregion

            //    hookbarBrep.Transform(translateBack);
            //    hookbarBrep.Transform(rotationBack);
            //    hookbarBrep.Transform(postRotationBack);
            //    Shape hookbarShape = new Shape(hookbarBrep, false, "connector");

            //    hookGuideBrep.Transform(translateBack);
            //    hookGuideBrep.Transform(rotationBack);
            //    hookGuideBrep.Transform(postRotationBack);
            //    Shape hookGuideShape = new Shape(hookGuideBrep, false, "connector");

            //    wallGuideFinalBrep.Transform(translateBack);
            //    wallGuideFinalBrep.Transform(rotationBack);
            //    wallGuideFinalBrep.Transform(postRotationBack);
            //    Shape wallGuideFinalShape = new Shape(wallGuideFinalBrep, false, "connector");

            //    ModelShape = new Shape(Model, false, "main body");

            //    EntityList.Add(hookbarShape);
            //    EntityList.Add(hookGuideShape);
            //    EntityList.Add(wallGuideFinalShape);
            //    EntityList.Add(ModelShape);

            //    #endregion

            //}
            //else
            //{
            //    // create sweep function
            //    var sweep = new Rhino.Geometry.SweepOneRail();
            //    sweep.AngleToleranceRadians = MyDoc.ModelAngleToleranceRadians;
            //    sweep.ClosedSweep = false;
            //    sweep.SweepTolerance = MyDoc.ModelAbsoluteTolerance;

            //    #region construct a spiral spring 

            //    SpringS = null;
            //    //SpringS = new Spiral(new Vector3d(springSSidePt - springSCenter), springSCenter, lastGearRadius * 1.2, Math.PI * 2, (int)(Energy * 10 - 1));
            //    Brep spring_s_origi = SpringS.GetModelinWorldCoordinate();

            //    spring_s_origi.Transform(translateBack);
            //    spring_s_origi.Transform(rotationBack);
            //    spring_s_origi.Transform(postRotationBack);

            //    SpringS.SetModel(spring_s_origi);

            //    EntityList.Add(SpringS);

            //    _ = new Fixation(shaftRodShape, SpringS);



            //    #endregion

            //    #region construct the rod that connects the spring outer end to the main body

            //    Point3d outerEndpt = SpringS.BaseCurve.PointAtNormalizedLength(1);
            //    List<Brep> mainbody = new List<Brep>();
            //    mainbody.Add(innerShell);
            //    List<Point3d> projMBPts = new List<Point3d>();
            //    projMBPts.Add(outerEndpt);
            //    Vector3d projDir = new Vector3d(0, 1, 0);
            //    Point3d[] projBrepPts;
            //    projBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(mainbody, projMBPts, projDir, MyDoc.ModelAbsoluteTolerance);

            //    Point3d connectorStPt = springSSidePt.DistanceTo(projBrepPts[0]) > springSSidePt.DistanceTo(projBrepPts[1]) ? projBrepPts[1] : projBrepPts[0];

            //    Plane fixerPln = new Plane(connectorStPt, new Vector3d(0, 1, 0));

            //    Vector3d fix_xp = 0.8 * fixerPln.XAxis;
            //    Vector3d fix_xn = -0.8 * fixerPln.XAxis;
            //    Vector3d fix_yp = 1 * fixerPln.YAxis;
            //    Vector3d fix_yn = -1 * fixerPln.YAxis;

            //    Point3d[] fixerPts = new Point3d[5];
            //    fixerPts[0] = connectorStPt + fix_xp + fix_yp;
            //    fixerPts[1] = connectorStPt + fix_xn + fix_yp;
            //    fixerPts[2] = connectorStPt + fix_xn + fix_yn;
            //    fixerPts[3] = connectorStPt + fix_xp + fix_yn;
            //    fixerPts[4] = connectorStPt + fix_xp + fix_yp;
            //    Curve fixerRect = new Polyline(fixerPts).ToNurbsCurve();

            //    Line rodRailLine = new Line(connectorStPt, outerEndpt);
            //    Curve rodRail = rodRailLine.ToNurbsCurve();

            //    Brep rod = new Brep();

            //    rod = sweep.PerformSweep(rodRail, fixerRect)[0];
            //    rod = rod.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

            //    rod.Transform(translateBack);
            //    rod.Transform(rotationBack);
            //    rod.Transform(postRotationBack);

            //    Shape fixer = new Shape(rod, false, "connector");
            //    EntityList.Add(fixer);

            //    #endregion


            //    #region Cut the bottom part

            //    Plane cutoutPln = new Plane(InnerCavity.GetBoundingBox(true).Center, new Vector3d(0, 0, 1));

            //    Vector3d co_xp = (InnerCavity.GetBoundingBox(true).Max.X - InnerCavity.GetBoundingBox(true).Min.X) / 2 * cutoutPln.XAxis;
            //    Vector3d co_xn = (-1) * (InnerCavity.GetBoundingBox(true).Max.X - InnerCavity.GetBoundingBox(true).Min.X) / 2 * cutoutPln.XAxis;
            //    Vector3d co_yp = (totalThickness * 0.35) / 2 * cutoutPln.YAxis;
            //    Vector3d co_yn = (-1) * (totalThickness * 0.35) / 2 * cutoutPln.YAxis;

            //    Point3d[] cutoutPts = new Point3d[5];
            //    cutoutPts[0] = InnerCavity.GetBoundingBox(true).Center + co_xp + co_yp;
            //    cutoutPts[1] = InnerCavity.GetBoundingBox(true).Center + co_xn + co_yp;
            //    cutoutPts[2] = InnerCavity.GetBoundingBox(true).Center + co_xn + co_yn;
            //    cutoutPts[3] = InnerCavity.GetBoundingBox(true).Center + co_xp + co_yn;
            //    cutoutPts[4] = InnerCavity.GetBoundingBox(true).Center + co_xp + co_yp;
            //    Curve cutoutRect = new Polyline(cutoutPts).ToNurbsCurve();


            //    Point3d[] cutoutRailPts = new Point3d[2];
            //    Vector3d dirCO = new Vector3d(0, 0, -1);
            //    cutoutRailPts[0] = InnerCavity.GetBoundingBox(true).Center + dirCO * 100;
            //    cutoutRailPts[1] = InnerCavity.GetBoundingBox(true).Center + dirCO * (InnerCavity.GetBoundingBox(true).Max.Z - InnerCavity.GetBoundingBox(true).Min.Z)/2;
            //    Curve coRail = new Polyline(cutoutRailPts).ToNurbsCurve();

            //    Brep coutoutBrep = new Brep();

            //    coutoutBrep = sweep.PerformSweep(coRail, cutoutRect)[0];
            //    coutoutBrep = coutoutBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);
            //    coutoutBrep.Flip();

            //    // Drill the holes for the output gear shaft (connecting to the two wheels)
            //    shDiffRodSpiral.Transform(translateBack);
            //    shDiffRodSpiral.Transform(rotationBack);
            //    shDiffRodSpiral.Transform(postRotationBack);

            //    var outputAxleModelBreps = Brep.CreateBooleanDifference(Model, shDiffRodSpiral, MyDoc.ModelAbsoluteTolerance);
            //    if (outputAxleModelBreps == null)
            //    {
            //        shDiffRodSpiral.Flip();
            //        outputAxleModelBreps = Brep.CreateBooleanDifference(Model, shDiffRodSpiral, MyDoc.ModelAbsoluteTolerance);
            //    }
            //    Brep outputAxleModelBrep = outputAxleModelBreps[0];
            //    Model = outputAxleModelBrep.DuplicateBrep();


            //    // Drill the hole for the key and the revolute joint
            //    shaftCutoutBrep.Transform(translateBack);
            //    shaftCutoutBrep.Transform(rotationBack);
            //    shaftCutoutBrep.Transform(postRotationBack);

            //    var drillers = Brep.CreateBooleanDifference(Model, shaftCutoutBrep, MyDoc.ModelAbsoluteTolerance);
            //    if (drillers == null)
            //    {
            //        shaftCutoutBrep.Flip();
            //        drillers = Brep.CreateBooleanDifference(Model, shaftCutoutBrep, MyDoc.ModelAbsoluteTolerance);
            //    }
            //    Model = drillers[0].DuplicateBrep();


            //    //modelDiffLaterUsedBrep.Transform(translateBack);
            //    //modelDiffLaterUsedBrep.Transform(rotationBack);
            //    //modelDiffLaterUsedBrep.Transform(postRotationBack);

            //    //var placeBreps = Brep.CreateBooleanDifference(Model, modelDiffLaterUsedBrep, MyDoc.ModelAbsoluteTolerance);
            //    //if (placeBreps == null)
            //    //{
            //    //    modelDiffLaterUsedBrep.Flip();
            //    //    placeBreps = Brep.CreateBooleanDifference(Model, modelDiffLaterUsedBrep, MyDoc.ModelAbsoluteTolerance);
            //    //}
            //    //Model = placeBreps[0].DuplicateBrep();

            //    // Cut off the bottom of the main body
            //    coutoutBrep.Transform(translateBack);
            //    coutoutBrep.Transform(rotationBack);
            //    coutoutBrep.Transform(postRotationBack);

            //    Brep[] finalModels = Brep.CreateBooleanDifference(Model, coutoutBrep, MyDoc.ModelAbsoluteTolerance);
            //    if (finalModels == null)
            //    {
            //        coutoutBrep.Flip();
            //        finalModels = Brep.CreateBooleanDifference(Model, coutoutBrep, MyDoc.ModelAbsoluteTolerance);
            //    }
            //    Model = finalModels[0].DuplicateBrep();

            //    ModelShape = new Shape(Model, false, "main body");
            //    EntityList.Add(ModelShape);

            //    #endregion

            //}
            //#endregion
            #endregion
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
        

        /// <summary>
        /// Generate the gear train inside the model
        /// </summary>
        /// <param name="startPoint">the position of thepinion for the input gear</param>
        /// <param name="xEnd">the center position of the output gear</param>
        /// <param name="outDiameter">the height (z) of the model that the gear train is added to</param>
        /// <param name="totalThickness">the thickness (y) of the model</param>
        /// <param name="xSpaceEnd">the length (x) of the model</param>
        /// <param name="outputAxle">the type of the output gear shaft</param>
        /// <returns></returns>
        public void ConstructGearTrain(Point3d startPoint, double xEnd, double outDiameter, double totalThickness, double xSpaceEnd, int outputAxle,
            Transform translateBack, Transform rotationBack, Transform postRotationBack)
        {
            #region old code
            //_startPoint = startPoint;
            //_xEnd = xEnd;
            //_outDiameter = outDiameter;
            //_totalThickness = totalThickness;
            //_xSpaceEnd = xSpaceEnd;
            //_outputAxle = outputAxle;
            //_translateBack = translateBack;
            //_rotationBack = rotationBack;
            //_postRotationBack = postRotationBack;

            //List<Brep> result = new List<Brep>();

            //#region Step 0: Create a shell
            //double shellThickness = 1.6;
            //if(innerShell == null)
            //{
            //    Brep[] innerShells;
            //    Brep[] innerWalls;

            //    processingwin.BringToFront();
            //    processingwin.Show();
            //    Brep[] shells = Brep.CreateOffsetBrep(Model, (-1) * shellThickness, false, true, MyDoc.ModelRelativeTolerance, out innerShells, out innerWalls);

            //    innerShell = shells[0];
            //    processingwin.Hide();

            //    Plane xySplitPln = new Plane(Model.GetBoundingBox(true).Center, new Vector3d(0, 0, 1));

            //    Point3d[] ioPts = new Point3d[2];
            //    Vector3d upZ = new Vector3d(0, 0, 1);
            //    ioPts[0] = innerShell.GetBoundingBox(true).Center - upZ * (innerShell.GetBoundingBox(true).Max.Z - innerShell.GetBoundingBox(true).Min.Z);
            //    ioPts[1] = innerShell.GetBoundingBox(true).Center + upZ * (innerShell.GetBoundingBox(true).Max.Z - innerShell.GetBoundingBox(true).Min.Z);
            //    Curve railio = new Polyline(ioPts).ToNurbsCurve();

            //    Brep link = new Brep();
            //    link = Brep.CreatePipe(railio, 1, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

            //    Brep innerShellDup = innerShell.DuplicateBrep();
            //    var ioBreps = Brep.CreateBooleanUnion(new List<Brep> { innerShellDup, link }, MyDoc.ModelAbsoluteTolerance);
            //    if (ioBreps == null)
            //    {
            //        link.Flip();
            //        ioBreps = Brep.CreateBooleanUnion(new List<Brep> { innerShellDup, link }, MyDoc.ModelAbsoluteTolerance);
            //    }

            //    Brep ioBrep = ioBreps[0];

            //    #region test by LH
            //    //MyDoc.Objects.AddBrep(ioBrep);
            //    //MyDoc.Views.Redraw();
            //    #endregion

            //    var finalShellBreps = Brep.CreateBooleanDifference(Model, ioBrep, MyDoc.ModelAbsoluteTolerance);
            //    if (finalShellBreps == null)
            //    {
            //        ioBrep.Flip();
            //        finalShellBreps = Brep.CreateBooleanDifference(Model, ioBrep, MyDoc.ModelAbsoluteTolerance);
            //    }
            //    Brep finalShellBrep = finalShellBreps[0];

            //    Model = finalShellBrep.DuplicateBrep();

            //    #region test by LH
            //    //MyDoc.Objects.AddBrep(finalShellBrep);
            //    //MyDoc.Views.Redraw();
            //    #endregion
            //}

            //#endregion

            //// As now, *Model* is the shell brep

            //#region Step 1: construct the gear train 

            //if (InputType == 1)
            //{
            //    springWidthThreadshold = 15;
            //}
            //else
            //{
            //    springWidthThreadshold = 0;
            //}

            //Point3d gearTrainStPos = startPoint + new Vector3d(1,0,0) * springWidthThreadshold;

            //double xStart = gearTrainStPos.X;
            //double xLength = xEnd - xStart;
            //double gap = 0.7;   // the distance between components
            //int pinionTeethNum = 13; //intial number is 13
            //double minRatio = 14/13.0; //to ensure that the gear teeth number is integral
            //double module0 = 0.75; //inintial gear module
            //double pitch = Math.PI * module0;
            ////gear facewidth = min(1.6, pitch*3)  
            //double thickness = 3;
            //double backlash = 0.6;
            //double backlashGap = 1.374; // old: 1.374
            //double maxGearDiameter = outDiameter - 2 * gap;
            //double pairGearRatio = minRatio;
            //double module = module0;
            //double pressureAngle = 20; // for 3D printing, a 20-25 degree angle is suggested
            //double tolerance = 0.5;
            //double discThickness = 2;
            //double shaftRadius = Math.Max(1.6, pinionTeethNum * module / 6.1);
            //double discRadius = shaftRadius + 2 * tolerance;

            //if (xLength < 0) return;

            //#region new version by LH

            //#region Step 1: calcualte the possible gear sets

            //List<gearSetDesign> gearSets = new List<gearSetDesign>();
            //int numSet = 1;
            //double gearRadius = pairGearRatio * (module * pinionTeethNum / 2);
            //int gearTeethNum = (int)(pairGearRatio * pinionTeethNum);
            //double totalLenGearSet = 0;
            //double totalThicknessGearSet = 0;

            //do
            //{
            //    pairGearRatio = gearTeethNum * 1.0 / pinionTeethNum;
            //    gearRadius = gearTeethNum * module / 2;
            //    // totalLenGearSet = numSet * (gearRadius + backlash * backlashGap + pinionTeethNum * module / 2);
            //    // totalThicknessGearSet = numSet * (thickness + tolerance) + 2 * gap;

            //    if((gearRadius <= maxGearDiameter/2))
            //    {
            //        // the current gear set is acceptable, increase the teeth number of the gear
            //        int maxNum = (int)Math.Floor(xLength / (gearRadius + backlash * backlashGap + pinionTeethNum * module / 2));

            //        if(maxNum >= 1)
            //        {
            //            if(maxNum * (thickness + tolerance) + 2 * gap <= totalThickness)
            //            {
            //                gearSetDesign possibleGearSet = new gearSetDesign(module, Math.Pow(pairGearRatio, maxNum), pinionTeethNum * module / 2, maxNum, pairGearRatio);
            //                gearSets.Add(possibleGearSet);
            //                gearTeethNum++;
            //            } 
            //        }
            //        else
            //        {
            //            break;
            //        }
            //    }
            //    else
            //    {
            //        module += 0.15;
            //        if(module >3)
            //        {
            //            module -= 0.15;
            //            break;
            //        }
            //        else
            //        {
            //            gearTeethNum = 14;
            //        }
            //    }

            //} while(true);

            //// re-order the list in an ascending order of gear ratio
            //int len = gearSets.Count;
            //for (int i = 0; i < len - 1; i++)
            //{
            //    for (int j = 0; j < len - 1 - i; j++)
            //    {
            //        if (gearSets.ElementAt(j).GearRatio > gearSets.ElementAt(j + 1).GearRatio)
            //        { 
            //            gearSetDesign temp = new gearSetDesign(gearSets.ElementAt(j + 1).Module, gearSets.ElementAt(j + 1).GearRatio, 
            //                gearSets.ElementAt(j + 1).PinionRadius, gearSets.ElementAt(j + 1).NumSet, gearSets.ElementAt(j + 1).GrUnit);

            //            gearSets.ElementAt(j + 1).Module = gearSets.ElementAt(j).Module;
            //            gearSets.ElementAt(j + 1).GearRatio = gearSets.ElementAt(j).GearRatio;
            //            gearSets.ElementAt(j + 1).NumSet = gearSets.ElementAt(j).NumSet;
            //            gearSets.ElementAt(j + 1).PinionRadius = gearSets.ElementAt(j).PinionRadius;
            //            gearSets.ElementAt(j + 1).GrUnit = gearSets.ElementAt(j).GrUnit;

            //            gearSets.ElementAt(j).GearRatio = temp.GearRatio;
            //            gearSets.ElementAt(j).Module = temp.Module;
            //            gearSets.ElementAt(j).NumSet = temp.NumSet;
            //            gearSets.ElementAt(j).PinionRadius = temp.PinionRadius;
            //            gearSets.ElementAt(j).GrUnit = temp.GrUnit;
            //        }
            //    }
            //}

            //#endregion

            //#region Step 2: map the input speed to the proper gear set
            //gearSetDesign targetGearSet = null;

            //if (len > 10)
            //{
            //    // select 10 sets to match the speed input
            //    int fromSource = 0;
            //    int toSource = 9;
            //    int fromTarget = 0;
            //    int toTarget = len - 1;
            //    int sel_idx = (int)(((int)Speed - fromSource + 1) * 1.0 / (toSource - fromSource + 1) * (toTarget - fromTarget + 1) + fromTarget);

            //    targetGearSet = new gearSetDesign(gearSets.ElementAt(sel_idx).Module, gearSets.ElementAt(sel_idx).GearRatio,
            //                                        gearSets.ElementAt(sel_idx).PinionRadius, gearSets.ElementAt(sel_idx).NumSet, gearSets.ElementAt(sel_idx).GrUnit);
            //}
            //else if(len == 10)
            //{
            //    targetGearSet = new gearSetDesign(gearSets.ElementAt((int)Speed).Module, gearSets.ElementAt((int)Speed).GearRatio,
            //                                        gearSets.ElementAt((int)Speed).PinionRadius, gearSets.ElementAt((int)Speed).NumSet, gearSets.ElementAt((int)Speed).GrUnit);
            //}
            //else
            //{
            //    // set the highest speed input the same
            //    if((int)Speed >= len)
            //    {
            //        targetGearSet = new gearSetDesign(gearSets.ElementAt(len-1).Module, gearSets.ElementAt(len - 1).GearRatio,
            //                                        gearSets.ElementAt(len - 1).PinionRadius, gearSets.ElementAt(len - 1).NumSet, gearSets.ElementAt(len - 1).GrUnit);
            //    }
            //    else
            //    {
            //        targetGearSet = new gearSetDesign(gearSets.ElementAt((int)Speed).Module, gearSets.ElementAt((int)Speed).GearRatio,
            //                                        gearSets.ElementAt((int)Speed).PinionRadius, gearSets.ElementAt((int)Speed).NumSet, gearSets.ElementAt((int)Speed).GrUnit);
            //    }
            //}


            //#region old code
            ////int numPairMax = -1; // the max num of two-gear pair can be added

            //////numPairMax = (int)(xLength / (backlash * backlashGap
            //////                + module0 * pinionTeethNum * (minRatio + 1) / 2));


            ////numPairMax = (int)(xLength / (backlash * backlashGap
            ////                + Math.Min(module0 * pinionTeethNum * 5 / 2, maxGearDiameter/2) + module0 * pinionTeethNum / 2));
            ////// find the range of number of two-gear pairs that can be added in this space
            ////int numPairMin = 100;
            ////numPairMax = (int)Math.Min(numPairMax, (totalThickness - 2 * gap) / (gap + thickness));

            ////if(numPairMax >= 1)
            ////{

            ////    for(int num = 1; num <= numPairMax; num++)
            ////    {
            ////        double r = (xLength / num - backlash * backlashGap) * 2 / (module0 * pinionTeethNum) - 1;
            ////        if (r > 5)
            ////        {
            ////            numPairMin = num + 1;
            ////        }
            ////        else
            ////        {
            ////            numPairMin = num;
            ////            break;
            ////        }
            ////        //double p_radius = xLength / num - backlash * backlashGap - maxGearDiameter / 2;
            ////        //if(p_radius > maxGearDiameter / 2)
            ////        //{
            ////        //    numPairMin = num + 1;
            ////        //}
            ////        //else
            ////        //{
            ////        //    break;
            ////        //}
            ////    }
            ////}

            ////if (numPairMax < numPairMin) return;
            ////int numSection = numPairMax - numPairMin + 1;
            ////int currNum = (int)Math.Floor(Speed / (10.0 / numSection)) + numPairMin;

            ////int baseSpeed = (int)Math.Ceiling((currNum - numPairMin) * (10.0 / numSection));
            ////int iterationNum = (int)Speed - baseSpeed + 1;

            ////// start from two identical gears: R = 1

            ////double mgr = (xLength / currNum - backlash * backlashGap);

            ////int teethMax = (int)(mgr / module0);
            ////teethEqualInt = (teethMax + pinionTeethNum) / 2;

            ////double mNew = mgr / teethEqualInt;
            ////grNew = (xLength / currNum - backlash * backlashGap) * 2 / (mNew * teethEqualInt) - 1;

            ////for (int i = 1; i <= iterationNum; ++i)
            ////{
            ////    double newMGR = mgr - (mgr - module0 * pinionTeethNum) * i / iterationNum;

            ////    teethMax = (int)(newMGR / module0);

            ////    teethEqualInt = (teethMax + pinionTeethNum) / 2;

            ////    mNew = newMGR / teethEqualInt;
            ////    grNew = (xLength / currNum - backlash * backlashGap) * 2 / (mNew * teethEqualInt) - 1;
            ////}

            ////currModule = (xLength - currNum * backlashGap * backlash) / (currNum * teethEqualInt * (1 + grNew) / 2);
            ////int gearTeethNum = (int)(teethEqualInt * grNew);

            //#endregion

            //currModule = targetGearSet.Module;
            //grNew = targetGearSet.GrUnit;
            //int currNum = targetGearSet.NumSet;

            //double gearDistance = pinionTeethNum * (1 + grNew) * currModule / 2 + backlash * backlashGap;

            //#endregion

            //// shift all the gears to one side, which is the Y postive side
            //// to find the smallest Y offset to start the gear center positions
            //BoundingBox innerShellBoundingBox = innerShell.GetBoundingBox(true);

            ////double maxshift = (totalThickness - 2 * gap - 2 * shellThickness) - currNum * (2 * thickness + tolerance); 
            //double yOriginal = 0;
            ////if(maxshift > 0)
            ////{
            ////    yOriginal -= maxshift/2;
            ////}

            //yOriginal = innerShellBoundingBox.Min.Y + 2 * gap;

            //// configure all the gear centers
            //List<Point3d> gearCenters = new List<Point3d>();
            //double xPos = xEnd;
            //double yPos = yOriginal;
            //for(int i = 0; i < currNum; i++)
            //{
            //    gearCenters.Add(new Point3d(xPos, yPos, 0));
            //    xPos -= gearDistance;
            //    yPos += tolerance;
            //    gearCenters.Add(new Point3d(xPos, yPos, 0));
            //    yPos += thickness;
            //}

            //Vector3d gearDir = new Vector3d();

            //List<Brep> gears = new List<Brep>();
            //List<Brep> shafts = new List<Brep>();
            //List<Brep> partsForShaftDifference = new List<Brep>();

            //foreach(Point3d pt in gearCenters)
            //{
            //    if(gearCenters.IndexOf(pt) %2 == 0)
            //    {

            //        #region Step 1: create the pinion from the output gear end
            //        gearDir = gearCenters.ElementAt(gearCenters.IndexOf(pt) + 1) - pt;
            //        Gear PinionGear = new Gear(pt, gearDir, direction, pinionTeethNum, currModule, pressureAngle, thickness + tolerance, 0, false);
            //        Brep pinion = PinionGear.Model;

            //        gearEntities.Add(PinionGear);
            //        gears.Add(pinion);


            //        #endregion

            //        #region test by LH
            //        MyDoc.Objects.AddBrep(innerShell);
            //        MyDoc.Views.Redraw();
            //        MyDoc.Objects.AddPoint(pt);
            //        MyDoc.Views.Redraw(); 
            //        MyDoc.Objects.AddBrep(pinion);
            //        MyDoc.Views.Redraw();
            //        #endregion

            //        #region Step 2: create the shaft for the coaxial pinion and big gear

            //        // first, get the projected points of the center point on the innershell
            //        List<Brep> bs = new List<Brep>();
            //        bs.Add(innerShell);
            //        List<Point3d> projPts = new List<Point3d>();
            //        projPts.Add(pt);
            //        Vector3d projDir = new Vector3d(0, 1, 0);
            //        Point3d[] projBrepPts;
            //        projBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(bs, projPts, projDir, MyDoc.ModelAbsoluteTolerance);

            //        if(projBrepPts.Count() == 2)
            //        {
            //            Point3d pt1 = new Point3d();    // the point on the postive side
            //            Point3d pt2 = new Point3d();    // the point on the negative side

            //           // _ = new Fixation(PinionGear, gearEntities.Last());

            //            if(projBrepPts[0].Y > projBrepPts[1].Y)
            //            {
            //                pt1 = projBrepPts[0] + projDir * 0.2;
            //                pt2 = projBrepPts[1] - projDir * 0.2;
            //            }
            //            else
            //            {
            //                pt1 = projBrepPts[1] + projDir * 0.2;
            //                pt2 = projBrepPts[0] - projDir * 0.2;
            //            }

            //            Line shaftRailLine = new Line(pt2, pt1);
            //            Curve shaftRail = shaftRailLine.ToNurbsCurve();
            //            Brep[] shaftRods = Brep.CreatePipe(shaftRail, shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //            Brep shaftRod = shaftRods[0];

            //            if (gearCenters.IndexOf(pt) != 0)
            //            {
            //                // create a brep to different the gears
            //                Brep bDiffPart = new Brep();

            //                Brep[] bDiffParts = Brep.CreatePipe(shaftRail, shaftRadius + tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                bDiffPart = bDiffParts[0];

            //                partsForShaftDifference.Add(bDiffPart);

            //                // add two discs around the gear
            //                Point3d disc_first_pt1 = pt - projDir * (thickness + gap + discThickness);
            //                Point3d disc_first_pt2 = pt - projDir * (thickness + gap);
            //                Point3d disc_second_pt1 = pt + projDir * (thickness + gap + tolerance);
            //                Point3d disc_second_pt2 = pt + projDir * (thickness + gap + tolerance + discThickness);

            //                Line discFirstRailLine = new Line(disc_first_pt1, disc_first_pt2);
            //                Line discSecondRailLine = new Line(disc_second_pt1, disc_second_pt2);
            //                Curve firstDiscRail = discFirstRailLine.ToNurbsCurve();
            //                Curve secondDiscRail = discSecondRailLine.ToNurbsCurve();
            //                Brep[] firstDiscs = Brep.CreatePipe(firstDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                Brep[] secondDiscs = Brep.CreatePipe(secondDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                Brep firstDisc = firstDiscs[0];
            //                Brep secondDisc = secondDiscs[0];

            //                var shaftAllParts = Brep.CreateBooleanUnion(new List<Brep> { shaftRod, firstDisc, secondDisc }, MyDoc.ModelAbsoluteTolerance);
            //                if (shaftAllParts != null)
            //                {
            //                    shafts.Add(shaftAllParts[0]);
            //                    Shape shaftShape = new Shape(shaftAllParts[0],false,"shaft");
            //                    shaftEntities.Add(shaftShape);
            //                }

            //            }
            //            else
            //            {
            //                // only for the output gear

            //                if (outputAxle == 1)
            //                {
            //                    // the output gear shaft stay in the object
            //                    //Point3d closePt = pt.DistanceTo(projBrepPts[0]) > pt.DistanceTo(projBrepPts[1]) ? projBrepPts[1] : projBrepPts[0];
            //                    //Point3d shSt = pt.Y > closePt.Y ? closePt : pt;
            //                    //Point3d shEnd = pt.Y > closePt.Y ? (pt + projDir * 4) : (closePt + projDir * 4);

            //                    //Line sfRailLine = new Line(shSt, shEnd);
            //                    //Curve sfRail= sfRailLine.ToNurbsCurve();
            //                    //Brep[] shRods = Brep.CreatePipe(sfRail, shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    //Brep shRod = shRods[0];

            //                    //shafts.Add(shRod);

            //                    // create a brep to different the gears
            //                    Brep bDiffPart = new Brep();

            //                    Brep[] bDiffParts = Brep.CreatePipe(shaftRail, shaftRadius + tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    bDiffPart = bDiffParts[0];

            //                    partsForShaftDifference.Add(bDiffPart);

            //                    // add two discs around the gear
            //                    Point3d disc_first_pt1 = pt - projDir * (gap + discThickness);
            //                    Point3d disc_first_pt2 = pt - projDir * (gap);
            //                    Point3d disc_second_pt1 = pt + projDir * (thickness + gap + tolerance);
            //                    Point3d disc_second_pt2 = pt + projDir * (thickness + gap + tolerance + discThickness);

            //                    Line discFirstRailLine = new Line(disc_first_pt1, disc_first_pt2);
            //                    Line discSecondRailLine = new Line(disc_second_pt1, disc_second_pt2);
            //                    Curve firstDiscRail = discFirstRailLine.ToNurbsCurve();
            //                    Curve secondDiscRail = discSecondRailLine.ToNurbsCurve();
            //                    Brep[] firstDiscs = Brep.CreatePipe(firstDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    Brep[] secondDiscs = Brep.CreatePipe(secondDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    Brep firstDisc = firstDiscs[0];
            //                    Brep secondDisc = secondDiscs[0];

            //                    var shaftAllParts = Brep.CreateBooleanUnion(new List<Brep> { shaftRod, firstDisc, secondDisc }, MyDoc.ModelAbsoluteTolerance);
            //                    if (shaftAllParts != null)
            //                    {
            //                        shafts.Add(shaftAllParts[0]);
            //                        Shape shaftShape = new Shape(shaftAllParts[0], false, "shaft");
            //                        shaftEntities.Add(shaftShape);
            //                    }

            //                }
            //                else
            //                {
            //                    // the output gear shaft extend out of the object

            //                    Point3d newPt2 = pt2 - projDir * 4;
            //                    Point3d newPt1 = pt1 + projDir * 4;

            //                    Line shRailLine = new Line(newPt2, newPt1);
            //                    Curve shRail = shRailLine.ToNurbsCurve();
            //                    Brep[] shRods = Brep.CreatePipe(shRail, shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    Brep shRod = shRods[0];

            //                    shafts.Add(shRod);
            //                    Shape shaftShape = new Shape(shRod, false, "shaft");
            //                    shaftEntities.Add(shaftShape);

            //                    // difference with the model brep
            //                    Brep[] shDiffRods = Brep.CreatePipe(shRail, shaftRadius + tolerance * 5/4, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    shDiffRodSpiral = shDiffRods[0];

            //                    Vector3d translateDrivenPart = shRailLine.ToNurbsCurve().PointAtNormalizedLength(0.5) - DrivenPart.Direction.ToNurbsCurve().PointAtNormalizedLength(0.5);
            //                    Transform wheelTranslation = Transform.Translation(translateDrivenPart);

            //                    DrivenPart.GetModelinWorldCoordinate().Transform(wheelTranslation);
            //                    EntityList.Add(DrivenPart);

            //                    //_ = new Fixation(shaftShape, DrivenPart);
            //                    //_ = new Fixation(PinionGear, shaftShape);
            //                }

            //            }
            //        }

            //        #endregion
            //    }
            //    else
            //    {
            //        // create the big gear 
            //        gearDir = pt - gearCenters.ElementAt(gearCenters.IndexOf(pt) - 1);
            //        gearTeethNum = (int)(pinionTeethNum * grNew);
            //        Gear BigGear = new Gear(pt, gearDir, direction, gearTeethNum, currModule, pressureAngle, thickness, 0, false);
            //        Brep bigGear = BigGear.Model;

            //        _ = new Engagement(BigGear, gearEntities.Last());

            //        #region test by LH
            //        //MyDoc.Objects.AddBrep(bigGear);
            //        //MyDoc.Views.Redraw();
            //        #endregion

            //        //Vector3d planeVec = new Vector3d(1, 0, 0);
            //        //Transform directionRotate = Transform.Rotation(new Vector3d(0, 1, 0), gearDir, new Point3d(0, 0, 0));
            //        //planeVec.Transform(directionRotate);
            //        //Vector3d normalVec = new Vector3d(planeVec.Y * gearDir.Z - gearDir.Y * planeVec.Z,
            //        //    planeVec.Z * gearDir.X - gearDir.Z * planeVec.X,
            //        //    planeVec.X * gearDir.Y - gearDir.X * planeVec.Y);
            //        double meshAngle = Math.PI / (pinionTeethNum * grNew);
            //        Transform rotat = Transform.Rotation(meshAngle, new Vector3d(0,-1,0), pt);
            //        if(pinionTeethNum %2 == 1 && (pinionTeethNum * grNew) % 2 == 0)
            //        {
            //            bigGear.Transform(rotat);
            //        }

            //        BigGear.SetModel(bigGear);
            //        gearEntities.Add(BigGear);
            //        gears.Add(bigGear);

            //        if(gearCenters.IndexOf(pt) == gearCenters.Count() - 1)
            //        {   
            //            // create the last shaft
            //            List<Brep> bs = new List<Brep>();
            //            lastGearPos = pt;
            //            bs.Add(innerShell);
            //            List<Point3d> projPts = new List<Point3d>();
            //            projPts.Add(pt);
            //            Vector3d projDir = new Vector3d(0, 1, 0);
            //            Point3d[] projBrepPts;
            //            projBrepPts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(bs, projPts, projDir, MyDoc.ModelAbsoluteTolerance);

            //            // Set spring parameters
            //            springSSidePt = pt.DistanceTo(projBrepPts[0]) > pt.DistanceTo(projBrepPts[1]) ? projBrepPts[0] : projBrepPts[1];
            //            keySSidePt = pt.DistanceTo(projBrepPts[0]) > pt.DistanceTo(projBrepPts[1]) ? projBrepPts[1] : projBrepPts[0];
            //            springSCenter = (springSSidePt + pt) / 2 - 4 * projDir;
            //            lastGearRadius = currModule * gearTeethNum / 2;

            //            if (InputType == 1)
            //            {
            //                if (projBrepPts.Count() == 2)
            //                {
            //                    Point3d pt1 = new Point3d();    // the point on the postive side
            //                    Point3d pt2 = new Point3d();    // the point on the negative side

            //                    if (projBrepPts[0].Y > projBrepPts[1].Y)
            //                    {
            //                        pt1 = projBrepPts[0] + projDir * 0.2;
            //                        pt2 = projBrepPts[1] - projDir * 0.2;
            //                    }
            //                    else
            //                    {
            //                        pt1 = projBrepPts[1] + projDir * 0.2;
            //                        pt2 = projBrepPts[0] - projDir * 0.2;
            //                    }

            //                    // create the small pinion that engages with the rack
            //                    Point3d pinionPt = pt + new Vector3d(0, 1, 0) * (thickness + tolerance);
            //                    Gear pinion = new Gear(pinionPt, -gearDir, direction, pinionTeethNum, currModule, pressureAngle, thickness+tolerance, 0, false);
            //                    Brep pinionGear = pinion.Model;

            //                    _ = new Fixation(pinion, gearEntities.Last());
            //                    gearEntities.Add(pinion);
            //                    gears.Add(pinionGear);


            //                    // create the shaft
            //                    Line shaftRailLine = new Line(pt2, pt1);
            //                    Curve shaftRail = shaftRailLine.ToNurbsCurve();
            //                    Brep[] shaftRods = Brep.CreatePipe(shaftRail, shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    Brep shaftRod = shaftRods[0];


            //                    // create a brep to different the gears
            //                    Brep bDiffPart = new Brep();

            //                    Brep[] bDiffParts = Brep.CreatePipe(shaftRail, shaftRadius + tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    bDiffPart = bDiffParts[0];

            //                    partsForShaftDifference.Add(bDiffPart);

            //                    // add two discs around the gear
            //                    Point3d disc_first_pt1 = pt - projDir * (gap + discThickness);
            //                    Point3d disc_first_pt2 = pt - projDir * (gap);
            //                    Point3d disc_second_pt1 = pt + projDir * (thickness * 2 + tolerance + gap);
            //                    Point3d disc_second_pt2 = pt + projDir * (thickness * 2 + tolerance + gap + discThickness);

            //                    Line discFirstRailLine = new Line(disc_first_pt1, disc_first_pt2);
            //                    Line discSecondRailLine = new Line(disc_second_pt1, disc_second_pt2);
            //                    Curve firstDiscRail = discFirstRailLine.ToNurbsCurve();
            //                    Curve secondDiscRail = discSecondRailLine.ToNurbsCurve();
            //                    Brep[] firstDiscs = Brep.CreatePipe(firstDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    Brep[] secondDiscs = Brep.CreatePipe(secondDiscRail, discRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    Brep firstDisc = firstDiscs[0];
            //                    Brep secondDisc = secondDiscs[0];

            //                    var shaftAllParts = Brep.CreateBooleanUnion(new List<Brep> { shaftRod, firstDisc, secondDisc }, MyDoc.ModelAbsoluteTolerance);
            //                    if (shaftAllParts != null)
            //                    {
            //                        shafts.Add(shaftAllParts[0]);
            //                        Shape shaftshape = new Shape(shaftAllParts[0], false, "shaft");
            //                        shaftEntities.Add(shaftshape);
            //                    }
            //                }
            //            }
            //            else
            //            {
            //                if (projBrepPts.Count() == 2)
            //                {
            //                    // Generate the revolute joint

            //                    #region Step 1: geneate the central rod
            //                    Point3d pt1 = new Point3d();    // the point on the negative side
            //                    Point3d pt2 = new Point3d();    // the point on the postive side


            //                    if (projBrepPts[0].Y > projBrepPts[1].Y)
            //                    {
            //                        pt2 = projBrepPts[0] - projDir * 0.8;
            //                        pt1 = projBrepPts[1] - projDir * 20;
            //                    }
            //                    else
            //                    {
            //                        pt2 = projBrepPts[1] - projDir * 0.8;
            //                        pt1 = projBrepPts[0] - projDir * 20;
            //                    }
            //                    Line shaftRailLine = new Line(pt1, pt2);
            //                    Curve shaftRail = shaftRailLine.ToNurbsCurve();
            //                    Brep[] shaftRods = Brep.CreatePipe(shaftRail, shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    Brep shaftRod = shaftRods[0];

            //                    shaftCutoutBrep = Brep.CreatePipe(shaftRail, shaftRadius + tolerance, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];


            //                    // Create the handler -- the key
            //                    Vector3d xpos = new Vector3d(1, 0, 0);
            //                    Point3d handlerSt = pt1 - xpos * shaftRadius;
            //                    Point3d handlerEnd = pt1 + xpos * shaftRadius;
            //                    Line handlerLine = new Line(handlerSt, handlerEnd);
            //                    Curve handlerRail = handlerLine.ToNurbsCurve();

            //                    Brep handlerBrep = Brep.CreatePipe(handlerRail, 5, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

            //                    handlerBrep.Transform(translateBack);
            //                    handlerBrep.Transform(rotationBack);
            //                    handlerBrep.Transform(postRotationBack);
            //                    handlerShape = new Shape(handlerBrep, false, "key");
            //                    EntityList.Add(handlerShape);

            //                    shaftRod.Transform(translateBack);
            //                    shaftRod.Transform(rotationBack);
            //                    shaftRod.Transform(postRotationBack);

            //                    shaftRodShape = new Shape(shaftRod, false, "joint");
            //                    EntityList.Add(shaftRodShape);

            //                    //_ = new Fixation(shaftRodShape, BigGear);
            //                    //_ = new Fixation(shaftRodShape, shaftEntities.First());
            //                    _ = new Fixation(shaftRodShape, DrivenPart);

            //                    #endregion

            //                    #region Step 2: generate the disc

            //                    Point3d pt3 = pt2 - projDir * 1.6;
            //                    Line discRailLine = new Line(pt3, pt2);
            //                    Curve discRail = discRailLine.ToNurbsCurve();
            //                    Brep[] discRods = Brep.CreatePipe(discRail, shaftRadius * 2, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians);
            //                    Brep discRod = discRods[0];

            //                    discRod.Transform(translateBack);
            //                    discRod.Transform(rotationBack);
            //                    discRod.Transform(postRotationBack);

            //                    discRodShape = new Shape(discRod, false, "joint");
            //                    EntityList.Add(discRodShape);
            //                    #endregion

            //                    #region Step 3: generate the bearing

            //                    Point3d pt4 = pt2 + projDir;
            //                    Point3d pt5 = pt3 - projDir * gap;

            //                    Line bearingLine = new Line(pt5, pt4);
            //                    Curve bearingRail = bearingLine.ToNurbsCurve();
            //                    Brep bearingExterior = Brep.CreatePipe(bearingRail, shaftRadius * 2 + gap + shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];
            //                    Brep bearingInterior = Brep.CreatePipe(bearingRail, shaftRadius * 2 + gap, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

            //                    Brep laterUsedBrep = Brep.CreatePipe(bearingRail, shaftRadius + gap, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];
            //                    modelDiffLaterUsedBrep = bearingExterior.DuplicateBrep();


            //                    var bearingBreps = Brep.CreateBooleanDifference(bearingExterior, bearingInterior, MyDoc.ModelAbsoluteTolerance);
            //                    if (bearingBreps == null)
            //                    {
            //                        bearingInterior.Flip();
            //                        bearingBreps = Brep.CreateBooleanDifference(bearingExterior, bearingInterior, MyDoc.ModelAbsoluteTolerance);
            //                    }
            //                    Brep bearingBrep = bearingBreps[0];

            //                    double capThickness = 1.2;
            //                    Point3d pt6 = pt5 - projDir * capThickness;
            //                    Line capLine = new Line(pt6, pt5);
            //                    Curve capRail = capLine.ToNurbsCurve();
            //                    Brep capBrep = Brep.CreatePipe(capRail, shaftRadius * 2 + gap + shaftRadius, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];
            //                    Brep capDiffBrep = Brep.CreatePipe(capRail, shaftRadius + gap, false, PipeCapMode.Flat, true, MyDoc.ModelAbsoluteTolerance, MyDoc.ModelAngleToleranceRadians)[0];

            //                    Brep capBrep1 = Brep.CreateBooleanDifference(capBrep, capDiffBrep, MyDoc.ModelAbsoluteTolerance)[0];


            //                    Brep bearingAllBrep = Brep.CreateBooleanUnion(new List<Brep> { bearingBrep, capBrep1}, MyDoc.ModelAbsoluteTolerance)[0];

            //                    Point3d openSlotPt = (pt2 + 0.6 * projDir + pt5 + 0.2*projDir) / 2;
            //                    Vector3d osNormal = new Vector3d(0, 0, 1);
            //                    Plane openSlotPln = new Plane(openSlotPt, osNormal);

            //                    // create sweep function
            //                    var sweep = new Rhino.Geometry.SweepOneRail();
            //                    sweep.AngleToleranceRadians = MyDoc.ModelAngleToleranceRadians;
            //                    sweep.ClosedSweep = false;
            //                    sweep.SweepTolerance = MyDoc.ModelAbsoluteTolerance;

            //                    Vector3d os_xp = shaftRadius * openSlotPln.XAxis;
            //                    Vector3d os_xn = (-shaftRadius) * openSlotPln.XAxis;
            //                    Vector3d os_yp = (pt2.DistanceTo(pt5) + 0.4) / 2 * openSlotPln.YAxis;
            //                    Vector3d os_yn = (-1) * (pt2.DistanceTo(pt5) + 0.4) / 2 * openSlotPln.YAxis;

            //                    Point3d[] openSlotPts = new Point3d[5];
            //                    openSlotPts[0] = openSlotPt + os_xp + os_yp;
            //                    openSlotPts[1] = openSlotPt + os_xn + os_yp;
            //                    openSlotPts[2] = openSlotPt + os_xn + os_yn;
            //                    openSlotPts[3] = openSlotPt + os_xp + os_yn;
            //                    openSlotPts[4] = openSlotPt + os_xp + os_yp;
            //                    Curve openSlotRect = new Polyline(openSlotPts).ToNurbsCurve();


            //                    Point3d[] openSlotRailPts = new Point3d[2];
            //                    openSlotRailPts[0] = openSlotPt;
            //                    openSlotRailPts[1] = openSlotPt + osNormal * 20;
            //                    Curve osRail = new Polyline(openSlotRailPts).ToNurbsCurve();

            //                    Brep openSlotBrep = new Brep();

            //                    openSlotBrep = sweep.PerformSweep(osRail, openSlotRect)[0];
            //                    openSlotBrep = openSlotBrep.CapPlanarHoles(MyDoc.ModelAbsoluteTolerance);

            //                    Brep openSlotBrep1 = openSlotBrep.DuplicateBrep();

            //                    Transform openSlotBrepRotation = Transform.Rotation(Math.PI / 2, -projDir, openSlotPt);
            //                    Transform openSlotBrepRotation1 = Transform.Rotation(Math.PI, -projDir, openSlotPt);
            //                    openSlotBrep1.Transform(openSlotBrepRotation);


            //                    Brep firstBlock = Brep.CreateBooleanUnion(new List<Brep> { openSlotBrep, openSlotBrep1 }, MyDoc.ModelAbsoluteTolerance)[0];

            //                    Brep openSlotBrep2 = firstBlock.DuplicateBrep();
            //                    openSlotBrep2.Transform(openSlotBrepRotation1);

            //                    Brep secondBlock0 = Brep.CreateBooleanUnion(new List<Brep> { firstBlock, laterUsedBrep }, MyDoc.ModelAbsoluteTolerance)[0];

            //                    Brep secondBlock = Brep.CreateBooleanUnion(new List<Brep> { secondBlock0, openSlotBrep2 }, MyDoc.ModelAbsoluteTolerance)[0];


            //                    Brep[] bearingFinals = Brep.CreateBooleanDifference(bearingAllBrep, secondBlock, MyDoc.ModelAbsoluteTolerance);
            //                    if(bearingFinals == null)
            //                    {
            //                        secondBlock.Flip();
            //                        bearingFinals = Brep.CreateBooleanDifference(bearingAllBrep, secondBlock, MyDoc.ModelAbsoluteTolerance);
            //                    }
            //                    Brep bearingFinal = bearingFinals[0];

            //                    bearingFinal.Transform(translateBack);
            //                    bearingFinal.Transform(rotationBack);
            //                    bearingFinal.Transform(postRotationBack);

            //                    bearingShape= new Shape(bearingFinal, false, "joint");
            //                    EntityList.Add(bearingShape);
            //                    #endregion

            //                }
            //            }

            //        }
            //    }
            //}

            //// Boolean different the gears from partsForShaftDifference to drill the gears except for the output gear

            //var finalGears = Brep.CreateBooleanDifference(gears, partsForShaftDifference, MyDoc.ModelAbsoluteTolerance);
            //if (finalGears == null)
            //{
            //    foreach(Brep b in partsForShaftDifference)
            //    {
            //        b.Flip();
            //    }
            //    finalGears = Brep.CreateBooleanDifference(gears, partsForShaftDifference, MyDoc.ModelAbsoluteTolerance);
            //}


            //#region Last step: add all objects to the entitylist
            ////// so far, finalGears has all the gears, shafts has all the gear shafts (w/ or w/out discs)

            ////foreach(Brep s in shafts)
            ////{
            ////    MyDoc.Objects.AddBrep(s);
            ////    MyDoc.Views.Redraw();
            ////}

            //int idx = 0;
            //foreach(Brep g in finalGears)
            //{
            //    //MyDoc.Objects.AddBrep(g);
            //    //MyDoc.Views.Redraw();
            //    //result.Add(g);
            //    gearEntities.ElementAt(idx).SetModel(g);
            //    idx++;
            //}

            //foreach(Shape sh in shaftEntities)
            //{
            //    Brep b_origi = sh.GetModelinWorldCoordinate();
            //    b_origi.Transform(translateBack);
            //    b_origi.Transform(rotationBack);
            //    b_origi.Transform(postRotationBack);
            //    sh.SetModel(b_origi);

            //    EntityList.Add(sh);
            //}
            //foreach(Gear g in gearEntities)
            //{
            //    Brep b_origi = g.GetModelinWorldCoordinate();
            //    b_origi.Transform(translateBack);
            //    b_origi.Transform(rotationBack);
            //    b_origi.Transform(postRotationBack);
            //    g.SetModel(b_origi);

            //    EntityList.Add(g);
            //}
            ////if (springS != null)
            ////    _ = new Fixation(shaftRodShape, SpringS);
            //#endregion

            //#endregion

            //#endregion
            #endregion
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
        public void CreateShell()
        {
            double shellThickness = 2;
            Brep part2;
            GearParameter lgp = _gearParam.parameters.Last();
            Brep lgCylinder = new Cylinder(new Circle(new Plane(lgp.center, lgp.norm), lgp.radius + 2), lgp.faceWidth + 0.6 + 1.3 * 2).ToBrep(true, true);
            lgCylinder.Transform(Transform.Translation(-lgp.norm * 1.6));
            part2 = Brep.CreateBooleanDifference(b2, lgCylinder, myDoc.ModelAbsoluteTolerance)[0];
            Brep[] shells = Brep.CreateOffsetBrep(b2, (-1) * shellThickness, false, true, myDoc.ModelRelativeTolerance, out _, out _);
            Brep innerShell = shells[0];
            innerShell.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == innerShell.SolidOrientation)
                innerShell.Flip();
            part2 = Brep.CreateBooleanDifference(part2, innerShell, myDoc.ModelAbsoluteTolerance)[0];
            //Cut b2 with shaft if needed
            foreach (Entity e in entityList)
            {
                if (e.Name == "MiddleShellBreakerShaft")
                {
                    Shaft s = (Shaft)e;
                    Plane p = new Plane(s.StartPt, s.AxisDir);
                    Circle c = new Circle(p, 2.1);
                    Cylinder cy = new Cylinder(c, s.Len);
                    part2 = Brep.CreateBooleanDifference(part2, cy.ToBrep(true, true), myDoc.ModelAbsoluteTolerance)[0];
                }
            }
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
            Brep cutBox = new Box(boxPlane, new Interval(-bboxMainDimension * 0.35, bboxMainDimension * 0.35), new Interval(-10, 10)
                , new Interval(0, bboxMainDimension * 5)).ToBrep();
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
            part2 = Brep.CreateBooleanDifference(part2, cutBarrel, myDoc.ModelAbsoluteTolerance)[0];
            part2.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == part2.SolidOrientation)
                part2.Flip();

            #endregion

            //Cut b3 with gear cylinder
            Brep part3 = Brep.CreateBooleanDifference(b3, lgCylinder, myDoc.ModelAbsoluteTolerance)[0];
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
        #endregion
    }
}
