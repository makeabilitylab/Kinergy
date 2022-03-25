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
        private Entity spring;
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
        public IntermittentRotation(Brep Model,  Vector3d Direction, double Energy, double Angle, int Speed)
        {
            model = Model;
            energy = Energy;
            angle = Angle;
            direction = Direction;
            modelCut = new List<Shape>();
            myDoc = RhinoDoc.ActiveDoc;
            locks = new List<Lock>();
            speed = Speed;
        }
        public void AddSprings(Entity springControl)
        {
            entityList.Remove(spring);
            spring = springControl;
            entityList.Add(spring);
        }

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
                    lockPartIdx.Clear();
                }

                if (locks.Count > 0)
                    locks.Clear();

                Lock LockHead;
                double ratchetRadius = lockDisToAxis * 0.5;

                if (spiralLockNorm)
                    LockHead = new Lock(spiralLockCen, spiralLockDir, ratchetRadius, true);
                else
                    LockHead = new Lock(spiralLockCen, spiralLockDir, ratchetRadius, false);

                Vector3d centerLinkDirection = new Vector3d(spiralLockCen) - new Vector3d(lockSelection.lockCtrlPointSelected);
                double centerLinkLen = centerLinkDirection.Length;
                centerLinkDirection.Unitize();


                #region add lock parts to the entitylist

                cutBarrel = null;
                addBarrel = null;
                Lock lockBase = new Lock(spiralLockDir, spiralLockCen, lockSelection.lockCtrlPointSelected, ratchetRadius, false, myDoc, ref cutBarrel, ref addBarrel, "lockbase");
                Entity tempAddBarrel = new Entity(addBarrel, false, "");
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
            Brep cutBox = new Box(boxPlane, new Interval(-bboxMainDimension * 0.3, bboxMainDimension * 0.3), new Interval(-10, 10)
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
            BoundingBox part3bbox = part3.GetBoundingBox(true);
            Brep cuttingRod = new Cylinder(new Circle(new Plane(part3bbox.Center, _otherAxis), 5), bboxMainDimension * 10).ToBrep(true, true);
            cuttingRod.Transform(Transform.Translation(_otherAxis * (-bboxMainDimension * 5)));
            cuttingRod.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == cuttingRod.SolidOrientation)
                cuttingRod.Flip();
            try
            {
                part3 = Brep.CreateBooleanDifference(part3, cuttingRod, myDoc.ModelAbsoluteTolerance)[0];
            }
            catch { }
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
            part3.Transform(Transform.Translation(_mainAxis * clearance2));
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
        public List<Lock> Locks { get => locks; set => locks = value; }
        public List<Shape> ModelCut { get => modelCut; set => modelCut = value; }
        public Curve Skeleton { get => skeleton; set => skeleton = value; }
        public Brep Model { get => model; set => model = value; }
        public double Energy { get => energy; set => energy = value; }
        public int Speed { get => speed; set => speed = value; }
        public double Angle { get => angle; set => angle = value; }
    }
}
