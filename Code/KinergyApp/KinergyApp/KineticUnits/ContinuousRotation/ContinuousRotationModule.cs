using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy.KineticUnit;
using KinergyUtilities;
using Rhino;
using Rhino.DocObjects;
using Rhino.Input;
using HumanUIforKinergy.KinergyUtilities;
using Kinergy.Geom;
using System.Linq;
using System.Drawing;

namespace ConRotation
{
    public class ContinuousRotationModule : GH_Component
    {
        // Variables that store information
        Brep model;             // the original brep model
        Brep conBrep;           // the Brep that is selected and converted
        Brep innerCavity;
        double t1, t2; // the positoins of start point and end point of the segment on the normalized skeleton
        Curve skeleton;     // skeleton
        int speedLevel;         // value of the strength slide bar
        int roundLevel;   // value of the speed slide bar
        Vector3d direction;             // kinetic unit direction
        ContinuousRotation motion;
        List<Arrow> lockDirCandidates;
        Arrow p;
        int energyChargingMethod;       // pressing: 1; turning: 2
        Vector3d orientationDir;
        int outputAxle;
        bool OperatingOutputAxleMethod;
        bool multipleSelections;

        //Transform dirToXTranlationBack;
        //Transform dirToXRotationBack;
        //Transform yToPoseTrans;
        Vector3d axisToSkeleton = new Vector3d(0, 0, 0);
        Vector3d skeletonToAxis = new Vector3d(0, 0, 0);
        Transform rotateAxisToOrientation = Transform.Identity;
        Transform rotateOrientationToAxis = Transform.Identity;
        Transform transSkeleton = Transform.Identity;
        Transform transSkeletonBack = Transform.Identity;
        Transform offsetTranslation = Transform.Identity;

        Transform eeDirectionToYAxis = Transform.Identity;
        Transform eeYAxisToDirection = Transform.Identity;

        Guid xIndicatorID, yIndicatorID, zIndicatorID;
        double innerHeight, innerDepth;

        List<Brep> allBrepExisted = new List<Brep>();
        List<Guid> endEffectorIDs = new List<Guid>();

        int alignment = -1; // 1: aligned with X axis; 2: aligned with Y axis; 3: aligned with Z axis
        Point3d alignmentPt = new Point3d();

        // Variables used for different functions
        bool lockState;
        double min_wire_diamter;
        double min_coil_num;
        double energy;
        double speed;
        bool isLockSet;
        Guid selObjId;
        List<Guid> toBeBaked;

        // Region selection related variables
        Point3d center = Point3d.Unset;
        Guid guid1, guid2, ArrowCurve;
        bool OperatingArrow = false;
        bool OperatingEnergyChargingMethod = false;
        bool PlaneGenerated = false;
        bool ArrowGenerated = false;
        bool PlaneSelected = false;
        double arrowScale;
        Plane pl1, pl2;
        PlaneSurface s1, s2;
        Guid selected = Guid.Empty;
        Vector3d skeletonVec = Vector3d.Unset;
        Vector3d v = Vector3d.Unset;
        ProcessingWin processingwin = new ProcessingWin();

        RhinoDoc myDoc;
        bool testBodySelBtn;
        bool testMotionControlPosSetBtn;
        bool testEEPosSetBtn;
        bool testMotionAxisDirSetBtn;
        bool testPreBtn;
        bool testBakeBtn;
        int motionControlMethod; // 1: press; 2: turn


        /// <summary>
        /// Initializes a new instance of the ContinuousRotationModule class.
        /// </summary>
        public ContinuousRotationModule()
          : base("ContinuousRotationModule", "CRModule",
              "The kinetic unit for continuous rotation",
              "Kinergy", "KineticUnits")
        {
            model = null;
            conBrep = new Brep();
            innerCavity = new Brep();
            t1 = 0;
            t2 = 1;
            skeleton = null;
            roundLevel = 5;
            speedLevel = 5;
            direction = new Vector3d();
            motion = null;
            lockDirCandidates = new List<Arrow>();
            p = null;
            energyChargingMethod = 1;
            myDoc = RhinoDoc.ActiveDoc;
            orientationDir = new Vector3d();
            outputAxle = 1;
            OperatingOutputAxleMethod = false;
            multipleSelections = false;

            lockState = false;
            min_wire_diamter = 2.8;
            min_coil_num = 3;
            energy = 0.5;
            speed = 0.5;
            arrowScale = 0;
            isLockSet = false;
            selObjId = Guid.Empty;
            toBeBaked = new List<Guid>();
            xIndicatorID = Guid.Empty;
            yIndicatorID = Guid.Empty;
            zIndicatorID = Guid.Empty;
            innerHeight = double.MaxValue;
            innerDepth = double.MaxValue;

            testBodySelBtn = false;
            testMotionControlPosSetBtn = false;
            testEEPosSetBtn = false;
            testMotionAxisDirSetBtn = false;
            testPreBtn = false;
            testBakeBtn = false;
            motionControlMethod = -1;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // User triggers for actions
            pManager.AddBooleanParameter("RegionSelection", "Reg", "Enabling region selection and direction calculation", GH_ParamAccess.item);
            pManager.AddIntegerParameter("MotionControlMethod", "MCM", "The method to control the motion (press or turn)", GH_ParamAccess.item);
            pManager.AddBooleanParameter("SetMotionControlPos", "MCPos", "Set the motion control position", GH_ParamAccess.item);
            pManager.AddBooleanParameter("SetEEPos", "EEPos", "Set the end-effector position", GH_ParamAccess.item);
            pManager.AddBooleanParameter("SetAxisDir", "ADir", "Set the motion axis direction", GH_ParamAccess.item);

            pManager.AddBooleanParameter("AddLock", "L", "Enabling locking", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Preview", "Pre", "Enabling preview", GH_ParamAccess.item);

            // Value listeners 
            pManager.AddIntegerParameter("Speed", "S", "Speed of motion", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Rounds", "R", "The rounds of the output motion", GH_ParamAccess.item);

            // Confirm and bake all components
            pManager.AddBooleanParameter("ComponentsBake", "Bk", "comfirm and bake all components", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Kinetic unit", "KU", "Kinetic Unit instance for instant translation", GH_ParamAccess.item);
            //pManager.AddBrepParameter("Original brep", "Brep", "The target model to move", GH_ParamAccess.item);
            pManager.AddBrepParameter("Models", "M", "", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Preview launcher", "Pre", "enable the preview", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool reg_input = false, control_pos = false, ee_pos = false, motion_axis = false,  addlock_input = false, pre_input = false, bake_input = false;
            int motion_control_method = -1;
            int speed_input = 5;
            int round_input = 5;

            #region input param readings
            if (!DA.GetData(0, ref reg_input))
                return;
            if (!DA.GetData(1, ref motion_control_method))
                return;
            if (!DA.GetData(2, ref control_pos))
                return;
            if (!DA.GetData(3, ref ee_pos))
                return;
            if (!DA.GetData(4, ref motion_axis))
                return;
            if (!DA.GetData(5, ref addlock_input))
                return;
            if (!DA.GetData(6, ref pre_input))
                return;
            if (!DA.GetData(7, ref speed_input))
                return;
            if (!DA.GetData(8, ref round_input))
                return;
            if (!DA.GetData(9, ref bake_input))
                return;
            #endregion

            // variables to control states
            bool toSelectRegion = false, toSetMotionControl = false, toSetEEPos = false, toSetAxisDir = false, toAdjustParam = false, toAddLock = false, toRemoveLock = false, toPreview = false, toBake = false;

            #region Input check. This determines how the cell respond to changed params
            if (!reg_input && testBodySelBtn)
            {
                toSelectRegion = true;
                testBodySelBtn = false;
            }
            else if (reg_input)
            {
                testBodySelBtn = true;
            }

            motionControlMethod = motion_control_method;

            if (!control_pos && testMotionControlPosSetBtn)
            {
                toSetMotionControl = true;
                testMotionControlPosSetBtn = false;
            }
            else if (control_pos)
            {
                testMotionControlPosSetBtn = true;
            }

            if (!ee_pos && testEEPosSetBtn)
            {
                toSetEEPos = true;
                testEEPosSetBtn = false;
            }
            else if (ee_pos)
            {
                testEEPosSetBtn = true;
            }

            if (!motion_axis && testMotionAxisDirSetBtn)
            {
                toSetAxisDir = true;
                testMotionAxisDirSetBtn = false;
            }
            else if (motion_axis)
            {
                testMotionAxisDirSetBtn = true;
            }

            if (lockState != addlock_input)
            {
                lockState = addlock_input;
                if (lockState)
                    toAddLock = true;
                else
                    toRemoveLock = true;
            }
            if (!pre_input && testPreBtn)
            {
                toPreview = true;
                testPreBtn = false;
            }
            else if (pre_input)
            {
                testPreBtn = true;
            }

            if (!bake_input && testBakeBtn)
            {
                toBake = true;
                testBakeBtn = false;
            }
            else if (bake_input)
            {
                testBakeBtn = true;
            }

            if (speedLevel == speed_input && roundLevel == round_input)
            {
                toAdjustParam = false;
            }
            else
            {
                speedLevel = speed_input;
                roundLevel = round_input;
                toAdjustParam = true;
            }

            #endregion

            if (toBake)
            {
                if (motion != null)
                {
                    if (motion.EntityList != null)
                    {
                        foreach (Entity b in motion.EntityList)
                        {
                            Brep tempB = b.GetModelinWorldCoordinate();
                            RhinoDoc.ActiveDoc.Objects.AddBrep(tempB);
                        }
                        RhinoDoc.ActiveDoc.Views.Redraw();
                        this.ExpirePreview(true);
                    }
                }
            }

            if (toSelectRegion)
            {
                // select the target model and the region to be converted

                if (selObjId != Guid.Empty)
                {
                    RhinoDoc.ActiveDoc.Objects.Show(selObjId, true);
                    RhinoDoc.ActiveDoc.Views.Redraw();
                    toBeBaked.Clear();
                    selObjId = Guid.Empty;
                }

                ObjRef objSel_ref;
                var rc = RhinoGet.GetOneObject("Select a surface or polysurface", false, ObjectType.AnyObject, out objSel_ref);
                if (rc == Rhino.Commands.Result.Success)
                {
                    #region Pre-process #1: get the selected brep

                    selObjId = objSel_ref.ObjectId;
                    ObjRef currObj = new ObjRef(selObjId);

                    model = currObj.Brep();

                    #endregion

                    #region Pre-process #2: ask the user to specify the axis the object is aligned with

                    GenerateCoordinateIndicators(myDoc, model);
                    Rhino.Input.Custom.GetPoint indicatorSelectPt = new Rhino.Input.Custom.GetPoint();
                    indicatorSelectPt.SetCommandPrompt(@"Please select the axis the object is aligned with.");
                    indicatorSelectPt.MouseMove += IndicatorSelectPt_MouseMove;
                    indicatorSelectPt.MouseDown += IndicatorSelectPt_MouseDown;
                    indicatorSelectPt.Get(true);

                    myDoc.Objects.Hide(xIndicatorID, true);
                    myDoc.Objects.Hide(yIndicatorID, true);
                    myDoc.Objects.Hide(zIndicatorID, true);
                    myDoc.Views.Redraw();

                    switch (alignment)
                    {
                        case 1: v = Vector3d.XAxis; break;
                        case 2: v = Vector3d.YAxis; break;
                        case 3: v = Vector3d.ZAxis; break;
                        default: break;
                    }

                    #endregion

                    #region Step 1: drag the two planes to decide the portion

                    Rhino.Input.Custom.GetPoint gp2 = new Rhino.Input.Custom.GetPoint();
                    gp2.SetCommandPrompt("Click and drag the partition plane to adjust their position. Press enter to confirm and move on");
                    gp2.MouseDown += Gp2_MouseDown; 
                    gp2.MouseMove += Gp2_MouseMove; 
                    gp2.AcceptNothing(true);

                    Rhino.Input.GetResult r2;
                    do
                    {
                        if (!PlaneGenerated)
                            GeneratePlanes();
                        r2 = gp2.Get(true);

                    } while (r2 != Rhino.Input.GetResult.Nothing);
                    PlaneSelected = true;

                    #endregion

                    #region Step 2: find the biggest inner cavity 

                    RhinoDoc.ActiveDoc.Objects.Delete(guid1, true);
                    RhinoDoc.ActiveDoc.Objects.Delete(guid2, true);
                    PlaneGenerated = false;

                    if (PlaneSelected)
                    {
                        Plane p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), -v);
                        Plane p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), v);

                        Brep[] Cut_Brep1rest = model.Trim(p1Reverse, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

                        Brep BrepRest = null;
                        try
                        {
                            BrepRest = Cut_Brep1rest[0];
                        }
                        catch
                        {
                            BrepRest = model;
                        }

                        Brep[] Cut_Brep2 = BrepRest.Trim(p2Reverse, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                        Brep BrepTarget = null;
                        try
                        {
                            BrepTarget = Cut_Brep2[0];
                        }
                        catch
                        {
                            BrepTarget = BrepRest;
                        }

                        try
                        {
                            BrepTarget = Cut_Brep2[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                        }
                        catch
                        { }

                        switch (alignment)
                        {
                            case 1:
                                {
                                    // aligned with the X axis
                                    // height is along Z axis
                                    // depth is along Y axis

                                    BoundingBox tarBox = BrepTarget.GetBoundingBox(true);
                                    tarBox.Inflate(-0.2);
                                    Brep boxBrep = tarBox.ToBrep();
                                    double ori_z = tarBox.Max.Z - tarBox.Min.Z;
                                    double ori_x = tarBox.Max.X - tarBox.Min.X;
                                    double ori_y = tarBox.Max.Y - tarBox.Min.Y;

                                    int i = 0;
                                    while(Brep.CreateBooleanIntersection(BrepTarget, boxBrep, myDoc.ModelAbsoluteTolerance).Count() > 0)
                                    {
                                        if(i%2 == 0)
                                        {
                                            // decrease height
                                            Transform hScale = Transform.Scale(Plane.WorldXY, 1, 1, (1 - 0.1 / ori_z));
                                            boxBrep.Transform(hScale);
                                        }
                                        else
                                        {
                                            // decrease depth
                                            Transform dScale = Transform.Scale(Plane.WorldXY, 1, (1 - 0.1/ori_y), 1);
                                            boxBrep.Transform(dScale);
                                        }
                                        i += 1;
                                    }

                                    innerCavity = boxBrep;

                                    BoundingBox innerCavityBox = innerCavity.GetBoundingBox(true);
                                    innerDepth = innerCavityBox.Max.Y - innerCavityBox.Min.Y;
                                    innerHeight = innerCavityBox.Max.Z - innerCavityBox.Min.Z;

                                }
                                break;
                            case 2:
                                {
                                    // aligned with the Y axis
                                    // height is along Z axis
                                    // depth is along X axis

                                    BoundingBox tarBox = BrepTarget.GetBoundingBox(true);
                                    tarBox.Inflate(-0.2);
                                    Brep boxBrep = tarBox.ToBrep();
                                    double ori_z = tarBox.Max.Z - tarBox.Min.Z;
                                    double ori_x = tarBox.Max.X - tarBox.Min.X;
                                    double ori_y = tarBox.Max.Y - tarBox.Min.Y;

                                    int i = 0;
                                    while (Brep.CreateBooleanIntersection(BrepTarget, boxBrep, myDoc.ModelAbsoluteTolerance).Count() > 0)
                                    {
                                        if (i % 2 == 0)
                                        {
                                            // decrease height
                                            Transform hScale = Transform.Scale(Plane.WorldXY, 1, 1, (1 - 0.1 / ori_z));
                                            boxBrep.Transform(hScale);
                                        }
                                        else
                                        {
                                            // decrease depth
                                            Transform dScale = Transform.Scale(Plane.WorldXY, (1 - 0.1/ori_x), 1, 1);
                                            boxBrep.Transform(dScale);
                                        }
                                        i += 1;
                                    }

                                    innerCavity = boxBrep;

                                    BoundingBox innerCavityBox = innerCavity.GetBoundingBox(true);
                                    innerDepth = innerCavityBox.Max.X - innerCavityBox.Min.X;
                                    innerHeight = innerCavityBox.Max.Z - innerCavityBox.Min.Z;

                                }
                                break;
                            case 3:
                                {
                                    // aligned with the Z axis
                                    // height is along y axis
                                    // depth is along x axis

                                    BoundingBox tarBox = BrepTarget.GetBoundingBox(true);
                                    tarBox.Inflate(-0.2);
                                    Brep boxBrep = tarBox.ToBrep();
                                    double ori_z = tarBox.Max.Z - tarBox.Min.Z;
                                    double ori_x = tarBox.Max.X - tarBox.Min.X;
                                    double ori_y = tarBox.Max.Y - tarBox.Min.Y;

                                    int i = 0;
                                    while (Brep.CreateBooleanIntersection(BrepTarget, boxBrep, myDoc.ModelAbsoluteTolerance).Count() > 0)
                                    {
                                        if (i % 2 == 0)
                                        {
                                            // decrease height
                                            Transform hScale = Transform.Scale(Plane.WorldXY, 1, (1 - 0.1/ori_y), 1);
                                            boxBrep.Transform(hScale);
                                        }
                                        else
                                        {
                                            // decrease depth
                                            Transform dScale = Transform.Scale(Plane.WorldXY, 1, (1 - 0.1 / ori_y), 1);
                                            boxBrep.Transform(dScale);
                                        }
                                        i += 1;
                                    }

                                    innerCavity = boxBrep;

                                    BoundingBox innerCavityBox = innerCavity.GetBoundingBox(true);
                                    innerDepth = innerCavityBox.Max.X - innerCavityBox.Min.X;
                                    innerHeight = innerCavityBox.Max.Y - innerCavityBox.Min.Y;
                                }
                                break;
                            default: break;
                        }
                        
                    }

                    #endregion

                    #region Step 3: Input the energy-charging method

                    RhinoApp.KeyboardEvent += RhinoApp_KeyboardEvent1;
                    Rhino.Input.Custom.GetPoint gp3 = new Rhino.Input.Custom.GetPoint();
                    gp3.SetCommandPrompt(@"Press '1' to select 'Pressing' or '2' to select 'Turning' as the energy-charging method. Press 'Enter' to continue.");
                    gp3.AcceptNothing(true);
                    Rhino.Input.GetResult r3;

                    OperatingEnergyChargingMethod = true;
                    do
                    {
                        r3 = gp3.Get(true);

                    } while (r3 != Rhino.Input.GetResult.Nothing);
                    OperatingEnergyChargingMethod = false;

                    #endregion

                }

            }

            #region old code
            //if (toSetEndEffector)
            //{
            //    // Ask the user to select a Brep and calculate the orientation of the embedded kinetic unit
            //    List<ObjRef> objSel_refs = new List<ObjRef>();
            //    List<Guid> selObjId1s = new List<Guid>();
            //    List<Brep> eeBreps = new List<Brep>();

            //    #region Pre-process step: ask the user to select the end-effector(s)
            //    RhinoApp.KeyboardEvent += RhinoApp_KeyboardEvent3;
            //    Rhino.Input.Custom.GetPoint gp7 = new Rhino.Input.Custom.GetPoint();
            //    gp7.SetCommandPrompt(@"Please select at least one Brep or Surface as the end-effector. Press 'Enter' to continue.");
            //    gp7.AcceptNothing(true);
            //    Rhino.Input.GetResult r7;

            //    multipleSelections = true;
            //    do
            //    {
            //        ObjRef tempObjSel_ref;
            //        Guid selObjIdTemp = Guid.Empty;

            //        var rc = RhinoGet.GetOneObject(@"Please select at least one Brep or Surface as the end-effector. Press 'Enter' to continue.", false, ObjectType.AnyObject, out tempObjSel_ref);
            //        if (rc == Rhino.Commands.Result.Success)
            //        {
            //            // select a brep
            //            selObjIdTemp = tempObjSel_ref.ObjectId;
            //            endEffectorIDs.Add(selObjIdTemp);
            //            ObjRef currObj = new ObjRef(selObjIdTemp);

            //            Brep endeffector = currObj.Brep();
            //            eeBreps.Add(endeffector);

            //        }
            //        r7 = gp7.Get(true);

            //    } while (r7 != Rhino.Input.GetResult.Nothing);
            //    multipleSelections = false;

            //    #endregion

            //    #region Step 0: set the output gear axle type

            //    RhinoApp.KeyboardEvent += RhinoApp_KeyboardEvent2; ;
            //    Rhino.Input.Custom.GetPoint gp4 = new Rhino.Input.Custom.GetPoint();
            //    gp4.SetCommandPrompt(@"Press '1' to set the output gear shaft in the object or '2' to extend the shaft out of the object. Press 'Enter' to continue.");
            //    gp4.AcceptNothing(true);
            //    Rhino.Input.GetResult r4;

            //    OperatingOutputAxleMethod = true;
            //    do
            //    {
            //        r4 = gp4.Get(true);

            //    } while (r4 != Rhino.Input.GetResult.Nothing);
            //    OperatingOutputAxleMethod = false;

            //    #endregion

            //    #region Step 1: configure the position and direction of the kinetic unit

            //    if (eeBreps.Count() == 1)
            //    {
            //        // only one object is selected as the end-effector

            //        #region Prepare the endpoints and transformation for generating the kinetic unit
            //        Brep endeffector = eeBreps.ElementAt(0);

            //        Point3d ptS = skeleton.PointAtNormalizedLength(t1);
            //        Point3d ptE = skeleton.PointAtNormalizedLength(t2);

            //        Point3d tarPt = new Point3d();
            //        Point3d springPosPt = new Point3d();
            //        BoundingBox eeBoundingBox = endeffector.GetBoundingBox(true);
            //        Point3d ee_center = eeBoundingBox.Center;
            //        if (ee_center.DistanceTo(ptS) >= ee_center.DistanceTo(ptE))
            //        {
            //            tarPt = ptE;
            //            springPosPt = ptS;

            //        }
            //        else
            //        {
            //            tarPt = ptS;
            //            springPosPt = ptE;
            //        }

            //        Plane eeDirPlane = new Plane(tarPt, new Vector3d(springPosPt - tarPt));
            //        Point3d dirPt = eeDirPlane.ClosestPoint(ee_center);
            //        Vector3d rawDir = new Vector3d(dirPt - tarPt);

            //        Point3d inputPt = springPosPt;
            //        Point3d outputPt = tarPt; 

            //        // Currently, Kinergy only cares about the transformation along the height
            //        switch (alignment)
            //        {
            //            case 1:
            //                {
            //                    orientationDir = new Vector3d(0, 0, dirPt.Z - tarPt.Z);
            //                    innerHeight = innerHeight - Math.Abs(dirPt.Z - tarPt.Z);
            //                    axisToSkeleton = new Vector3d(tarPt - new Point3d(tarPt.X, 0, 0));
            //                    skeletonToAxis = new Vector3d(new Point3d(tarPt.X, 0, 0) - tarPt);
            //                    rotateAxisToOrientation = Transform.Rotation(Vector3d.XAxis, Vector3d.XAxis, skeleton.PointAtNormalizedLength((t1+t2)/2));
            //                    rotateOrientationToAxis = Transform.Rotation(Vector3d.XAxis, Vector3d.XAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));

            //                }break;
            //            case 2:
            //                {
            //                    orientationDir = new Vector3d(0, 0, dirPt.Z - tarPt.Z);
            //                    innerHeight = innerHeight - Math.Abs(dirPt.Z - tarPt.Z);
            //                    axisToSkeleton = new Vector3d(tarPt - new Point3d(0, tarPt.Y, 0));
            //                    skeletonToAxis = new Vector3d(new Point3d(0, tarPt.Y, 0) - tarPt);
            //                    rotateAxisToOrientation = Transform.Rotation(Vector3d.XAxis, Vector3d.YAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));
            //                    rotateOrientationToAxis = Transform.Rotation(Vector3d.YAxis, Vector3d.XAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));

            //                }
            //                break;
            //            case 3:
            //                {
            //                    orientationDir = new Vector3d(0, dirPt.Y - tarPt.Y, 0);
            //                    innerHeight = innerHeight - Math.Abs(dirPt.Y - tarPt.Y);
            //                    axisToSkeleton = new Vector3d(tarPt - new Point3d(0, 0, tarPt.Z));
            //                    skeletonToAxis = new Vector3d(new Point3d(0, 0, tarPt.Z) - tarPt);
            //                    rotateAxisToOrientation = Transform.Rotation(Vector3d.XAxis, Vector3d.ZAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));
            //                    rotateOrientationToAxis = Transform.Rotation(Vector3d.ZAxis, Vector3d.XAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));

            //                }
            //                break;
            //            default:break;
            //        }

            //        #endregion

            //        #region Parse energy and the speed

            //        speed = speedLevel;
            //        // Parse the energy to 0.1-1
            //        energy = (energyLevel + 1) / 10;

            //        #endregion

            //        #region Create an instance of Continuous Rotation class

            //        myDoc.Objects.Hide(selObjId, true);

            //        motion = new ContinuousRotation(model, direction, energy, speed, energyChargingMethod, innerCavity);      // the second argument represents if the skeleton is curved

            //        #endregion

            //        #region Using orientationDir and tarPt as the axis of the last gear

            //        transSkeleton = Transform.Translation(skeletonToAxis);
            //        transSkeletonBack = Transform.Translation(axisToSkeleton);
            //        offsetTranslation = Transform.Translation(orientationDir);

            //        inputPt.Transform(transSkeleton);
            //        inputPt.Transform(rotateOrientationToAxis);
            //        outputPt.Transform(transSkeleton);
            //        outputPt.Transform(rotateOrientationToAxis);
            //        innerCavity.Transform(transSkeleton);
            //        innerCavity.Transform(rotateOrientationToAxis);

            //        double xEnd = outputPt.X;

            //        Brep innerCavityBrep = innerCavity.DuplicateBrep();
            //        double xSpaceEnd = innerCavityBrep.GetBoundingBox(true).Max.X;

            //        #region old code
            //        //// Transform from the current orientation and direction to X axis
            //        //Point3d startPoint = springPosPt;

            //        //Transform dirToXRotation = Transform.Rotation(direction, new Vector3d(1, 0, 0), startPoint);
            //        //Point3d projectedSpringPosPt = new Point3d(startPoint.X, 0, 0);
            //        //Transform dirToXTranlation = Transform.Translation(new Vector3d(projectedSpringPosPt - startPoint));

            //        //// Transform back from X axis to the current kinetic unit orientation
            //        //dirToXTranlationBack = Transform.Translation(new Vector3d(startPoint - projectedSpringPosPt));
            //        //dirToXRotationBack = Transform.Rotation(new Vector3d(1, 0, 0), direction, startPoint);

            //        //// Last step, rotate back to the pose of the kinetic unit
            //        //Vector3d originalYVector = new Vector3d(0, 1, 0);
            //        //originalYVector.Transform(dirToXTranlationBack);
            //        //originalYVector.Transform(dirToXRotationBack);
            //        ////yToPoseTrans = Transform.Rotation(originalYVector, orientationDir, startPoint);
            //        //yToPoseTrans = Transform.Identity;

            //        //// Start transform
            //        //startPoint.Transform(dirToXRotation);
            //        //startPoint.Transform(dirToXTranlation);

            //        //Point3d endPt = new Point3d(tarPt);
            //        //endPt.Transform(dirToXRotation);
            //        //endPt.Transform(dirToXTranlation);

            //        //double xEnd = endPt.X;
            //        //Brep modelDup = model.DuplicateBrep();
            //        //modelDup.Transform(dirToXRotation);
            //        //modelDup.Transform(dirToXTranlation);


            //        //double outDiameter = Math.Abs(modelDup.GetBoundingBox(true).Max.Z - modelDup.GetBoundingBox(true).Min.Z);
            //        ////double outDiameter = Double.MaxValue;

            //        ////foreach(var v in modelDup.Vertices)
            //        ////{
            //        ////    if (Math.Abs(v.Location.Z) < outDiameter / 2)
            //        ////        outDiameter = Math.Abs(v.Location.Z) * 2;
            //        ////}
            //        //double totalThickness = Math.Abs(modelDup.GetBoundingBox(true).Max.Y - modelDup.GetBoundingBox(true).Min.Y);
            //        ////double totalThickness = Double.MaxValue;

            //        ////foreach(var v in modelDup.Vertices)
            //        ////{
            //        ////    if (Math.Abs(v.Location.Y) < totalThickness / 2)
            //        ////        totalThickness = Math.Abs(v.Location.Y) * 2;
            //        ////}
            //        //Brep innerCavityBrep = innerCavity.DuplicateBrep();
            //        //innerCavityBrep.Transform(dirToXTranlation);
            //        //innerCavityBrep.Transform(dirToXRotation);
            //        //double xSpaceEnd = innerCavityBrep.GetBoundingBox(true).Max.X;

            //        #endregion

            //        motion.ConstructGearTrain(inputPt, xEnd, innerHeight, innerDepth, xSpaceEnd, outputAxle,
            //            transSkeletonBack, rotateAxisToOrientation, offsetTranslation);
            //        motion.ConstructSpring(inputPt, xEnd, innerHeight, innerDepth, xSpaceEnd, outputAxle,
            //            transSkeletonBack, rotateAxisToOrientation, offsetTranslation);

            //        foreach (var obj in myDoc.Objects)
            //        {
            //            Guid tempID = obj.Id;
            //            if (!endEffectorIDs.Contains(tempID))
            //            {
            //                ObjRef currObj = new ObjRef(tempID);

            //                Brep tempBrep = currObj.Brep();

            //                bool isFind = false;
            //                foreach (Entity en in motion.EntityList)
            //                {
            //                    if (en.Model == tempBrep)
            //                    {
            //                        isFind = true;
            //                        break;
            //                    }

            //                }

            //                if (!isFind)
            //                {
            //                    motion.EntityList.Add(new Shape(tempBrep, false, ""));
            //                }
            //            }
            //        }

            //        #endregion
            //    }
            //    else
            //    {
            //        // multiple objects are selected as the end-effectors

            //        #region Prepare the endpoints

            //        Point3d ptS = skeleton.PointAtNormalizedLength(t1);
            //        Point3d ptE = skeleton.PointAtNormalizedLength(t2);

            //        Point3d tarPt = new Point3d();
            //        Point3d springPosPt = new Point3d();

            //        Point3d eeCenter = new Point3d(0,0,0);
            //        Brep eeBrepAll = new Brep();

            //        List<Point3d> brepCenters = new List<Point3d>();
            //        Point3d eeCen1 = new Point3d();
            //        Point3d eeCen2 = new Point3d();

            //        foreach (Brep b in eeBreps)
            //        {
            //            BoundingBox eeBoundingBox = b.GetBoundingBox(true);
            //            Point3d ee_center = eeBoundingBox.Center;

            //            brepCenters.Add(ee_center);

            //            //eeBrepAll.Append(b);
            //            eeBrepAll = b;

            //            eeCenter = eeCenter + ee_center;
            //        }
            //        eeCen1 = brepCenters.ElementAt(0);
            //        eeCen2 = brepCenters.ElementAt(1);

            //        eeCenter = eeCenter / eeBreps.Count();

            //        if (eeCenter.DistanceTo(ptS) >= eeCenter.DistanceTo(ptE))
            //        {
            //            tarPt = ptE;
            //            springPosPt = ptS;
            //        }
            //        else
            //        {
            //            tarPt = ptS;
            //            springPosPt = ptE;
            //        }

            //        #endregion

            //        #region transform the model so that the skeleton is on the X axis

            //        //Plane eeDirPlane = new Plane(tarPt, new Vector3d(springPosPt - tarPt));
            //        //Point3d dirPt = eeDirPlane.ClosestPoint(eeCenter);
            //        //Vector3d rawDir = new Vector3d(dirPt - tarPt);

            //        Point3d inputPt = springPosPt;
            //        Point3d outputPt = tarPt;

            //        // Currently, Kinergy only cares about the transformation along the height
            //        switch (alignment)
            //        {
            //            case 1:
            //                {
            //                    // X axis

            //                    // first rotate the entire model from the current aligned axis to X axis
            //                    rotateAxisToOrientation = Transform.Rotation(Vector3d.XAxis, Vector3d.XAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));
            //                    rotateOrientationToAxis = Transform.Rotation(Vector3d.XAxis, Vector3d.XAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));

            //                    // second, rotate the model so that the generated gears only move along Z axis
            //                    eeCen1.Transform(rotateOrientationToAxis);
            //                    eeCen2.Transform(rotateOrientationToAxis);
            //                    double xDiff = Math.Abs(eeCen1.X - eeCen2.X);
            //                    double yDiff = Math.Abs(eeCen1.Y - eeCen2.Y);
            //                    double zDiff = Math.Abs(eeCen1.Z - eeCen2.Z);

            //                    if (xDiff >= yDiff && xDiff >= zDiff)
            //                    {
            //                        // impossible
            //                    }
            //                    else if (yDiff >= xDiff && yDiff >= zDiff)
            //                    {
            //                        // rotate the end-effector direction (Y axis) to Y axis
            //                        eeDirectionToYAxis = Transform.Rotation(Vector3d.YAxis, Vector3d.YAxis, (eeCen1 + eeCen2) / 2);
            //                        eeYAxisToDirection = Transform.Rotation(Vector3d.YAxis, Vector3d.YAxis, (eeCen1 + eeCen2) / 2);
            //                    }
            //                    else
            //                    {
            //                        // rotate the end-effector direction (Z axis) to Y axis
            //                        eeDirectionToYAxis = Transform.Rotation(-Vector3d.ZAxis, Vector3d.YAxis, (eeCen1 + eeCen2) / 2);
            //                        eeYAxisToDirection = Transform.Rotation(Vector3d.YAxis, -Vector3d.ZAxis, (eeCen1 + eeCen2) / 2);
            //                    }

            //                    // third, transform the inputPt, outputPt, model, innerCavity, cen1, cen2 with two rotation matrix
            //                    eeCen1.Transform(eeDirectionToYAxis);
            //                    eeCen2.Transform(eeDirectionToYAxis);
            //                    inputPt.Transform(rotateOrientationToAxis);
            //                    inputPt.Transform(eeDirectionToYAxis);
            //                    outputPt.Transform(rotateOrientationToAxis);
            //                    outputPt.Transform(eeDirectionToYAxis);

            //                    model.Transform(rotateOrientationToAxis);
            //                    model.Transform(eeDirectionToYAxis);
            //                    innerCavity.Transform(rotateOrientationToAxis);
            //                    innerCavity.Transform(eeDirectionToYAxis);

            //                    // only transform the skeleton here
            //                    Point3d newCen = (eeCen1 + eeCen2) / 2;
            //                    orientationDir = new Vector3d(0, 0, newCen.Z - outputPt.Z);
            //                    offsetTranslation = Transform.Translation(orientationDir);
            //                    //innerHeight = innerHeight - Math.Abs(newCen.Z - outputPt.Z);
            //                    inputPt.Transform(offsetTranslation);
            //                    outputPt.Transform(offsetTranslation);

            //                    // transform all the skeleton, model, and innerCavity to align the skeleton on the X axis
            //                    axisToSkeleton = new Vector3d(outputPt - new Point3d(outputPt.X, 0, 0));
            //                    skeletonToAxis = new Vector3d(new Point3d(outputPt.X, 0, 0) - outputPt);
            //                    transSkeleton = Transform.Translation(skeletonToAxis);
            //                    transSkeletonBack = Transform.Translation(axisToSkeleton);

            //                    outputPt.Transform(transSkeleton);
            //                    inputPt.Transform(transSkeleton);
            //                    model.Transform(transSkeleton);
            //                    innerCavity.Transform(transSkeleton);
            //                    eeCen1.Transform(transSkeleton);
            //                    eeCen2.Transform(transSkeleton);
            //                    newCen.Transform(transSkeleton);

            //                }
            //                break;
            //            case 2:
            //                {
            //                    // Y axis

            //                    // first rotate the entire model from the current aligned axis to X axis 
            //                    rotateAxisToOrientation = Transform.Rotation(Vector3d.XAxis, Vector3d.YAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));
            //                    rotateOrientationToAxis = Transform.Rotation(Vector3d.YAxis, Vector3d.XAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));

            //                    // second, rotate the model so that the generated gears only move along Z axis
            //                    eeCen1.Transform(rotateOrientationToAxis);
            //                    eeCen2.Transform(rotateOrientationToAxis);
            //                    double xDiff = Math.Abs(eeCen1.X - eeCen2.X);
            //                    double yDiff = Math.Abs(eeCen1.Y - eeCen2.Y);
            //                    double zDiff = Math.Abs(eeCen1.Z - eeCen2.Z);

            //                    if (xDiff >= yDiff && xDiff >= zDiff)
            //                    {
            //                        // impossible
            //                    }
            //                    else if (yDiff >= xDiff && yDiff >= zDiff)
            //                    {
            //                        // rotate the end-effector direction (Y axis) to Y axis
            //                        eeDirectionToYAxis = Transform.Rotation(Vector3d.YAxis, Vector3d.YAxis, (eeCen1 + eeCen2) / 2);
            //                        eeYAxisToDirection = Transform.Rotation(Vector3d.YAxis, Vector3d.YAxis, (eeCen1 + eeCen2) / 2);
            //                    }
            //                    else
            //                    {
            //                        // rotate the end-effector direction (Z axis) to Y axis
            //                        eeDirectionToYAxis = Transform.Rotation(-Vector3d.ZAxis, Vector3d.YAxis, (eeCen1 + eeCen2) / 2);
            //                        eeYAxisToDirection = Transform.Rotation(Vector3d.YAxis, -Vector3d.ZAxis, (eeCen1 + eeCen2) / 2);
            //                    }

            //                    // third, transform the inputPt, outputPt, model, innerCavity, cen1, cen2 with two rotation matrix
            //                    eeCen1.Transform(eeDirectionToYAxis);
            //                    eeCen2.Transform(eeDirectionToYAxis);
            //                    inputPt.Transform(rotateOrientationToAxis);
            //                    inputPt.Transform(eeDirectionToYAxis);
            //                    outputPt.Transform(rotateOrientationToAxis);
            //                    outputPt.Transform(eeDirectionToYAxis);

            //                    model.Transform(rotateOrientationToAxis);
            //                    model.Transform(eeDirectionToYAxis);
            //                    innerCavity.Transform(rotateOrientationToAxis);
            //                    innerCavity.Transform(eeDirectionToYAxis);

            //                    // only transform the skeleton here
            //                    Point3d newCen = (eeCen1 + eeCen2) / 2;
            //                    orientationDir = new Vector3d(0, 0, newCen.Z - outputPt.Z);
            //                    offsetTranslation = Transform.Translation(orientationDir);
            //                    //innerHeight = innerHeight - Math.Abs(newCen.Z - outputPt.Z);
            //                    inputPt.Transform(offsetTranslation);
            //                    outputPt.Transform(offsetTranslation);

            //                    // transform all the skeleton, model, and innerCavity to align the skeleton on the X axis
            //                    axisToSkeleton = new Vector3d(outputPt - new Point3d(outputPt.X, 0, 0));
            //                    skeletonToAxis = new Vector3d(new Point3d(outputPt.X, 0, 0) - outputPt);
            //                    transSkeleton = Transform.Translation(skeletonToAxis);
            //                    transSkeletonBack = Transform.Translation(axisToSkeleton);

            //                    outputPt.Transform(transSkeleton);
            //                    inputPt.Transform(transSkeleton);
            //                    model.Transform(transSkeleton);
            //                    innerCavity.Transform(transSkeleton);
            //                    eeCen1.Transform(transSkeleton);
            //                    eeCen2.Transform(transSkeleton);
            //                    newCen.Transform(transSkeleton);

            //                }
            //                break;
            //            case 3:
            //                {
            //                    // Z axis

            //                    // first rotate the entire model from the current aligned axis to X axis 
            //                    rotateAxisToOrientation = Transform.Rotation(Vector3d.XAxis, Vector3d.ZAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));
            //                    rotateOrientationToAxis = Transform.Rotation(Vector3d.ZAxis, Vector3d.XAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));

            //                    // second, rotate the model so that the generated gears only move along Z axis
            //                    eeCen1.Transform(rotateOrientationToAxis);
            //                    eeCen2.Transform(rotateOrientationToAxis);
            //                    double xDiff = Math.Abs(eeCen1.X - eeCen2.X);
            //                    double yDiff = Math.Abs(eeCen1.Y - eeCen2.Y);
            //                    double zDiff = Math.Abs(eeCen1.Z - eeCen2.Z);

            //                    if (xDiff >= yDiff && xDiff >= zDiff)
            //                    {
            //                        // impossible
            //                    }
            //                    else if (yDiff >= xDiff && yDiff >= zDiff)
            //                    {
            //                        // rotate the end-effector direction (Y axis) to Y axis
            //                        eeDirectionToYAxis = Transform.Rotation(Vector3d.YAxis, Vector3d.YAxis, (eeCen1 + eeCen2) / 2);
            //                        eeYAxisToDirection = Transform.Rotation(Vector3d.YAxis, Vector3d.YAxis, (eeCen1 + eeCen2) / 2);
            //                    }
            //                    else
            //                    {
            //                        // rotate the end-effector direction (Z axis) to Y axis
            //                        eeDirectionToYAxis = Transform.Rotation(-Vector3d.ZAxis, Vector3d.YAxis, (eeCen1 + eeCen2) / 2);
            //                        eeYAxisToDirection = Transform.Rotation(Vector3d.YAxis, -Vector3d.ZAxis, (eeCen1 + eeCen2) / 2);
            //                    }

            //                    // third, transform the inputPt, outputPt, model, innerCavity, cen1, cen2 with two rotation matrix
            //                    eeCen1.Transform(eeDirectionToYAxis);
            //                    eeCen2.Transform(eeDirectionToYAxis);
            //                    inputPt.Transform(rotateOrientationToAxis);
            //                    inputPt.Transform(eeDirectionToYAxis);
            //                    outputPt.Transform(rotateOrientationToAxis);
            //                    outputPt.Transform(eeDirectionToYAxis);

            //                    model.Transform(rotateOrientationToAxis);
            //                    model.Transform(eeDirectionToYAxis);
            //                    innerCavity.Transform(rotateOrientationToAxis);
            //                    innerCavity.Transform(eeDirectionToYAxis);

            //                    // only transform the skeleton here
            //                    Point3d newCen = (eeCen1 + eeCen2) / 2;
            //                    orientationDir = new Vector3d(0, 0, newCen.Z - outputPt.Z);
            //                    offsetTranslation = Transform.Translation(orientationDir);
            //                    //innerHeight = innerHeight - Math.Abs(newCen.Z - outputPt.Z);
            //                    inputPt.Transform(offsetTranslation);
            //                    outputPt.Transform(offsetTranslation);

            //                    // transform all the skeleton, model, and innerCavity to align the skeleton on the X axis
            //                    axisToSkeleton = new Vector3d(outputPt - new Point3d(outputPt.X, 0, 0));
            //                    skeletonToAxis = new Vector3d(new Point3d(outputPt.X, 0, 0) - outputPt);
            //                    transSkeleton = Transform.Translation(skeletonToAxis);
            //                    transSkeletonBack = Transform.Translation(axisToSkeleton);

            //                    outputPt.Transform(transSkeleton);
            //                    inputPt.Transform(transSkeleton);
            //                    model.Transform(transSkeleton);
            //                    innerCavity.Transform(transSkeleton);
            //                    eeCen1.Transform(transSkeleton);
            //                    eeCen2.Transform(transSkeleton);
            //                    newCen.Transform(transSkeleton);

            //                    //orientationDir = new Vector3d(0, dirPt.Y - tarPt.Y, 0);
            //                    //innerHeight = innerHeight - Math.Abs(dirPt.Y - tarPt.Y);
            //                    //axisToSkeleton = new Vector3d(tarPt - new Point3d(0, 0, tarPt.Z));
            //                    //skeletonToAxis = new Vector3d(new Point3d(0, 0, tarPt.Z) - tarPt);
            //                    //rotateAxisToOrientation = Transform.Rotation(Vector3d.XAxis, Vector3d.ZAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));
            //                    //rotateOrientationToAxis = Transform.Rotation(Vector3d.ZAxis, Vector3d.XAxis, skeleton.PointAtNormalizedLength((t1 + t2) / 2));
            //                }
            //                break;
            //            default: break;
            //        }

            //        #endregion

            //        #region Set up the output gear position and the max X on the X axis

            //        //inputPt.Transform(transSkeleton);
            //        //inputPt.Transform(rotateOrientationToAxis);
            //        //outputPt.Transform(transSkeleton);
            //        //outputPt.Transform(rotateOrientationToAxis);
            //        //innerCavity.Transform(transSkeleton);
            //        //innerCavity.Transform(rotateOrientationToAxis);
            //        //eeCen1.Transform(transSkeleton);
            //        //eeCen1.Transform(rotateOrientationToAxis);
            //        //eeCen2.Transform(transSkeleton);
            //        //eeCen2.Transform(rotateOrientationToAxis);

            //        double xEnd = outputPt.X;

            //        Brep innerCavityBrep = innerCavity.DuplicateBrep();
            //        double xSpaceEnd = innerCavityBrep.GetBoundingBox(true).Max.X;

            //        #endregion


            //        #region Parse energy and the speed

            //        speed = speedLevel;
            //        // Parse the energy to 0.1-1
            //        energy = (energyLevel + 1) / 10;

            //        #endregion

            //        #region Create an instance of Continuous Rotation class

            //        myDoc.Objects.Hide(selObjId, true);
            //        motion = new ContinuousRotation(model, direction, energy, speed, energyChargingMethod, innerCavity);      // the second argument represents if the skeleton is curved

            //        #endregion

            //        Line dir = new Line();
            //        dir = new Line(eeCen1, eeCen2);

            //        if (dir != null)
            //        {
            //            eeBrepAll.Transform(rotateOrientationToAxis);
            //            eeBrepAll.Transform(eeDirectionToYAxis);
            //            eeBrepAll.Transform(transSkeleton);
            //            Wheel eeBrepWheel = new Wheel(eeBrepAll, dir, motion);
            //            motion.DrivenPart = eeBrepWheel;
            //        }

            //        #region old code
            //        //Point3d startPoint = springPosPt;

            //        //Transform dirToXRotation = Transform.Rotation(direction, new Vector3d(1, 0, 0), startPoint);
            //        //Point3d projectedSpringPosPt = new Point3d(startPoint.X, 0, 0);
            //        //Transform dirToXTranlation = Transform.Translation(new Vector3d(projectedSpringPosPt - startPoint));

            //        //// Transform back from X axis to the current kinetic unit orientation
            //        //dirToXTranlationBack = Transform.Translation(new Vector3d(startPoint - projectedSpringPosPt));
            //        //dirToXRotationBack = Transform.Rotation(new Vector3d(1, 0, 0), direction, startPoint);

            //        //// Last step, rotate back to the pose of the kinetic unit
            //        //Vector3d originalYVector = new Vector3d(0, 1, 0);
            //        //originalYVector.Transform(dirToXTranlationBack);
            //        //originalYVector.Transform(dirToXRotationBack);
            //        //yToPoseTrans = Transform.Translation(orientationDir);

            //        //// Start transform
            //        //startPoint.Transform(dirToXRotation);
            //        //startPoint.Transform(dirToXTranlation);

            //        //Point3d endPt = new Point3d(tarPt);
            //        //endPt.Transform(dirToXRotation);
            //        //endPt.Transform(dirToXTranlation);

            //        //double xEnd = endPt.X;
            //        //Brep modelDup = model.DuplicateBrep();
            //        //modelDup.Transform(dirToXRotation);
            //        //modelDup.Transform(dirToXTranlation);


            //        //double outDiameter;
            //        //double totalThickness;

            //        //if (directionType == 3)
            //        //    outDiameter = Math.Abs(2 * (modelDup.GetBoundingBox(true).Center.Z - translatingDis - modelDup.GetBoundingBox(true).Min.Z));
            //        //else
            //        //    outDiameter = Math.Abs(2 * (modelDup.GetBoundingBox(true).Center.Z - modelDup.GetBoundingBox(true).Min.Z));

            //        //if (directionType == 2)
            //        //    totalThickness = Math.Abs(modelDup.GetBoundingBox(true).Max.Y - modelDup.GetBoundingBox(true).Min.Y - translatingDis);
            //        //else
            //        //    totalThickness = Math.Abs(modelDup.GetBoundingBox(true).Max.Y - modelDup.GetBoundingBox(true).Min.Y);
            //        ////double outDiameter = Double.MaxValue;

            //        //Brep innerCavityBrep = innerCavity.DuplicateBrep();
            //        //innerCavityBrep.Transform(dirToXTranlation);
            //        //innerCavityBrep.Transform(dirToXRotation);

            //        //double xSpaceEnd = innerCavityBrep.GetBoundingBox(true).Max.X;

            //        #endregion

            //        motion.ConstructGearTrain(inputPt, xEnd, innerHeight, innerDepth, xSpaceEnd, outputAxle,
            //            transSkeletonBack, eeYAxisToDirection, rotateAxisToOrientation);
            //        motion.ConstructSpring(inputPt, xEnd, innerHeight, innerDepth, xSpaceEnd, outputAxle,
            //            transSkeletonBack, eeYAxisToDirection, rotateAxisToOrientation);

            //        //motion.ConstructGearTrain(startPoint, xEnd, outDiameter, totalThickness, xSpaceEnd, outputAxle,
            //        //    dirToXTranlationBack, dirToXRotationBack, yToPoseTrans);
            //        //motion.ConstructSpring(startPoint, xEnd, outDiameter, totalThickness, xSpaceEnd, outputAxle,
            //        //    dirToXTranlationBack, dirToXRotationBack, yToPoseTrans);

            //        foreach (var obj in myDoc.Objects)
            //        {
            //            Guid tempID = obj.Id;
            //            ObjRef currObj = new ObjRef(tempID);

            //            Brep tempBrep = currObj.Brep();

            //            bool isFind = false;
            //            foreach (Entity en in motion.EntityList)
            //            {
            //                if (en.Model == tempBrep)
            //                {
            //                    isFind = true;
            //                    break;
            //                }

            //            }

            //            if (!isFind)
            //            {
            //                motion.EntityList.Add(new Shape(tempBrep, false, ""));
            //            }
            //        }


            //    }
            //    #endregion

            //}

            #endregion

            if (toAddLock)
            {
                if (motion != null)
                    motion.ConstructLocks(transSkeletonBack, rotateAxisToOrientation, offsetTranslation);
            }

            if (toPreview)
            {

            }

            if (toAdjustParam)
            {
                if (motion != null)
                    // Reconstruct spiral and the gear train
                    motion.AdjustParameter(speedLevel, roundLevel);
            }


            if (toSetMotionControl)
            {

            }

            if (toSetEEPos)
            {

            }

            if (toSetAxisDir)
            {

            }


            if (toRemoveLock)
            {

            }

          
            

            DA.SetData(0, motion);
            //DA.SetData(1, model);
            if (motion == null)
                DA.SetDataList(1, null);
            else
                DA.SetDataList(1, motion.GetModel());
            DA.SetData(2, toPreview);
        }

        private void IndicatorSelectPt_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (xIndicatorID != Guid.Empty && yIndicatorID != Guid.Empty && zIndicatorID != Guid.Empty)
            {
                Brep xIndicatorBrep = (Brep)myDoc.Objects.FindId(xIndicatorID).Geometry;
                Brep yIndicatorBrep = (Brep)myDoc.Objects.FindId(yIndicatorID).Geometry;
                Brep zIndicatorBrep = (Brep)myDoc.Objects.FindId(zIndicatorID).Geometry;

                Point3d xClosestPt = xIndicatorBrep.ClosestPoint(e.Point);
                Point3d yClosestPt = yIndicatorBrep.ClosestPoint(e.Point);
                Point3d zClosestPt = zIndicatorBrep.ClosestPoint(e.Point);

                if (xClosestPt.DistanceTo(e.Point) <= yClosestPt.DistanceTo(e.Point) &&
                   xClosestPt.DistanceTo(e.Point) <= zClosestPt.DistanceTo(e.Point))
                {
                    alignment = 1;
                    alignmentPt = xClosestPt;
                }
                else if (yClosestPt.DistanceTo(e.Point) <= xClosestPt.DistanceTo(e.Point) &&
                    yClosestPt.DistanceTo(e.Point) <= zClosestPt.DistanceTo(e.Point))
                {
                    alignment = 2;
                    alignmentPt = yClosestPt;
                }
                else if (zClosestPt.DistanceTo(e.Point) <= yClosestPt.DistanceTo(e.Point) &&
                    zClosestPt.DistanceTo(e.Point) <= xClosestPt.DistanceTo(e.Point))
                {
                    alignment = 3;
                    alignmentPt = zClosestPt;
                }
            }
        }

        private void IndicatorSelectPt_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (xIndicatorID != Guid.Empty && yIndicatorID != Guid.Empty && zIndicatorID != Guid.Empty)
            {
                Brep xIndicatorBrep = (Brep)myDoc.Objects.FindId(xIndicatorID).Geometry;
                Brep yIndicatorBrep = (Brep)myDoc.Objects.FindId(yIndicatorID).Geometry;
                Brep zIndicatorBrep = (Brep)myDoc.Objects.FindId(zIndicatorID).Geometry;

                Point3d xClosestPt = xIndicatorBrep.ClosestPoint(e.Point);
                Point3d yClosestPt = yIndicatorBrep.ClosestPoint(e.Point);
                Point3d zClosestPt = zIndicatorBrep.ClosestPoint(e.Point);

                if (xClosestPt.DistanceTo(e.Point) <= yClosestPt.DistanceTo(e.Point) &&
                   xClosestPt.DistanceTo(e.Point) <= zClosestPt.DistanceTo(e.Point))
                {
                    alignment = 1;
                    alignmentPt = xClosestPt;
                }
                else if (yClosestPt.DistanceTo(e.Point) <= xClosestPt.DistanceTo(e.Point) &&
                    yClosestPt.DistanceTo(e.Point) <= zClosestPt.DistanceTo(e.Point))
                {
                    alignment = 2;
                    alignmentPt = yClosestPt;
                }
                else if (zClosestPt.DistanceTo(e.Point) <= yClosestPt.DistanceTo(e.Point) &&
                    zClosestPt.DistanceTo(e.Point) <= xClosestPt.DistanceTo(e.Point))
                {
                    alignment = 3;
                    alignmentPt = zClosestPt;
                }
            }

        }

        private void RhinoApp_KeyboardEvent3(int key)
        {
            if (!multipleSelections)
                return;
        }

        private void RhinoApp_KeyboardEvent2(int key)
        {
            if (!OperatingOutputAxleMethod)
                return;
            if (key == 49) //1
            {
                outputAxle = 1;
            }
            else if (key == 50)// 2
            {
                outputAxle = 2;
            }

        }

        private void RhinoApp_KeyboardEvent1(int key)
        {
            if (!OperatingEnergyChargingMethod)
                return;

            if (key == 49) //1
            {
                energyChargingMethod = 1;
            }
            else if (key == 50)// 2
            {
                energyChargingMethod = 2;
            }
            
        }

        private void Gp2_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (selected == Guid.Empty)
                return;
            // e.point is the currrent position of the 3D point in the Rhino scene
            double t = 0, tn = 0;
            skeleton.ClosestPoint(e.Point, out t);
            skeleton.NormalizedLengthParameter(t, out tn);
            if (tn < 0)
                tn = 0;
            if (tn > 1)
                tn = 1;
            if (selected == guid1)
            {
                //calculate where is the mouse and change t
                if (Math.Abs(t1 - tn) > 0.01)
                {
                    //move and update t1
                    Transform m = Transform.Translation(skeletonVec * (tn - t1));
                    RhinoDoc.ActiveDoc.Objects.Transform(guid1, m, true);
                    pl1.Transform(m);
                    t1 = tn;
                }

            }
            if (selected == guid2)
            {
                //calculate where is the mouse and change t
                if (Math.Abs(t2 - tn) > 0.01)
                {
                    //move and update t1
                    Transform m = Transform.Translation(skeletonVec * (tn - t2));
                    RhinoDoc.ActiveDoc.Objects.Transform(guid2, m, true);
                    pl2.Transform(m);
                    t2 = tn;
                }

            }
        }

        private void Gp2_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (selected != Guid.Empty)
                selected = Guid.Empty;
            else
            {
                var p = e.Point;
                double dis1 = Math.Abs(pl1.DistanceTo(p)), dis2 = Math.Abs(pl2.DistanceTo(p));

                List<double> distances = new List<double> { dis1, dis2 };
                double min = distances.Min();
                if (min > 5)
                { return; }
                else if (min == dis1)
                    selected = guid1;
                else if (min == dis2)
                    selected = guid2;
            }
        }

        /// <summary>
        /// Generate the 3D arrows that indicate X (red), Y (green), and Z (blue) axis
        /// </summary>
        /// <param name="myDoc">Rhino Document</param>
        private void GenerateCoordinateIndicators(RhinoDoc myDoc, Brep model)
        {
            #region color definitions
            int index_blue = myDoc.Materials.Add();
            Rhino.DocObjects.Material mat_blue = myDoc.Materials[index_blue];
            mat_blue.DiffuseColor = System.Drawing.Color.FromArgb(0, 0, 255);
            mat_blue.CommitChanges();

            Rhino.DocObjects.ObjectAttributes blue_attributes = new Rhino.DocObjects.ObjectAttributes();
            blue_attributes.MaterialIndex = index_blue;
            blue_attributes.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            blue_attributes.ObjectColor = Color.FromArgb(0, 0, 255);
            blue_attributes.ColorSource = ObjectColorSource.ColorFromObject;

            int index_red = myDoc.Materials.Add();
            Rhino.DocObjects.Material mat_red = myDoc.Materials[index_red];
            mat_red.DiffuseColor = System.Drawing.Color.FromArgb(255, 0, 0);
            mat_red.CommitChanges();

            Rhino.DocObjects.ObjectAttributes red_attributes = new Rhino.DocObjects.ObjectAttributes();
            red_attributes.MaterialIndex = index_red;
            red_attributes.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            red_attributes.ObjectColor = Color.FromArgb(255, 0, 0);
            red_attributes.ColorSource = ObjectColorSource.ColorFromObject;

            int index_green = myDoc.Materials.Add();
            Rhino.DocObjects.Material mat_green = myDoc.Materials[index_green];
            mat_green.DiffuseColor = System.Drawing.Color.FromArgb(0, 255, 0);
            mat_green.CommitChanges();

            Rhino.DocObjects.ObjectAttributes green_attributes = new Rhino.DocObjects.ObjectAttributes();
            green_attributes.MaterialIndex = index_green;
            green_attributes.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            green_attributes.ObjectColor = Color.FromArgb(0, 255, 0);
            green_attributes.ColorSource = ObjectColorSource.ColorFromObject;

            #endregion

            double cyl_radius = 1;
            double cyl_len = 50;
            double cone_len = 5;
            double cone_radius = 2.5;

            BoundingBox b_model = model.GetBoundingBox(true);
            Point3d cen_model = b_model.Center;

            #region add the X axis indicator

            Vector3d x_vec = new Vector3d(1, 0, 0);
            Line x_line = new Line(cen_model, cen_model + x_vec * cyl_len);
            Curve x_cur = x_line.ToNurbsCurve();
            Brep x_cyl_brep = Brep.CreatePipe(x_cur, cyl_radius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Plane x_cone_pln = new Plane(cen_model + x_vec * (cyl_len+cone_len-0.5), x_vec);
            Cone x_cone = new Cone(x_cone_pln, -cone_len, cone_radius);
            Brep x_cone_brep = Brep.CreateFromCone(x_cone, true);


            Brep x_indicator = Brep.CreateBooleanUnion(new List<Brep> { x_cyl_brep, x_cone_brep }, myDoc.ModelAbsoluteTolerance)[0];
            xIndicatorID = myDoc.Objects.AddBrep(x_indicator, red_attributes);

            #endregion

            #region add the Y axis indicator

            Vector3d y_vec = new Vector3d(0, 1, 0);
            Line y_line = new Line(cen_model, cen_model + y_vec * cyl_len);
            Curve y_cur = y_line.ToNurbsCurve();
            Brep y_cyl_brep = Brep.CreatePipe(y_cur, cyl_radius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Plane y_cone_pln = new Plane(cen_model + y_vec * (cyl_len+cone_len-0.5), y_vec);
            Cone y_cone = new Cone(y_cone_pln, -cone_len, cone_radius);
            Brep y_cone_brep = Brep.CreateFromCone(y_cone, true);

            Brep y_indicator = Brep.CreateBooleanUnion(new List<Brep> { y_cyl_brep, y_cone_brep }, myDoc.ModelAbsoluteTolerance)[0];
            yIndicatorID = myDoc.Objects.AddBrep(y_indicator, green_attributes);

            #endregion

            #region add the Z axis indicator

            Vector3d z_vec = new Vector3d(0, 0, 1);
            Line z_line = new Line(cen_model, cen_model + z_vec * cyl_len);
            Curve z_cur = z_line.ToNurbsCurve();
            Brep z_cyl_brep = Brep.CreatePipe(z_cur, cyl_radius, false, PipeCapMode.Flat, true, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Plane z_cone_pln = new Plane(cen_model + z_vec * (cyl_len+cone_len-0.5), z_vec);
            Cone z_cone = new Cone(z_cone_pln, -cone_len, cone_radius);
            Brep z_cone_brep = Brep.CreateFromCone(z_cone, true);

            Brep z_indicator = Brep.CreateBooleanUnion(new List<Brep> { z_cyl_brep, z_cone_brep }, myDoc.ModelAbsoluteTolerance)[0];
            zIndicatorID = myDoc.Objects.AddBrep(z_indicator, blue_attributes);

            #endregion
            myDoc.Views.Redraw();
        }
   
        private void GeneratePlanes()
        {
            PlaneGenerated = true;
            //Delete these before generating new ones
            RhinoDoc.ActiveDoc.Objects.Delete(guid1, true);
            RhinoDoc.ActiveDoc.Objects.Delete(guid2, true);

            BoxLike b = new BoxLike(model, v);
            BoundingBox box = b.Bbox;

            Interval xInterval = new Interval(-(box.Max.X - box.Min.X) * 0.6, (box.Max.X - box.Min.X) * 0.6);
            Interval yInterval = new Interval(-(box.Max.Y - box.Min.Y) * 0.6, (box.Max.Y - box.Min.Y) * 0.6);
            Interval zInterval = new Interval(-(box.Max.Z - box.Min.Z) * 0.6, (box.Max.Z - box.Min.Z) * 0.6);
            //box.Transform(b.RotateBack);
            /*Point3d start = box.PointAt(0, 0.5, 0.5);
            Point3d end = box.PointAt(1, 0.5, 0.5);*/ //this doesn't work!

            skeleton = b.Skeleton;
            //skeleton.Transform(b.RotateBack);
            skeletonVec = new Vector3d(skeleton.PointAtEnd) - new Vector3d(skeleton.PointAtStart);
            pl1 = new Plane(skeleton.PointAtNormalizedLength(t1), v);
            pl2 = new Plane(skeleton.PointAtNormalizedLength(t2), v);
            if(alignment == 1)
            {
                s1 = new PlaneSurface(pl1, yInterval, zInterval);
                s2 = new PlaneSurface(pl2, yInterval, zInterval);
            }
            else if(alignment == 2)
            {
                s1 = new PlaneSurface(pl1, zInterval, xInterval);
                s2 = new PlaneSurface(pl2, zInterval, xInterval);
            }
            else if(alignment == 3)
            {
                s1 = new PlaneSurface(pl1, xInterval, yInterval);
                s2 = new PlaneSurface(pl2, xInterval, yInterval);
            }
            else
            {
                s1 = new PlaneSurface(pl1, yInterval, zInterval);
                s2 = new PlaneSurface(pl2, yInterval, zInterval);
            }
            
            guid1 = RhinoDoc.ActiveDoc.Objects.Add(s1);
            guid2 = RhinoDoc.ActiveDoc.Objects.Add(s2);
            RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();
        }


        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("3d575e31-210f-46a0-9d8d-8c5cf62c6a92"); }
        }
    }
}