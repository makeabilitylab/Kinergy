using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy.KineticUnit;
using KinergyUtilities;
using Rhino;
using Rhino.Geometry;
using Rhino.DocObjects;
using Rhino.Input;
using HumanUIforKinergy.KinergyUtilities;
using Kinergy.Geom;
using System.Linq;
using System.Drawing;

namespace InterRotation
{
    public class IntermittentRotationModule : GH_Component
    {
        // Variables that store information
        Brep model;             // the original brep model
        Brep conBrep;           // the Brep that is selected and converted
        Brep innerCavity;
        double t1, t2; // the positoins of start point and end point of the segment on the normalized skeleton
        Curve skeleton;     // skeleton
        int speedLevel;         // value of the strength slide bar
        int stepAngleLevel;   // value of the step angle slide bar
        int strokeLevel;    // value of the stroke number
        int energyLevel;
        Vector3d direction;             // kinetic unit direction
        IntermittentRotation motion;
        List<Arrow> lockDirCandidates;
        Arrow p;

        // Variables used for different functions
        bool lockState;
        double min_wire_diamter;
        double min_coil_num;
        double energy;
        bool isLockSet;
        double speed;
        Guid selObjId;
        List<Guid> toBeBaked;

        // Region selection related variables
        Point3d center = Point3d.Unset;
        Guid guid1, guid2, ArrowCurve;
        bool OperatingArrow = false;
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

        bool dirReverseState = false;
        Guid rotationDirID = Guid.Empty;
        bool isSpringCW = true;
        bool dirControl = false;

        Helpers helperFun;
        
        WarningWin warningwin = new WarningWin();

        Vector3d selectedAxisVector = Vector3d.Unset;

        ObjectAttributes solidAttribute, orangeAttribute, redAttribute, blueAttribute, greenAttribute;
        int selectedAxisIndex;
        Guid convertedPortion;
        List<Brep> brepCut;
        Guid reserveBrepID1;
        Guid reserveBrepID2;
        Point3d motionCtrlPointSelected;
        Guid ee;
        Brep eeModel;
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

        List<Entity> axel_spacer_entities = new List<Entity>();
        List<Gear> gears = new List<Gear>();
        List<Entity> spring_entities = new List<Entity>();
        //List<GearParameter> gear_info = new List<GearParameter>();
        List<Point3d> lockPos = new List<Point3d>();
        bool spiralLockNorm = false;
        Vector3d spiralLockDir = new Vector3d();

        Guid ee1, ee2;
        Brep ee1Model, ee2Model;
        int endEffectorState = 0;//0 for unset, 1 for one ee, 2 for 2 ee;
        Vector3d mainAxis = Vector3d.Unset, perpAxis = Vector3d.Unset,shaftAxis=Vector3d.Unset;

        List<double> gr_list = new List<double>();
        List<GearTrainScheme> gear_schemes = new List<GearTrainScheme>();
        GenevaDrive genevaDrive = null;
        Entity GenevaWheel = null;
        Entity GenevaDrivingWheelWithPin = null;
        Entity GenevaStopper = null;

        Vector3d eeTranslation = Vector3d.Unset;
        Brep socketBrep = null;

        /// <summary>
        /// Initializes a new instance of the IntermittentRotationModule class.
        /// </summary>
        public IntermittentRotationModule()
          : base("IntermittentRotationModule", "IRModule",
              "The kinetic unit for intermittent rotation",
              "Kinergy", "KineticUnits")
        {
            model = null;
            conBrep = new Brep();
            innerCavity = new Brep();
            t1 = 0;
            t2 = 1;
            skeleton = null;
            speedLevel = 5;
            stepAngleLevel = 5;
            strokeLevel = 5;
            energyLevel = 5;
            direction = new Vector3d();
            motion = null;
            lockDirCandidates = new List<Arrow>();
            p = null;

            lockState = false;
            min_wire_diamter = 2.8;
            min_coil_num = 3;
            energy = 0.5;
            arrowScale = 0;
            isLockSet = false;
            selObjId = Guid.Empty;
            toBeBaked = new List<Guid>();
            speed = 0;

            myDoc = RhinoDoc.ActiveDoc;
            testBodySelBtn = false;
            testMotionControlPosSetBtn = false;
            testEEPosSetBtn = false;
            testMotionAxisDirSetBtn = false;
            testPreBtn = false;
            testBakeBtn = false;
            motionControlMethod = -1;
            brepCut = new List<Brep>() ;
            helperFun = new Helpers();
            #region material and color settings

            int solidIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material solidMat = myDoc.Materials[solidIndex];
            solidMat.DiffuseColor = System.Drawing.Color.White;
            solidMat.SpecularColor = System.Drawing.Color.White;
            solidMat.Transparency = 0;
            solidMat.CommitChanges();
            solidAttribute = new ObjectAttributes();
            //solidAttribute.LayerIndex = 2;
            solidAttribute.MaterialIndex = solidIndex;
            solidAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            solidAttribute.ObjectColor = Color.White;
            solidAttribute.ColorSource = ObjectColorSource.ColorFromObject;

            int orangeIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material orangeMat = myDoc.Materials[orangeIndex];
            orangeMat.DiffuseColor = System.Drawing.Color.Orange;
            orangeMat.Transparency = 0.3;
            orangeMat.SpecularColor = System.Drawing.Color.Orange;
            orangeMat.CommitChanges();
            orangeAttribute = new ObjectAttributes();
            //orangeAttribute.LayerIndex = 3;
            orangeAttribute.MaterialIndex = orangeIndex;
            orangeAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            orangeAttribute.ObjectColor = Color.Orange;
            orangeAttribute.ColorSource = ObjectColorSource.ColorFromObject;

            int redIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material redMat = myDoc.Materials[redIndex];
            redMat.DiffuseColor = System.Drawing.Color.Red;
            redMat.Transparency = 0.3;
            redMat.SpecularColor = System.Drawing.Color.Red;
            redMat.CommitChanges();
            redAttribute = new ObjectAttributes();
            //redAttribute.LayerIndex = 4;
            redAttribute.MaterialIndex = redIndex;
            redAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            redAttribute.ObjectColor = Color.Red;
            redAttribute.ColorSource = ObjectColorSource.ColorFromObject;

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

            int greenIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material greenMat = myDoc.Materials[greenIndex];
            greenMat.DiffuseColor = System.Drawing.Color.FromArgb(16, 150, 206);
            greenMat.SpecularColor = System.Drawing.Color.FromArgb(16, 150, 206);
            greenMat.Transparency = 0.7f;
            greenMat.TransparentColor = System.Drawing.Color.FromArgb(16, 150, 206);
            greenMat.CommitChanges();
            greenAttribute = new ObjectAttributes();
            //greenAttribute.LayerIndex = 6;
            greenAttribute.MaterialIndex = greenIndex;
            greenAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            greenAttribute.ObjectColor = Color.FromArgb(72, 232, 88);
            greenAttribute.ColorSource = ObjectColorSource.ColorFromObject;

            #endregion
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
            pManager.AddIntegerParameter("IntervalRotationAngle", "Ang", "The angle for each step", GH_ParamAccess.item);

            // Confirm and bake all components
            pManager.AddBooleanParameter("ComponentsBake", "Bk", "comfirm and bake all components", GH_ParamAccess.item);

            pManager.AddIntegerParameter("Stroke", "Str", "number of strokes", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Energy", "E", "Energy of the output motion", GH_ParamAccess.item);

            pManager.AddBooleanParameter("DirectionReverse", "DirR", "reverse the rotation direction", GH_ParamAccess.item);
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

        void hideDirIndicator()
        {
            if (!rotationDirID.Equals(Guid.Empty))
            {
                myDoc.Objects.Hide(rotationDirID, true);
                myDoc.Views.Redraw();
            }
        }
        void showDirIndicator(bool isCCW)
        {
            if (!rotationDirID.Equals(Guid.Empty))
            {
                myDoc.Objects.Delete(rotationDirID, true);
                myDoc.Views.Redraw();
            }

            Vector3d canvasNormal = new Vector3d();
            Vector3d canvasStart = new Vector3d();
            Vector3d canvasEnd = new Vector3d();
            Point3d canvasOrigin = new Point3d();
            Point3d extrudeEnd = new Point3d();

            var sweep = new SweepOneRail();
            sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            sweep.ClosedSweep = false;
            sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            //switch (selectedAxisIndex)
            //{
            //    case 1:
            //        canvasNormal = Vector3d.XAxis;
            //        canvasStart = Vector3d.YAxis;
            //        canvasEnd = Vector3d.ZAxis;
            //        canvasOrigin = ((model.GetBoundingBox(true).Max.X - model.GetBoundingBox(true).Min.X) * 0.5 + 10) * Vector3d.XAxis + model.GetBoundingBox(true).Center;
            //        extrudeEnd = ((model.GetBoundingBox(true).Max.X - model.GetBoundingBox(true).Min.X) * 0.5 + 13) * Vector3d.XAxis + model.GetBoundingBox(true).Center;
            //        break;
            //    case 2:
            //        canvasNormal = Vector3d.YAxis;
            //        canvasStart = Vector3d.ZAxis;
            //        canvasEnd = Vector3d.XAxis;
            //        canvasOrigin = ((model.GetBoundingBox(true).Max.Y - model.GetBoundingBox(true).Min.Y) * 0.5 + 10) * Vector3d.YAxis + model.GetBoundingBox(true).Center;
            //        extrudeEnd = ((model.GetBoundingBox(true).Max.Y - model.GetBoundingBox(true).Min.Y) * 0.5 + 13) * Vector3d.YAxis + model.GetBoundingBox(true).Center;
            //        break;
            //    case 3:
            //        canvasNormal = Vector3d.ZAxis;
            //        canvasStart = Vector3d.XAxis;
            //        canvasEnd = Vector3d.YAxis;
            //        canvasOrigin = ((model.GetBoundingBox(true).Max.Z - model.GetBoundingBox(true).Min.Z) * 0.5 + 10) * Vector3d.ZAxis + model.GetBoundingBox(true).Center;
            //        extrudeEnd = ((model.GetBoundingBox(true).Max.Z - model.GetBoundingBox(true).Min.Z) * 0.5 + 13) * Vector3d.ZAxis + model.GetBoundingBox(true).Center;
            //        break;
            //}

            canvasNormal = shaftAxis;
            Plane t_pln = new Plane(model.GetBoundingBox(true).Center, shaftAxis);
            canvasStart = t_pln.XAxis;
            canvasEnd = t_pln.YAxis;
            canvasOrigin = model.GetBoundingBox(true).Center + shaftAxis / shaftAxis.Length * 40;
            extrudeEnd = model.GetBoundingBox(true).Center + shaftAxis / shaftAxis.Length * 43;

            Curve rail = new Line(canvasOrigin, extrudeEnd).ToNurbsCurve();

            // By default, the indicator is for clockwise
            #region create the ring

            Brep outerRing = Brep.CreatePipe(rail, 15, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Brep innerRing = Brep.CreatePipe(rail, 10, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];

            outerRing.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == outerRing.SolidOrientation)
                outerRing.Flip();

            innerRing.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == innerRing.SolidOrientation)
                innerRing.Flip();

            Brep ring = Brep.CreateBooleanDifference(outerRing, innerRing, myDoc.ModelAbsoluteTolerance)[0];

            ring.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == ring.SolidOrientation)
                ring.Flip();

            #endregion

            #region subtract a corner

            Point3d sb0 = canvasOrigin;
            Point3d sb1 = canvasOrigin + canvasStart * 16;
            Point3d sb2 = canvasOrigin + canvasStart * 16 + canvasEnd * 16;
            Point3d sb3 = canvasOrigin + canvasEnd * 16;
            Point3d sb4 = sb0;

            List<Point3d> subBoxCorners = new List<Point3d>();
            subBoxCorners.Add(sb0);
            subBoxCorners.Add(sb1);
            subBoxCorners.Add(sb2);
            subBoxCorners.Add(sb3);
            subBoxCorners.Add(sb4);

            Polyline subBoxRect = new Polyline(subBoxCorners);
            Curve subBoxRectCrv = subBoxRect.ToNurbsCurve();

            Brep[] subBoxBreps = sweep.PerformSweep(rail, subBoxRectCrv);
            Brep subBoxBrep = subBoxBreps[0];
            Brep subBox = subBoxBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            subBox.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == subBox.SolidOrientation)
                subBox.Flip();

            Brep arc = Brep.CreateBooleanDifference(ring, subBox, myDoc.ModelAbsoluteTolerance)[0];

            arc.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == arc.SolidOrientation)
                arc.Flip();

            #endregion

            #region create the arrow

            Point3d tri0 = canvasOrigin + canvasEnd * 7.5;
            Point3d tri1 = canvasOrigin + canvasEnd * 12.5 + canvasStart * 5;
            Point3d tri2 = canvasOrigin + canvasEnd * 17.5;
            Point3d tri3 = tri0;

            List<Point3d> arrowCorners = new List<Point3d>();
            arrowCorners.Add(tri0);
            arrowCorners.Add(tri1);
            arrowCorners.Add(tri2);
            arrowCorners.Add(tri3);

            Polyline arrowRect = new Polyline(arrowCorners);
            Curve arrowRectCrv = arrowRect.ToNurbsCurve();

            Brep[] arrowBreps = sweep.PerformSweep(rail, arrowRectCrv);
            Brep arrowBrep = arrowBreps[0];
            Brep arrow = arrowBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

            arrow.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
            if (BrepSolidOrientation.Inward == arrow.SolidOrientation)
                arrow.Flip();

            #endregion

            Brep dirIndicator = Brep.CreateBooleanUnion(new List<Brep> { arc, arrow }, myDoc.ModelAbsoluteTolerance)[0];

            if (isCCW)
            {
                // flip the indicator
                Transform tran = Transform.Mirror(canvasOrigin, canvasStart);
                dirIndicator.Transform(tran);
            }

            //isSpringCW = !isCCW;
            isSpringCW = !isSpringCW;
            rotationDirID = myDoc.Objects.AddBrep(dirIndicator, redAttribute);
            myDoc.Views.Redraw();
        }



        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool reg_input = false, control_pos = false, ee_pos = false, motion_axis = false, addlock_input = false, pre_input = false, bake_input = false, revdir_input = false;
            int motion_control_method = -1;
            int speed_input = 5;
            int step_angle_input = 5;
            int stroke_input = 5;
            int energy_input = 5;

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
            if (!DA.GetData(8, ref step_angle_input))
                return;
            if (!DA.GetData(9, ref bake_input))
                return;
            if (!DA.GetData(10, ref stroke_input))
                return;
            if (!DA.GetData(11, ref energy_input))
                return;
            if (!DA.GetData(12, ref revdir_input))
                return;
            #endregion

            // variables to control states
            bool toSelectRegion = false, toAdjustParam = false, toSetMotionControl = false, toSetEEPos = false, toSetAxisDir = false, toAddLock = false, toRemoveLock = false, toPreview = false, toBake = false, toRevDir = false;
            bool toGenerateMechanism = false;
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

            if (speedLevel == speed_input && stepAngleLevel == step_angle_input && strokeLevel == stroke_input && energyLevel == energy_input)
            {
                toAdjustParam = false;
            }
            else
            {
                speedLevel = speed_input;
                stepAngleLevel = step_angle_input;
                strokeLevel = stroke_input;
                energyLevel = energy_input;
                toAdjustParam = true;
            }

            if (dirReverseState != revdir_input)
            {
                dirReverseState = revdir_input;
                showDirIndicator(dirReverseState);
                if (spring_entities.Count != 0)
                {
                    motion.AdjustParameter(-eeTranslation, eeMovingDirectionSelection, speedLevel, strokeLevel, energyLevel, gr_list, gear_schemes, isSpringCW, spring_entities.ElementAt(0), motionControlMethod, ref lockPos, ref spiralLockNorm, ref spiralLockDir);
                }
                //motion.AdjustParameter(energyLevel, displacement, isSpringCW);
                //motion.updateLock(dirReverseState);
            }

            #endregion

            if (toSelectRegion)
            {
                if (selObjId != Guid.Empty)
                {
                    myDoc.Objects.Show(selObjId, true);
                    myDoc.Views.Redraw();
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
                    myDoc.Objects.Select(currObj);

                    model = currObj.Brep();

                    #endregion

                    #region Pre-process #2: get the ininital inner cavity of the selected brep

                    BoundingBox box = model.GetBoundingBox(true);
                    box.Inflate(-2.0);
                    box.Transform(Transform.Scale(box.Center, 2));
                    //arrowScale = box.Diagonal.Length / 100;
                    center = box.Center;

                    #endregion
                }
            }

            if (toSetMotionControl)
            {
                if (model != null)
                {
                    //Step 1: select the axis (X, Y, or Z axis)
                    selectedAxisIndex = new XYZselection(center, myDoc, redAttribute, greenAttribute, blueAttribute, 30).selectedAxis;

                    //region Step 2: drag the two planes to decide the portion
                    PortionSelection pselect = new PortionSelection(model, selectedAxisIndex, myDoc);
                    t1 = pselect.t1;
                    t2 = pselect.t2;
                    selectedAxisVector = pselect.axis;
                    direction = selectedAxisVector;
                    skeleton = pselect.skeleton;
                    PlaneSelected = true;


                    #region Step 3: cut model and calculate the inner cavity


                    if (PlaneSelected)
                    {
                        brepCut.Clear();
                        // Call out the waiting window
                        processingwin.Show();

                        brepCut = Helpers.cutModel(model, skeleton, t1, t2, selectedAxisVector, myDoc);
                        innerCavity = Helpers.getInnerCavity(brepCut, selectedAxisVector);


                        processingwin.Hide();

                        myDoc.Objects.Hide(selObjId, true);

                        reserveBrepID1 = myDoc.Objects.AddBrep(brepCut[0]);
                        reserveBrepID2 = myDoc.Objects.AddBrep(brepCut[2]);

                        convertedPortion = myDoc.Objects.AddBrep(brepCut[1]);
                        myDoc.Objects.Select(convertedPortion);
                    }

                    #endregion

                    //Step 4: ask the user to select which side to add the motion control
                    motionCtrlPointSelected = new SideSelection(skeleton, myDoc, blueAttribute).motionCtrlPointSelected;

                    #region Step 5: create an instance of Continuous Translation class

                    motion = new IntermittentRotation(model,innerCavity, direction, energy_input, stroke_input, speed_input, helperFun, motionControlMethod);

                    motion.Set3Parts(t1, t2, brepCut[0], brepCut[1], brepCut[2]);

                    //toBeBaked.Add(reserveBrepID1);
                    //toBeBaked.Add(reserveBrepID2);
                    //myDoc.Views.Redraw();
                    #endregion
                }
            }

            if (toSetEEPos)
            {
                //The new process to implement. Let user select one or 2 models. When 1 model is selected, use the direction as shaft direction and generate gears.
                //If 2 models are selected, tell is they are on 2 sides, if not, throw exception; if so, use the direction as shaft direction.
                //When 1 ee is selected, use revolute joint. when 2, use a cut through axis.

                #region Step 1 First let user choose the ee models.Remind them not to accidentally click on 3 parts
                ObjRef objSel_ref1;
                var rc1 = RhinoGet.GetOneObject("Select the 1st end effector model. ", false, ObjectType.AnyObject, out objSel_ref1);
                if (rc1 == Rhino.Commands.Result.Success)
                {
                    //set up first ee
                    ee1 = objSel_ref1.ObjectId;
                    ObjRef currObj = new ObjRef(ee1);
                    //myDoc.Objects.Select(currObj);
                    ee1Model = currObj.Brep();
                    myDoc.Objects.Hide(ee1, true);
                    myDoc.Views.Redraw();
                }
                else
                {
                    endEffectorState = 0;
                    warningwin.Text = "No end effector model is selected.";
                    warningwin.Show();
                }
                if (rc1 == Rhino.Commands.Result.Success)
                {
                    ObjRef objSel_ref2;
                    var rc2 = RhinoGet.GetOneObject("Select the second end effector model if neeed. Or press Enter to skip this selection and go with one model ", true, ObjectType.AnyObject, out objSel_ref2);
                    if (rc2 == Rhino.Commands.Result.Success)
                    {
                        if (objSel_ref2 == null)
                        {
                            //Go with one ee
                            endEffectorState = 1;
                        }
                        else
                        {
                            //set up second ee 
                            ee2 = objSel_ref2.ObjectId;
                            ObjRef currObj = new ObjRef(ee2);
                            myDoc.Objects.Select(currObj);
                            ee2Model = currObj.Brep();
                            endEffectorState = 2;
                            myDoc.Objects.Hide(ee2, true);
                            myDoc.Views.Redraw();
                        }
                    }
                }

                #endregion
                
                toGenerateMechanism = true;
            }
            if (toGenerateMechanism)
            {
                #region Use the given ee to calculate direction, generate gear and shaft
                //Clacluate direction
                Vector3d mainAxis = Vector3d.Unset, perpAxis = Vector3d.Unset;

                BoundingBox Bbox = model.GetBoundingBox(false);
                switch (selectedAxisIndex)
                {
                    case 1: mainAxis = Vector3d.XAxis; break;
                    case 2: mainAxis = Vector3d.YAxis; break;
                    case 3: mainAxis = Vector3d.ZAxis; break;
                    default: break;
                }
                mainAxis.Unitize();
                //reverse main axis if needed based on user selection of ee side.
                if ((skeleton.PointAtNormalizedLength(0.5) - motionCtrlPointSelected) * mainAxis < 0)
                    mainAxis = -mainAxis;

                #region find the end effector center point
                BoundingBox innerCavityOriginalBbox = innerCavity.GetBoundingBox(true);
                double lastShaftInwardOffset = 15.5 + 2 + 0.6;//Xia's note: this value is to make sure the driven wheel is completely within middle part
                eeCenPt = innerCavityOriginalBbox.Center + (innerCavityOriginalBbox.Diagonal * mainAxis / 2 - lastShaftInwardOffset) * mainAxis;
                //myDoc.Objects.AddPoint(eeCenPt);
                //myDoc.Views.Redraw();
                #endregion

                if (endEffectorState == 1)
                {
                    //find the direction from eeCenpt to ee center
                    BoundingBox eeBbox = ee1Model.GetBoundingBox(true);
                    Vector3d eeVectorPrimitive = eeBbox.Center - eeCenPt;
                    Vector3d eeVector = eeVectorPrimitive - mainAxis * (eeVectorPrimitive * mainAxis);
                    shaftAxis = new Vector3d(eeVector);
                    shaftAxis.Unitize();
                    ObjRef currObj = new ObjRef(ee1);
                    ee1Model = currObj.Brep();
                    //move ee model to match eeVector
                    //ee1 = myDoc.Objects.Transform(ee1, Transform.Translation(eeVector - eeVectorPrimitive), true);
                    //myDoc.Objects.Hide(ee1, true);
                    ee1Model.Transform(Transform.Translation(eeVector - eeVectorPrimitive));
                }
                else
                {
                    //find the direction from one ee to the other
                    BoundingBox ee1Bbox = ee1Model.GetBoundingBox(true);
                    BoundingBox ee2Bbox = ee2Model.GetBoundingBox(true);
                    Vector3d eeVectorPrimitive = ee1Bbox.Center - ee2Bbox.Center;
                    Vector3d eeVector = eeVectorPrimitive - mainAxis * (eeVectorPrimitive * mainAxis);
                    shaftAxis = new Vector3d(eeVector);
                    shaftAxis.Unitize();
                    //move ee model to match ee vector. first move middle point to eeCenpt, then move 2 ee to same perp plane
                    Point3d eeMidPt = new Line(ee1Bbox.Center, ee2Bbox.Center).ToNurbsCurve().PointAtNormalizedLength(0.5);
                    Vector3d translation1 = eeCenPt - eeMidPt;
                    Vector3d translation2 = eeVector - eeVectorPrimitive;
                    ObjRef currObj = new ObjRef(ee1);
                    ee1Model = currObj.Brep();
                    currObj = new ObjRef(ee2);
                    ee2Model = currObj.Brep();
                    //ee1 = myDoc.Objects.Transform(ee1, Transform.Translation(translation1 + translation2 / 2), true);
                    //ee2 = myDoc.Objects.Transform(ee2, Transform.Translation(translation1 - translation2 / 2), true);
                    //myDoc.Objects.Hide(ee1, true);
                    //myDoc.Objects.Hide(ee2, true);
                    ee1Model.Transform(Transform.Translation(translation1 + translation2 / 2));
                    ee2Model.Transform(Transform.Translation(translation1 - translation2 / 2));
                    eeTranslation = translation1;
                }
                perpAxis = new Plane(eeCenPt, mainAxis, shaftAxis).Normal;
                perpAxis.Unitize();
                motion.Set3Axis(mainAxis, perpAxis, shaftAxis);
                if (endEffectorState == 2)
                {
                    eeTranslation = eeTranslation * perpAxis * perpAxis;
                }
                //calculate last gear position using given params
                double _b = 15.5;
                double _c = _b / Math.Cos(180.0 / step_angle_input * Math.PI / 180);
                double _a = Math.Sqrt(Math.Pow(_c, 2) - Math.Pow(_b, 2));
                Point3d lgc = eeCenPt - mainAxis * _c;
                #endregion
                #region generate gear, shaft, geneva drive and ee shaft
                //generate gear param
                Box innerCavityBox = new Box(innerCavity.GetBoundingBox(true));
                //Offset inner cavity by 2mm
                innerCavityBox.Inflate(-2);
                gear_schemes = GenerateGearTrain.GetGearTrainSchemes(mainAxis, shaftAxis, lgc, innerCavityBox, 3.6,motionControlMethod, 2);
                //select gear param based on input param
                if (gear_schemes.Count == 0)
                {
                    // the selected body is too small to embed the kinetic unit
                    warningwin.Text = "The selected body is too small to add the kinetic unit!";
                    warningwin.Show();
                }
                else
                {
                    Vector3d perVec = Vector3d.CrossProduct(mainAxis, shaftAxis);
                    perVec.Unitize();
                    double gw_dis = Math.Abs(innerCavityBox.BoundingBox.Diagonal * perVec / 2);

                    gr_list = new List<double>();
                    foreach (GearTrainScheme gts in gear_schemes)
                    {
                        foreach (GearTrainParam gtp in gts.parameters)
                        {
                            //Xia's note: added a distance filter to make sure geneva driving wheel won't correlate with shaft
                            if (motionControlMethod == 1)
                            {
                                if ((gtp.bullGearRadius + gtp.pinionRadius + 0.3 > _a + 2 + 0.6) && (gtp.pinionRadius + 0.6 + 1.125 + 2 + 2 + 0.4) <= gw_dis)
                                    gr_list.Add(gtp.gearRatio);
                            }
                            else
                            {
                                if (gtp.bullGearRadius + gtp.pinionRadius + 0.3 > _a + 2 + 0.6) 
                                    gr_list.Add(gtp.gearRatio);
                            }

                        }
                    }

                    gr_list.Sort();

                    int schemeNum = -1;
                    int paramNum = -1;
                    if (gr_list.Count == 0)
                    {
                        warningwin.Text = "The selected body is too small to add the kinetic unit!";
                        warningwin.Show();
                        return;
                    }
                    helperFun.mapSpeedToGears(speed_input, gr_list, gear_schemes, out schemeNum, out paramNum);

                    if (paramNum == -1 || schemeNum == -1)
                    {
                        warningwin.Text = "The selected body is too small to add the kinetic unit!";
                        warningwin.Show();
                        return;
                    }
                    selectedGearTrainParam = gear_schemes[schemeNum].parameters[paramNum];
                    //gear_info.Clear();
                    //gear_info = gear_schemes[schemeNum].parameters[paramNum].parameters;


                }

                //Register EE in motion
                List<Brep> ees = new List<Brep>();
                if (endEffectorState == 1)
                {
                    ees.Add(ee1Model);
                }
                else if (endEffectorState == 2)
                {
                    ees.Add(ee1Model);
                    ees.Add(ee2Model);
                }
                motion.SetEndEffectors(endEffectorState, ees);
                //Generate gear, shaft and spring
                #region generate all the axels and spacers for the gears
                axel_spacer_entities = helperFun.genAxelsStoppers(selectedGearTrainParam.parameters, model, motionControlMethod, 0.3);
                #endregion

                #region generate all the gears
                gears = helperFun.genGears(selectedGearTrainParam.parameters, motionControlMethod, 0.4, true);
                #endregion

                #region ask the user to select rotation direction and generate spring

                eeMovingDirectionSelection = 1; // 1:CW, 3: CCW
                //TODO check the param input
                
                if (selectedGearTrainParam.gearSetNumber % 2 == 1)
                {
                    // output direction is opposite with the input direction
                    dirControl = false;
                    motion.Old_direction = dirControl;
                }
                else
                {
                    // output direction is the same with the input direction
                    dirControl = true;
                    motion.Old_direction = dirControl;
                }

                spring_entities = helperFun.genSprings(selectedGearTrainParam.parameters, model, skeleton, mainAxis, motionControlMethod, strokeLevel, energyLevel, dirControl, out lockPos, out spiralLockNorm, out spiralLockDir, out socketBrep, gears.ElementAt(0));

                // determine the rotating direction
                showDirIndicator(false);

                #endregion
                #region Generate Geneva drive
                GearParameter lgp = selectedGearTrainParam.parameters.Last();
                lgc = lgp.center;
                double lgcCoordinate = new Vector3d(lgc) * shaftAxis;
                double lgcCoordinateOffset = new Vector3d(lgc + lgp.norm * lgp.faceWidth) * shaftAxis;
                double spaceLeftOver = 0;
                //Get the shaft axis range to tell if there's enough space for geneva drive
                Shaft lastShaft = null;
                foreach (Entity e in axel_spacer_entities)
                {
                    if (e.Name == "lastShaft")
                        lastShaft = (Shaft)e;
                }
                Point3d lastShaftStart = lastShaft.StartPt;
                Point3d lastShaftEnd = lastShaft.StartPt + lastShaft.AxisDir * lastShaft.Len;
                double lastShaftStartCoordinate = new Vector3d(lastShaftStart) * shaftAxis;
                double lastShaftEndCoordinate = new Vector3d(lastShaftEnd) * shaftAxis;
                Interval shaftAxisRange = new Interval(lastShaftStartCoordinate, lastShaftEndCoordinate);
                if (lastShaftEndCoordinate < lastShaftStartCoordinate)
                    shaftAxisRange.Swap();
                if (lgcCoordinate < lgcCoordinateOffset)
                {
                    spaceLeftOver = shaftAxisRange.Max - lgcCoordinateOffset - 0.6;
                }
                else
                {
                    spaceLeftOver = lgcCoordinateOffset - shaftAxisRange.Min - 0.6;
                }
                if (spaceLeftOver < 8)//TODO test this threshold to see if it's too big or too small
                {
                    warningwin.Text = "The selected body is too small to add the quick return structure!";
                    warningwin.Show();
                }
                else
                {
                    //Generate quick return and add it
                    BoundingBox bboxMid = brepCut[1].GetBoundingBox(true);
                    Point3d genevaDrivenWheelCenter = lgc + lgp.norm * (lgp.faceWidth) + lgp.norm * 3.9 + mainAxis * _c;
                    genevaDrive = new GenevaDrive(genevaDrivenWheelCenter, step_angle_input, lgp.norm, 3.6, mainAxis);
                    //Add geneva models
                    Brep gw = genevaDrive.GenevaModels[0], dw = genevaDrive.GenevaModels[1], pin = genevaDrive.GenevaModels[2], stopper = genevaDrive.GenevaModels[3];
                    Brep dwpin = Brep.CreateBooleanUnion(new List<Brep> { dw, pin }, myDoc.ModelAbsoluteTolerance)[0];
                    GenevaDrivingWheelWithPin = new Entity(dwpin, false, "Drivng wheel wheel with pin on");
                    GenevaWheel = new Entity(gw, false, "Geneva Wheel");
                    GenevaStopper = new Entity(stopper, false, "Geneva Stopper");

                    motion.AddGenevaDrive(GenevaDrivingWheelWithPin, GenevaWheel, GenevaStopper);
                    //Move last gear to join lg and cw
                    //gears.Last().Model.Transform(Transform.Translation(lgp.norm * 0.6));

                    #region add a new last shaft to connect ee
                    //Remove last 2 spacers
                    //List<Spacer> generatedSpacers = new List<Spacer>();
                    //for (int i = 0; i < axel_spacer_entities.Count; i++)
                    //{
                    //    Entity e = axel_spacer_entities[i];
                    //    if (e.GetType() == typeof(Spacer))
                    //        generatedSpacers.Add((Spacer)e);
                    //}
                    //axel_spacer_entities.Remove(generatedSpacers[generatedSpacers.Count - 1]);
                    //axel_spacer_entities.Remove(generatedSpacers[generatedSpacers.Count - 2]);

                    //Intersect with middle part to know how long shaft will be
                    axelDir = shaftAxis;
                    Curve crossLineCrv = new Line(genevaDrivenWheelCenter - axelDir * int.MaxValue, genevaDrivenWheelCenter + axelDir * int.MaxValue).ToNurbsCurve();
                    Curve[] crvs;
                    Point3d[] pts;
                    Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, model, myDoc.ModelAbsoluteTolerance, out crvs, out pts);

                    Point3d ptEnd = new Point3d();
                    Point3d ptStart = new Point3d();
                    Vector3d intersectVec = (pts[0] - pts[1]);
                    intersectVec.Unitize();
                    if (intersectVec * axelDir > 0.99)
                    {
                        ptEnd = pts[0] - axelDir * 1;
                        ptStart = pts[1] + axelDir * 1;
                    }
                    else
                    {
                        ptEnd = pts[1] - axelDir * 1;
                        ptStart = pts[0] + axelDir * 1;
                    }

                    //add new last shaft for geneva drive driven wheel. If one ee, use revolute joint. If 2 ee, use regular shaft
                    if (endEffectorState == 1)
                    {
                        //Add a new last shaft with a longer one with a revolute joint at end
                        Point3d pt1 = ptStart;
                        Point3d pt2 = ptEnd;
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
                        Point3d pt1 = ptStart;
                        Point3d pt2 = ptEnd;
                        //Add shaft to link 2 ees
                        Point3d axelStart = ee1Model.GetBoundingBox(true).Center;
                        Point3d axelEnd = ee2Model.GetBoundingBox(true).Center;
                        Vector3d axelDir = axelEnd - axelStart;
                        double axelLen = axelDir.Length;
                        axelDir.Unitize();
                        Shaft newLastShaft = new Shaft(axelStart, axelLen, 2, axelDir);
                        newLastShaft.SetName("MiddleShellBreakerShaft");
                        //TODO Add spacer along line within model ? Not adding for now to prevent bug
                        axel_spacer_entities.Add(newLastShaft);
                    }
                    else
                    {
                        throw new Exception("Unexpected situation! Invalid end effector state");
                    }


                    if (endEffectorState == 2)
                    {
                        foreach (Entity e in gears)
                        {
                            e.Model.Transform(Transform.Translation(-eeTranslation));
                        }
                        foreach (Entity e in axel_spacer_entities)
                        {
                            e.Model.Transform(Transform.Translation(-eeTranslation));
                        }
                        foreach (Entity e in spring_entities)
                        {
                            e.Model.Transform(Transform.Translation(-eeTranslation));
                        }
                        ee1Model.Transform(Transform.Translation(-eeTranslation));
                        ee2Model.Transform(Transform.Translation(-eeTranslation));

                        #endregion
                    }

                    List<Spacer> generatedSpacers = new List<Spacer>();
                    for (int i = 0; i < axel_spacer_entities.Count; i++)
                    {
                        Entity e = axel_spacer_entities[i];
                        if (e.GetType() == typeof(Spacer))
                            generatedSpacers.Add((Spacer)e);
                    }
                    if ((generatedSpacers[generatedSpacers.Count - 1].StartPt - lgc) * lgp.norm > 0)
                    {
                        axel_spacer_entities.Remove(generatedSpacers[generatedSpacers.Count - 1]);
                    }
                    else
                    {
                        axel_spacer_entities.Remove(generatedSpacers[generatedSpacers.Count - 2]);
                    }

                    motion.AddGears(gears, axel_spacer_entities, selectedGearTrainParam);
                    motion.AddSprings(spring_entities);

                    #endregion

                    #endregion
                }
            }


            if (toAddLock)
            {
                if (motion != null)
                    motion.ConstructLocks(-eeTranslation, lockPos, spiralLockNorm, spiralLockDir, selectedGearTrainParam, spring_entities, motionControlMethod);
            }

            if (toRemoveLock)
            {
                if (motion != null)
                    motion.RemoveLocks(motionControlMethod);
            }

            if (toPreview)
            {

            }

            if (toBake)
            {
                myDoc.Objects.Delete(reserveBrepID1, true);
                myDoc.Objects.Delete(reserveBrepID2, true);
                myDoc.Objects.Delete(convertedPortion, true);
                if (motion != null)
                    motion.CreateShell(socketBrep);
                if (motion != null)
                {
                    //foreach (Guid id in endEffectorCandidates)
                    //{
                    //    myDoc.Objects.Hide(id, true);
                    //}

                    if (motion.EntityList != null)
                    {
                        foreach (Entity b in motion.EntityList)
                        {
                            Brep tempB = b.GetModelinWorldCoordinate();
                            myDoc.Objects.AddBrep(tempB);
                        }
                        //if (motion.Spring.SpringDimensions != null)
                        //{
                        //    foreach (LinearDimension d in motion.Spring.SpringDimensions)
                        //    {
                        //        //if(d.Plane.ClosestPoint(d.Arrowhead1End))
                        //        //{
                        //        myDoc.Objects.AddLinearDimension(d);
                        //        //}
                        //    }
                        //}
                        myDoc.Views.Redraw();
                        this.ExpirePreview(true);
                    }
                }
            }

            if (toAdjustParam)
            {
                if (motion != null)
                    motion.AdjustParameter(-eeTranslation, eeMovingDirectionSelection, speedLevel, strokeLevel, energyLevel, gr_list, gear_schemes, isSpringCW, spring_entities.ElementAt(0), motionControlMethod, ref lockPos, ref spiralLockNorm, ref spiralLockDir);
            }

            DA.SetData(0, motion);
            //DA.SetData(1, model);
            if (motion == null)
                DA.SetDataList(1, null);
            else
                DA.SetDataList(1, motion.GetModel());
            DA.SetData(2, toPreview);
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
            get { return new Guid("e80ec7a7-0198-48de-9e37-da0cfed82266"); }
        }
    }
}