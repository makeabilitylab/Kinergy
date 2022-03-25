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

            switch (selectedAxisIndex)
            {
                case 1:
                    canvasNormal = Vector3d.XAxis;
                    canvasStart = Vector3d.YAxis;
                    canvasEnd = Vector3d.ZAxis;
                    canvasOrigin = ((model.GetBoundingBox(true).Max.X - model.GetBoundingBox(true).Min.X) * 0.5 + 10) * Vector3d.XAxis + model.GetBoundingBox(true).Center;
                    extrudeEnd = ((model.GetBoundingBox(true).Max.X - model.GetBoundingBox(true).Min.X) * 0.5 + 13) * Vector3d.XAxis + model.GetBoundingBox(true).Center;
                    break;
                case 2:
                    canvasNormal = Vector3d.YAxis;
                    canvasStart = Vector3d.ZAxis;
                    canvasEnd = Vector3d.XAxis;
                    canvasOrigin = ((model.GetBoundingBox(true).Max.Y - model.GetBoundingBox(true).Min.Y) * 0.5 + 10) * Vector3d.YAxis + model.GetBoundingBox(true).Center;
                    extrudeEnd = ((model.GetBoundingBox(true).Max.Y - model.GetBoundingBox(true).Min.Y) * 0.5 + 13) * Vector3d.YAxis + model.GetBoundingBox(true).Center;
                    break;
                case 3:
                    canvasNormal = Vector3d.ZAxis;
                    canvasStart = Vector3d.XAxis;
                    canvasEnd = Vector3d.YAxis;
                    canvasOrigin = ((model.GetBoundingBox(true).Max.Z - model.GetBoundingBox(true).Min.Z) * 0.5 + 10) * Vector3d.ZAxis + model.GetBoundingBox(true).Center;
                    extrudeEnd = ((model.GetBoundingBox(true).Max.Z - model.GetBoundingBox(true).Min.Z) * 0.5 + 13) * Vector3d.ZAxis + model.GetBoundingBox(true).Center;
                    break;
            }

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

            isSpringCW = !isCCW;
            rotationDirID = myDoc.Objects.AddBrep(dirIndicator, redAttribute);
            myDoc.Views.Redraw();
        }


        double ConvertInputAngleToDoubleType(string angleText)
        {
            double result = 0;
            try
            {
                result = Convert.ToDouble(angleText);
            }
            catch (Exception e)
            {
                result = 0;
            }
            return result;
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
                toAdjustParam = true;
            }

            if (dirReverseState != revdir_input)
            {
                dirReverseState = revdir_input;
                showDirIndicator(dirReverseState);
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

                    motion = new IntermittentRotation(model, direction, energy_input, stroke_input, speed_input);

                    motion.Set3Parts(t1, t2, brepCut[0], brepCut[1], brepCut[2]);

                    //toBeBaked.Add(reserveBrepID1);
                    //toBeBaked.Add(reserveBrepID2);
                    //myDoc.Views.Redraw();
                    #endregion
                }
            }

            if (toSetEEPos)
            {
                // select a direction around a circle on the end-effector's side
                EndEffectorDirectionSelection directionSelection = new EndEffectorDirectionSelection(brepCut, skeleton, direction, motionCtrlPointSelected, t1, t2, myDoc, redAttribute);

                kineticUnitDir = directionSelection.kineticUnitDir;
                axelDir = directionSelection.axelDir;
                eeCenPt = directionSelection.eeCenPt;
                //Xia's note: moved eeCenPt by 2mm inward.

                //Then select ee model
                ObjRef objSel_ref1;
                var rc1 = RhinoGet.GetOneObject("Select the end effector model. ", false, ObjectType.AnyObject, out objSel_ref1);
                if (rc1 == Rhino.Commands.Result.Success)
                {
                    //set up first ee
                    ee = objSel_ref1.ObjectId;
                    ObjRef currObj = new ObjRef(ee);
                    myDoc.Objects.Select(currObj);
                    eeModel = currObj.Brep();
                    myDoc.Objects.Hide(ee, true);
                    myDoc.Views.Redraw();
                }
                else
                {
                    warningwin.Text = "No end effector model is selected.";
                    warningwin.Show();
                }
                //Use ee position to calculate important parameters

                //TODO 

                //Then generate gears , crank and more. TODO change from CT to IO
                #region generate the spring motor, transmission mechanism, and the mechanism mating the end-effector
                Point3d startPt = skeleton.PointAtNormalizedLength(0);
                Point3d endPt = skeleton.PointAtNormalizedLength(1);

                if (t1 > t2)
                {
                    endPt = skeleton.PointAtNormalizedLength(t1);
                    startPt = skeleton.PointAtNormalizedLength(t2);
                }
                else
                {
                    startPt = skeleton.PointAtNormalizedLength(t1);
                    endPt = skeleton.PointAtNormalizedLength(t2);
                }
                double unitLenth = startPt.DistanceTo(endPt);
                //double initialOffset = finalGearPositionRatio * pts[1].DistanceTo(pts[0]);
                //motion.CalculateSpaceForKineticUnit(kineticUnitDir, axelDir, axelSpace, gearSpace, unitLenth, initialOffset, finalGearPositionRatio);

                // wait for Xia, eeLineDotPt is the point passed to Xia's function 
                //motion.GenerateGearTrain(finalGearPositionRatio, eeCenPt, speed_input, kineticUnitDir, axelDir);

                // convert the inner cavity brep into box
                Box innerCavityBox = new Box(innerCavity.GetBoundingBox(true));
                //Offset inner cavity by 2mm
                innerCavityBox.Inflate(-2);
                // gear's facewidth is fixed for our project except for the first gear in the gear train
                List<GearTrainScheme> gear_schemes = GenerateGearTrain.GetGearTrainSchemes(direction, axelDir, eeLineDotPt, innerCavityBox, 3.6);

                if (gear_schemes.Count == 0)
                {
                    // the selected body is too small to embed the kinetic unit
                    warningwin.Text = "The selected body is too small to add the kinetic unit!";
                    warningwin.Show();
                }
                else
                {
                    #region for testing
                    //List<GearParameter> parameters = gear_schemes[0].parameters[0].parameters;
                    //for (int i = 0; i < parameters.Count; i++)
                    //{
                    //    GearParameter p = parameters[i];
                    //    Gear newGear = new Gear(p.center, p.norm, p.xDirection, (int)Math.Floor(p.radius * 2), 1, 20, p.faceWidth, 0, false);
                    //    newGear.Generate();

                    //    myDoc.Objects.AddBrep(newGear.Model);
                    //    myDoc.Views.Redraw();
                    //}
                    #endregion

                    List<double> gr_list = new List<double>();
                    foreach (GearTrainScheme gts in gear_schemes)
                    {
                        foreach (GearTrainParam gtp in gts.parameters)
                        {
                            gr_list.Add(gtp.gearRatio);
                        }
                    }

                    gr_list.Sort();

                    int schemeNum = -1;
                    int paramNum = -1;

                    helperFun.mapSpeedToGears(speed_input, gr_list, gear_schemes, out schemeNum, out paramNum);

                    if (paramNum == -1 || schemeNum == -1)
                    {
                        return;
                    }
                    selectedGearTrainParam = gear_schemes[schemeNum].parameters[paramNum];
                    //gear_info.Clear();
                    //gear_info = gear_schemes[schemeNum].parameters[paramNum].parameters;

                    #region generate all the axels and spacers for the gears
                    axel_spacer_entities = helperFun.genAxelsStoppers(selectedGearTrainParam.parameters, model, motionControlMethod, 0.3);
                    #endregion

                    #region generate all the gears
                    gears = helperFun.genGears(selectedGearTrainParam.parameters, motionControlMethod, 0.4);
                    #endregion
                    motion.AddGears(gears, axel_spacer_entities, selectedGearTrainParam);
                }
                #endregion
            }

            if (toSetAxisDir)
            {

            }

            if (toAddLock)
            {

            }

            if (toRemoveLock)
            {

            }

            if (toPreview)
            {

            }

            if (toBake)
            {

            }

            if (toAdjustParam)
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