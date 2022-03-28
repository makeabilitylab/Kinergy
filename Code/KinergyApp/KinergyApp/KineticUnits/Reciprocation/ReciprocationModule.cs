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

namespace InterReciprocation
{
    public class ReciprocationModule : GH_Component
    {
        // Variables that store information
        Brep model;             // the original brep model
        Brep conBrep;           // the Brep that is selected and converted
        Brep innerCavity;
        double t1, t2; // the positoins of start point and end point of the segment on the normalized skeleton
        Curve skeleton;     // skeleton
        int speedLevel;
        int rangeLevel;
        int strokeLevel;
        int energyLevel;
        double displacement;   // value of the distance slide bar
        Vector3d direction;             // kinetic unit direction
        Reciprocation motion;
        List<Arrow> lockDirCandidates;
        Arrow p;
        Helpers helperFun;

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
        bool PlaneSelected = false;
        Guid selected = Guid.Empty;
        Vector3d skeletonVec = Vector3d.Unset;
        Vector3d selectedAxisVector = Vector3d.Unset;
        ProcessingWin processingwin = new ProcessingWin();
        WarningWin warningwin = new WarningWin();

        RhinoDoc myDoc;
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

        Entity crankWheel;
        Entity yokeSlider ;
        Entity stopWall ;
        Entity bearingBlock ;

        /// <summary>
        /// Initializes a new instance of the ReciprocationModule class.
        /// </summary>
        public ReciprocationModule()
          : base("ReciprocationModule", "RModule",
              "The kinetic unit for reciprocation",
              "Kinergy", "KineticUnits")
        {
            model = null;
            conBrep = new Brep();
            innerCavity = new Brep();
            brepCut = new List<Brep>();
            t1 = 0;
            t2 = 1;
            skeleton = null;
            speedLevel = 5;
            strokeLevel = 5;
            rangeLevel = 5;
            direction = new Vector3d();
            motion = null;
            lockDirCandidates = new List<Arrow>();
            p = null;
            speedLevel = 0;
            energyLevel = 5;

            lockState = false;
            min_wire_diamter = 2.8;
            min_coil_num = 3;
            energy = 0.5;
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
            pManager.AddIntegerParameter("Speed", "Speed", "Speed of motion", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Amplitude", "A", "The amplitude of the reciprocation", GH_ParamAccess.item);

            // Confirm and bake all components
            pManager.AddBooleanParameter("ComponentsBake", "Bk", "comfirm and bake all components", GH_ParamAccess.item);

            pManager.AddIntegerParameter("Stroke", "Str", "number of strokes", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Energy", "E", "Energy of the output motion", GH_ParamAccess.item);
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

        int ConvertPatternToIntType(string patternType)
        {
            int result = 0;
            switch (patternType)
            {
                case @"Uniform acceleration & deceleration": result = 0; break;
                case @"Slightly rising & suddenly falling": result = 1; break;
            }
            return result;
        }

        int ConvertAmpToIntType(string ampType)
        {
            int result = 0;
            switch (ampType)
            {
                case "small": result = 0; break;
                case "medium": result = 1; break;
                case "big": result = 1; break;
            }
            return result;
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool reg_input = false, control_pos = false, ee_pos = false, motion_axis = false, addlock_input = false, pre_input = false, bake_input = false;
            int motion_control_method = -1;
            int speed_input = 5;
            int amp_input = 5;
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
            if (!DA.GetData(8, ref amp_input))
                return;
            if (!DA.GetData(9, ref bake_input))
                return;
            if (!DA.GetData(10, ref stroke_input))
                return;
            if (!DA.GetData(11, ref energy_input))
                return;
            #endregion

            // variables to control states
            bool toSelectRegion = false, toSetMotionControl = false, toSetEEPos = false, toSetAxisDir = false, toAdjustParam = false, toAddLock = false, toRemoveLock = false, toPreview = false, toBake = false;
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

            if (speedLevel == speed_input && rangeLevel == amp_input && strokeLevel == stroke_input && energy_input == energyLevel)
            {
                toAdjustParam = false;
            }
            else
            {
                speedLevel = speed_input;
                rangeLevel = amp_input;
                strokeLevel = stroke_input;
                energyLevel = energy_input;
                toAdjustParam = true;
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

                    motion = new Reciprocation(model, direction,  energy_input,  speed_input,innerCavity);//TODO adjust

                    motion.Set3Parts(t1, t2, brepCut[0], brepCut[1], brepCut[2]);

                    //toBeBaked.Add(reserveBrepID1);
                    //toBeBaked.Add(reserveBrepID2);
                    //myDoc.Views.Redraw();
                    #endregion
                }
            }

            if (toSetEEPos)
            {
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
                    //myDoc.Objects.Hide(ee, true);
                    //myDoc.Views.Redraw();
                }
                else
                {
                    warningwin.Text = "No end effector model is selected.";
                    warningwin.Show();
                }

                motion.SetEndEffector(eeModel);
                toGenerateMechanism = true;
            }
            if(toGenerateMechanism)
            { 
                //Then generate gears , quick return and more. TODO change from CT to IO
                #region generate the spring motor, transmission mechanism, and the mechanism mating the end-effector
                Vector3d mainAxis = Vector3d.Unset, perpAxis = Vector3d.Unset, shaftAxis = Vector3d.Unset;
                Box Bbox = new Box(model.GetBoundingBox(false));
                Interval shaftAxisRange=new Interval();
                switch (selectedAxisIndex)
                {
                    case 1: mainAxis = Vector3d.XAxis;perpAxis = Vector3d.ZAxis;shaftAxis=Vector3d.YAxis ;shaftAxisRange = Bbox.Y; break;
                    case 2: mainAxis = Vector3d.YAxis; perpAxis = Vector3d.ZAxis; shaftAxis = Vector3d.XAxis ; shaftAxisRange = Bbox.X; break;
                    case 3: mainAxis = Vector3d.ZAxis; perpAxis = Vector3d.XAxis; shaftAxis = Vector3d.YAxis ; shaftAxisRange = Bbox.Y; break;
                    default: break;
                }
                motion.Set3Axis(mainAxis, perpAxis, shaftAxis);
                //reverse main axis if needed based on user selection of ee side.
                if ((skeleton.PointAtNormalizedLength(0.5) - motionCtrlPointSelected) * mainAxis < 0)
                    mainAxis = -mainAxis;
                //double initialOffset = finalGearPositionRatio * pts[1].DistanceTo(pts[0]);
                //motion.CalculateSpaceForKineticUnit(kineticUnitDir, axelDir, axelSpace, gearSpace, unitLenth, initialOffset, finalGearPositionRatio);

                // wait for Xia, eeLineDotPt is the point passed to Xia's function 
                //motion.GenerateGearTrain(finalGearPositionRatio, eeCenPt, speed_input, kineticUnitDir, axelDir);

                // convert the inner cavity brep into box
                Box innerCavityBox = new Box(innerCavity.GetBoundingBox(true));
                //Offset inner cavity by 2mm
                innerCavityBox.Inflate(-2);

                BoundingBox innerCavityOriginalBbox = innerCavity.GetBoundingBox(true);
                double lastShaftInwardOffset = 15;
                eeCenPt = innerCavityOriginalBbox.Center + (innerCavityOriginalBbox.Diagonal * mainAxis / 2 - lastShaftInwardOffset) * mainAxis;

                // gear's facewidth is fixed for our project except for the first gear in the gear train
                List<GearTrainScheme> gear_schemes = GenerateGearTrain.GetGearTrainSchemes(mainAxis, shaftAxis, eeCenPt, innerCavityBox, 3.6,1);

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
                            if(gtp.gearSetNumber>1)
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
                    gears = helperFun.genGears(selectedGearTrainParam.parameters, motionControlMethod, 0.4,true);
                    #endregion
                    //Move add gear to later since they need to be edited
                    //motion.AddGears(gears, axel_spacer_entities, selectedGearTrainParam);
                }
                #endregion
                #region add quick return
                //Calculate the distance between last 2 shafts to know about quick return params
                double shaftDistance = selectedGearTrainParam.bullGearRadius + selectedGearTrainParam.pinionRadius + 0.3;
                double quickReturnRadiusMin = 5, quickReturnRadiusMax= shaftDistance - 8;
                double quickReturnRadius = quickReturnRadiusMin+(quickReturnRadiusMax-quickReturnRadiusMin)*rangeLevel/10;

                //Use selected gear param to figure out quick return position and direction
                GearParameter lgp = selectedGearTrainParam.parameters.Last();
                Point3d lgc = lgp.center;
                double lgcCoordinate = new Vector3d(lgc) * shaftAxis;
                double lgcCoordinateOffset = new Vector3d(lgc+lgp.norm*lgp.faceWidth) * shaftAxis;
                double spaceLeftOver = 0;
                if(lgcCoordinate<lgcCoordinateOffset)
                {
                    spaceLeftOver = shaftAxisRange.Max - lgcCoordinateOffset-0.6;

                }
                else
                {
                    spaceLeftOver = lgcCoordinateOffset - shaftAxisRange.Min-0.6;
                }
                if(spaceLeftOver<15)//TODO test this threshold to see if it's too big or too small
                {
                    warningwin.Text = "The selected body is too small to add the quick return structure!";
                    warningwin.Show();
                }
                else
                {
                    //Generate quick return and add it
                    BoundingBox bboxMid = brepCut[1].GetBoundingBox(true);
                    double sliderLen = quickReturnRadius + mainAxis * (bboxMid.Max - lgc)+10;

                    QuickReturn qr = new QuickReturn(lgc + lgp.norm * (lgp.faceWidth+0.6), lgp.norm, mainAxis, quickReturnRadius * 2, 3.6, sliderLen);
                    //Add qr models
                    Brep cw = qr.CrankWheelSolid, pin = qr.PinSolid;
                    Brep cwpin = Brep.CreateBooleanUnion(new List<Brep> { cw, pin }, myDoc.ModelAbsoluteTolerance)[0];
                    crankWheel = new Entity(cwpin, false, "Crank wheel with pin on");
                    Brep yoke = qr.YokeSolid, slider = qr.SliderSolid;
                    Brep ys = Brep.CreateBooleanUnion(new List<Brep> { yoke, slider }, myDoc.ModelAbsoluteTolerance)[0];
                    yokeSlider = new Entity(ys, false, "Yoke and slider joined");
                    stopWall = new Entity(qr.StopWallSolid, false, "Stop wall");
                    Brep bb = qr.BearingBlockSolid;
                    //Move bearing block model to edge of part2
                    BoundingBox bboxbb = bb.GetBoundingBox(true);
                    Vector3d bbTranslation = mainAxis * (mainAxis * (bboxMid.Max - bboxbb.Min));
                    bb.Transform(Transform.Translation(bbTranslation));
                    bearingBlock = new Entity(bb, false, "Bearing block");
                    motion.AddQuickReturn(crankWheel, yokeSlider, bearingBlock,stopWall,slider);

                    #region Find last shaft and replace it with 2 one sided shaft for quick return
                    Shaft lastShaft = null;
                    foreach (Entity e in axel_spacer_entities)
                    {
                        if (e.Name == "lastShaft")
                            lastShaft = (Shaft)e;
                    }
                    //Replace it
                    axel_spacer_entities.Remove(lastShaft);
                    Point3d pt1 = lastShaft.StartPt;
                    Point3d pt2 = pt1 + lastShaft.AxisDir * lastShaft.Len;
                    //Tell which one is for socket, which is for regular shaft
                    Point3d axel1Start, axel1End;
                    Point3d axel2Start, axel2End;
                    if (lastShaft.AxisDir*lgp.norm>0.99)
                    {//pt1 is for socket, pt2 is for shaft
                        axel1Start = pt1 ;
                        axel1End = lgp.center + lgp.norm * (lgp.faceWidth + 0.6 + 3.6);
                        axel2Start = pt2;
                        axel2End = lgp.center + lgp.norm * (lgp.faceWidth + 0.6 + 9.1);
                    }
                    else
                    {//pt2 is for socket,pt1 is for shaft. Use reversed direction
                        axel1Start = pt2 ;
                        axel1End = lgp.center + lgp.norm * (lgp.faceWidth + 0.6 + 3.6);
                        axel2Start = pt1;
                        axel2End = lgp.center + lgp.norm * (lgp.faceWidth + 0.6 + 9.1);
                    }
                    //Params of replaced axel, disc, revolute joint
                    
                    
                    Shaft newLastShaft1 = new Shaft(axel1Start, axel1Start.DistanceTo(axel1End), 1.5, lgp.norm);
                    Shaft newLastShaft2 = new Shaft(axel2Start, axel2Start.DistanceTo(axel2End), 1.5, -lgp.norm);
                    axel_spacer_entities.Add(newLastShaft1);
                    axel_spacer_entities.Add(newLastShaft2);
                    //Move last gear to join lg and cw
                    gears.Last().Model.Transform(Transform.Translation( lgp.norm * 0.6));
                    //Remove last 2 spacers
                    List<Spacer> generatedSpacers = new List<Spacer>();
                    for(int i=0;i<axel_spacer_entities.Count;i++)
                    {
                        Entity e = axel_spacer_entities[i];
                        if (e.GetType() == typeof(Spacer))
                            generatedSpacers.Add((Spacer)e);
                    }
                    axel_spacer_entities.Remove(generatedSpacers[generatedSpacers.Count - 1]);
                    axel_spacer_entities.Remove(generatedSpacers[generatedSpacers.Count - 2]);
                    #endregion

                    motion.AddGears(gears, axel_spacer_entities, selectedGearTrainParam);
                }
                #endregion
                #region generate spring
                spring_entities = helperFun.genSprings(selectedGearTrainParam.parameters, model, skeleton, mainAxis, motionControlMethod, strokeLevel, energyLevel, eeMovingDirectionSelection, out lockPos, out spiralLockNorm, out spiralLockDir);
                motion.AddSprings(spring_entities.ElementAt(0));
                #endregion
                toGenerateMechanism = false;
            }


            if (toAddLock)
            {
                if (motion != null)
                    motion.ConstructLocks(lockPos, spiralLockNorm, spiralLockDir, selectedGearTrainParam, motionControlMethod);
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
                    motion.CreateShell();
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
            get { return new Guid("13d90716-e581-40a2-8cd0-238d671dfd98"); }
        }
    }
}