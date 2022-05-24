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

namespace InterOscillation
{
    public class IntermittentOscillationModule : GH_Component
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
        IntermittentOscillation motion;
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
        Vector3d mainAxis;
        Vector3d shaftAxis;
        Vector3d otherAxis;
        Vector3d oscillationMidVector;
        Brep socketBrep = null;
        bool dirControl = false;
        /// <summary>
        /// Initializes a new instance of the IntermittentOscillation class.
        /// </summary>
        public IntermittentOscillationModule()
          : base("IntermittentOscillation", "IOModule",
              "The kinetic unit for intermittent oscillation",
              "Kinergy", "KineticUnits")
        {
            model = null;
            conBrep = new Brep();
            innerCavity = new Brep();
            t1 = 0;
            t2 = 1;
            skeleton = null;
            speedLevel = 5;
            rangeLevel = 5;
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
            speed = 0.5;
            isLockSet = false;
            selObjId = Guid.Empty;
            toBeBaked = new List<Guid>();

            myDoc = RhinoDoc.ActiveDoc;
            testBodySelBtn = false;
            testMotionControlPosSetBtn = false;
            testEEPosSetBtn = false;
            testMotionAxisDirSetBtn = false;
            testPreBtn = false;
            testBakeBtn = false;
            motionControlMethod = -1;
            brepCut = new List<Brep>();
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
            pManager.AddIntegerParameter("Range", "R", "The range of the oscillation", GH_ParamAccess.item);

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
            int range_input = 5;
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
            if (!DA.GetData(8, ref range_input))
                return;
            if (!DA.GetData(9, ref bake_input))
                return;
            if (!DA.GetData(10, ref stroke_input))
                return;
            if (!DA.GetData(11, ref energy_input))
                return;
            #endregion

            // variables to control states
            bool toSelectRegion = false, toAdjustParam = false, toSetMotionControl = false, toSetEEPos = false, toSetAxisDir = false, toAddLock = false, toRemoveLock = false, toPreview = false, toBake = false;
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

            if (speedLevel == speed_input && rangeLevel == range_input && strokeLevel == stroke_input && energyLevel == energy_input)
            {
                toAdjustParam = false;
            }
            else
            {
                speedLevel = speed_input;
                rangeLevel = range_input;
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

                    motion = new IntermittentOscillation(model,selectedAxisIndex,direction,innerCavity,motionCtrlPointSelected, motionControlMethod,energy_input,stroke_input,speed_input);

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
                //get the main axis
                mainAxis = Vector3d.Unset;
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
                shaftAxis = axelDir;
                otherAxis = kineticUnitDir;
                motion.Set3Axis(mainAxis, kineticUnitDir, axelDir);
                motion.SetEndEffector(eeModel);
                //toGenerateMechanism = true;

            }
            if(toSetAxisDir)
            {
                //Let user select oscillation direction center line
                double lgcInwardOffset = 15;
                Point3d lastShaftPos = eeCenPt - mainAxis * lgcInwardOffset;
                double sideOffsetDistance = 18;
                double defaultCrankWheelRadius = 12;
                //Generate candidate points and tell if they are within model
                double positiveAngleRange = 0, negativeAngleRange = 0;
                for(double i=0;i<=90;i++)
                {
                    Point3d anchorCenter = lastShaftPos + axelDir * sideOffsetDistance - mainAxis * defaultCrankWheelRadius * 1.5;
                    Transform rotation = Transform.Rotation(i / 180 * Math.PI, axelDir, lastShaftPos);
                    anchorCenter.Transform(rotation);
                    Sphere s = new Sphere(anchorCenter, 2);
                    //myDoc.Objects.AddSphere(s);
                    //myDoc.Views.Redraw();
                    Brep[] result = Brep.CreateBooleanDifference(brepCut[1], s.ToBrep(), myDoc.ModelAbsoluteTolerance);
                    if (result.Count()>0)
                        break;
                    positiveAngleRange = i;
                }
                for (double i = 0; i >= -90; i--)
                {
                    Point3d anchorCenter = lastShaftPos + axelDir * sideOffsetDistance - mainAxis * defaultCrankWheelRadius * 1.5;
                    Transform rotation = Transform.Rotation(i / 180 * Math.PI, axelDir, lastShaftPos);
                    anchorCenter.Transform(rotation);
                    Sphere s = new Sphere(anchorCenter, 2);
                    //myDoc.Objects.AddSphere(s);
                    //myDoc.Views.Redraw();
                    Brep[] result = Brep.CreateBooleanDifference(brepCut[1], s.ToBrep(), myDoc.ModelAbsoluteTolerance);
                    if (result.Count() > 0)
                        break;
                    negativeAngleRange = i;
                }
                //Then let user select direction
                RangedCircularDirectionSelecton selection = new RangedCircularDirectionSelecton(lastShaftPos, mainAxis, shaftAxis, otherAxis, 
                    new Interval(negativeAngleRange, positiveAngleRange),myDoc, redAttribute);
                oscillationMidVector = selection.selectedVector;
                toGenerateMechanism = true;
            }
            if(toGenerateMechanism)
            {
                //Then generate gears , crank and more. TODO change from CT to IO
                #region generate transmission mechanism, and the mechanism mating the end-effector
                
                Box innerCavityBox = new Box(innerCavity.GetBoundingBox(true));
                //Offset inner cavity by 2mm
                innerCavityBox.Inflate(-2);
                // gear's facewidth is fixed for our project except for the first gear in the gear train
                double lgcInwardOffset =18;
                //myDoc.Objects.AddBox(innerCavityBox, blueAttribute);
                //myDoc.Objects.AddPoint(eeCenPt - mainAxis * lgcInwardOffset, blueAttribute);
                //myDoc.Views.Redraw();
                List<GearTrainScheme> gear_schemes = GenerateGearTrain.GetGearTrainSchemes(direction, axelDir, eeCenPt-mainAxis*lgcInwardOffset, innerCavityBox, 3.6,motionControlMethod,1);

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

                    Vector3d perVec = Vector3d.CrossProduct(direction, axelDir);
                    perVec.Unitize();
                    double gw_dis = Math.Abs(innerCavityBox.BoundingBox.Diagonal * perVec / 2);
                    List<double> gr_list = new List<double>();
                    foreach (GearTrainScheme gts in gear_schemes)
                    {
                        foreach (GearTrainParam gtp in gts.parameters)
                        {
                            if (motionControlMethod == 1)
                            {
                                if ((gtp.pinionRadius + 0.6 + 1.125 + 2 + 2 + 0.4) <= gw_dis && gtp.gearSetNumber > 1)
                                    gr_list.Add(gtp.gearRatio);
                            }
                            else
                            {
                                if ( gtp.gearSetNumber > 1 && gtp.gearSetNumber <=3)
                                    gr_list.Add(gtp.gearRatio);
                            }     

                            //if (gtp.pinionRadius > lgcInwardOffset)
                            //    continue;
                            ////if(gtp.bullGearRadius+gtp.pinionRadius+0.3-1.5>crankRadius*1.5+3.5 && gtp.gearSetNumber>1
                            //if (motionControlMethod==2 && gtp.gearSetNumber <= 1)//When motion control method is turn, the gear set number should be more than 1 so anchor shaft wouldn't conflict with spiral
                            //    continue;
                            //else
                            //    gr_list.Add(gtp.gearRatio);
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
                    axel_spacer_entities = helperFun.genAxelsStoppers(selectedGearTrainParam.parameters, model, motionControlMethod);
                    #endregion

                    #region generate all the gears
                    gears = helperFun.genGears(selectedGearTrainParam.parameters, motionControlMethod, 0.4, true);
                    #endregion
                    #region generate and add crank slotted lever
                    double crankRadiusMax = Math.Min(lgcInwardOffset,(selectedGearTrainParam.pinionRadius+selectedGearTrainParam.bullGearRadius-2-3.5)/1.5);
                    double crankRadiusMin = 5;
                    double crankRadius = crankRadiusMin + (crankRadiusMax - crankRadiusMin) * rangeLevel / 10;
                    GearParameter lgp = selectedGearTrainParam.parameters.Last();
                    Point3d eeCenter = eeModel.GetBoundingBox(true).Center;
                    Vector3d leverVec = eeCenter - lgp.center;
                    leverVec.Unitize();
                    double leverLen = (eeCenter - lgp.center) * oscillationMidVector / (leverVec * oscillationMidVector) / (leverVec * oscillationMidVector);
                    CrankSlottedLever CSL = new CrankSlottedLever(lgp.center + lgp.norm * lgp.faceWidth, lgp.norm, oscillationMidVector, crankRadius, leverLen);
                    
                    motion.AddCrankSlottedLever(CSL);
                    #endregion
                    #region Adjust last shaft
                    Shaft lastShaft = null;
                    foreach (Entity e in axel_spacer_entities)
                    {
                        if (e.Name == "lastShaft")
                            lastShaft = (Shaft)e;
                    }
                    Point3d lastShaftStart = lastShaft.StartPt;
                    Point3d lastShaftEnd = lastShaft.StartPt + lastShaft.AxisDir * lastShaft.Len;
                    axel_spacer_entities.Remove(lastShaft);
                    Point3d shaft1Start = Point3d.Unset;
                    Point3d shaft1End = Point3d.Unset;
                    Point3d shaft2Start = Point3d.Unset;
                    Point3d shaft2End = Point3d.Unset;
                    Vector3d shaft1Dir = Vector3d.Unset;
                    Vector3d shaft2Dir = Vector3d.Unset;
                    if (lastShaft.AxisDir*lgp.norm>0)
                    {
                        shaft1Start = lastShaftStart;
                        shaft1End = lgp.center + lgp.norm * (lgp.faceWidth + 3.6 + 0.3 + 1);
                        shaft2Start = lastShaftEnd;
                        shaft2End= lgp.center + lgp.norm * (lgp.faceWidth + 8.1);
                        shaft1Dir = lgp.norm;
                        shaft2Dir = -lgp.norm;
                    }
                    else
                    {
                        shaft1Start = lastShaftEnd;
                        shaft1End = lgp.center + lgp.norm * (lgp.faceWidth + 3.6 + 0.3 + 1);
                        shaft2Start = lastShaftStart;
                        shaft2End = lgp.center + lgp.norm * (lgp.faceWidth + 8.1);
                        shaft1Dir = -lgp.norm;
                        shaft2Dir = lgp.norm;
                    }
                    Shaft shaft1 = new Shaft(shaft1Start, shaft1Start.DistanceTo(shaft1End), 2, shaft1Dir);
                    Shaft shaft2 = new Shaft(shaft2Start, shaft2Start.DistanceTo(shaft2End), 2, shaft2Dir);
                    axel_spacer_entities.Add(shaft1);
                    axel_spacer_entities.Add(shaft2);
                    //Adjust spacer position
                    List<Spacer> generatedSpacers = new List<Spacer>();
                    for (int i = 0; i < axel_spacer_entities.Count; i++)
                    {
                        Entity e = axel_spacer_entities[i];
                        if (e.GetType() == typeof(Spacer))
                            generatedSpacers.Add((Spacer)e);
                    }
                    if ((generatedSpacers[generatedSpacers.Count - 1].StartPt - lgp.center) * lgp.norm > 0)
                    {
                        generatedSpacers[generatedSpacers.Count - 1].Model.Transform(Transform.Translation(lgp.norm * 3.6));
                    }
                    else
                    {
                        generatedSpacers[generatedSpacers.Count - 2].Model.Transform(Transform.Translation(lgp.norm * 3.6));
                    }
                    //Add a new shaft for lever
                    Point3d anchorCenter = CSL.AnchorCenter+lgp.norm*2.2;
                    Curve crossLineCrv = new Line(anchorCenter - axelDir * int.MaxValue, anchorCenter + axelDir * int.MaxValue).ToNurbsCurve();
                    Curve[] crvs;
                    Point3d[] pts;
                    Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, model, myDoc.ModelAbsoluteTolerance, out crvs, out pts);
                    
                    Point3d ptEnd = new Point3d();
                    Point3d ptStart = new Point3d();
                    Vector3d intersectVec = (pts[1] - pts[0]);
                    intersectVec.Unitize();
                    if (intersectVec * lgp.norm > 0.99)
                    {
                        ptEnd = anchorCenter-lgp.norm*1.25;
                        ptStart = pts[1] - lgp.norm * 1;
                    }
                    else
                    {
                        ptEnd = anchorCenter - lgp.norm * 1.25;
                        ptStart = pts[0] - lgp.norm * 1;
                    }
                    Shaft anchorShaft = new Shaft(ptStart, ptStart.DistanceTo(ptEnd), 1.5, -lgp.norm);
                    //Add 2 spacer for anchor shaft
                    Spacer s1 = new Spacer(ptEnd, 1, 2.2, 3.0, lgp.norm);
                    Spacer s2 = new Spacer(ptEnd+lgp.norm*(2+1.3+0.25), 1, 2.2, 3.0, lgp.norm);
                    axel_spacer_entities.Add(s1);
                    axel_spacer_entities.Add(s2);
                    axel_spacer_entities.Add(anchorShaft);
                    #endregion
                    motion.AddGears(gears, axel_spacer_entities, selectedGearTrainParam);

                    eeMovingDirectionSelection = 1; // 1:CW, 3: CCW
                                                    //TODO check the param input
                    if (selectedGearTrainParam.gearSetNumber % 2 == 1)
                    {
                        // output direction is opposite with the input direction
                        dirControl = false;
                        //motion.Old_direction = dirControl;
                    }
                    else
                    {
                        // output direction is the same with the input direction
                        dirControl = true;
                        //motion.Old_direction = dirControl;
                    }

                    spring_entities = helperFun.genSprings(selectedGearTrainParam.parameters, model, skeleton, mainAxis, motionControlMethod, strokeLevel, energyLevel, dirControl, out lockPos, out spiralLockNorm, out spiralLockDir, out socketBrep, gears.ElementAt(0));
                    motion.AddSprings(spring_entities);
                }
                #endregion
                toGenerateMechanism = false;
            }

            if (toAddLock)
            {
                if (motion != null)
                    motion.ConstructLocks(lockPos, spiralLockNorm, spiralLockDir, selectedGearTrainParam, spring_entities, motionControlMethod);
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
                {
                    motion.CreateShell(socketBrep);
                }
                    
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
            get { return new Guid("2ac231f7-6aef-4ec3-807a-75b045ab6b78"); }
        }
    }
}