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

namespace ConTranslation
{
    public class ContinuousTranslationModule : GH_Component
    {
        // Variables that store information
        Brep model;             // the original brep model
        Brep conBrep;           // the Brep that is selected and converted
        Brep innerCavity;
        double t1, t2; // the positoins of start point and end point of the segment on the normalized skeleton
        Curve skeleton;     // skeleton
        int speedLevel;         // value of the strength slide bar
        int distanceLevel;
        int energyLevel;
        double displacement;   // value of the distance slide bar
        Vector3d direction;             // kinetic unit direction
        ContinuousTranslation motion;
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
        Guid ArrowCurve;
        bool OperatingArrow = false;
        bool PlaneGenerated = false;
        bool ArrowGenerated = false;
        bool PlaneSelected = false;
        double arrowScale;
        Plane pl1, pl2;
        PlaneSurface s1, s2;
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
        Guid xArrowID, yArrowID, zArrowID;
        int selectedAxisIndex;
        Guid guide1, guide2;
        Guid convertedPortion;
        List<Brep> brepCut;
        Guid reserveBrepID1;
        Guid reserveBrepID2;
        Guid motionCtrlPtID1;
        Guid motionCtrlPtID2;
        Point3d motionCtrlPointSelected;

        Guid eeCircleID = Guid.Empty;
        Point3d eeCircleDotPt = new Point3d();
        Guid eeLineID = Guid.Empty;
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
        List<Entity> gear_entities = new List<Entity>();
        List<Entity> spring_entities = new List<Entity>();
        //List<GearParameter> gear_info = new List<GearParameter>();
        Point3d lockPos = new Point3d();

        /// <summary>
        /// Initializes a new instance of the ContinuousTranslationModule class.
        /// </summary>
        public ContinuousTranslationModule()
          : base("ContinuousTranslationModule", "CTModule",
              "The continuous translation kinetic unit",
              "Kinergy", "KineticUnits")
        {
            model = null;
            conBrep = new Brep();
            innerCavity = new Brep();
            t1 = 0;
            t2 = 1;
            skeleton = null;
            speedLevel = 5;
            distanceLevel = 5;
            energyLevel = 5;
            direction = new Vector3d();
            motion = null;
            lockDirCandidates = new List<Arrow>();
            p = null;

            lockState = false;
            min_wire_diamter = 2.8;
            min_coil_num = 3;
            energy = 0.5;
            speed = 4;
            displacement = 0;
            arrowScale = 0;
            isLockSet = false;
            selObjId = Guid.Empty;
            toBeBaked = new List<Guid>();

            helperFun = new Helpers();

            myDoc = RhinoDoc.ActiveDoc;
            testBodySelBtn = false;
            testMotionControlPosSetBtn = false;
            testEEPosSetBtn = false;
            testMotionAxisDirSetBtn = false;
            testPreBtn = false;
            testBakeBtn = false;
            motionControlMethod = -1;

            selectedAxisIndex = -1; // 1 - x axis, 2 - y axis, 3 - z axis 
            guide1 = Guid.Empty;
            guide2 = Guid.Empty;
            brepCut = new List<Brep>();
            convertedPortion = Guid.Empty;
            reserveBrepID1 = Guid.Empty;
            reserveBrepID2 = Guid.Empty;
            motionCtrlPtID1 = Guid.Empty;
            motionCtrlPtID2 = Guid.Empty;
            motionCtrlPointSelected = new Point3d();

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
            pManager.AddIntegerParameter("Distance", "S", "Translation distance of motion", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Energy", "E", "The energy of the spring", GH_ParamAccess.item);


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
            bool reg_input = false,  control_pos = false, ee_pos = false, motion_axis = false, addlock_input = false, pre_input = false, bake_input = false;
            int motion_control_method = -1;
            int speed_input = 5;
            int dis_input = 5;
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
            if (!DA.GetData(8, ref dis_input))
                return;
            if (!DA.GetData(9, ref energy_input))
                return;
            if (!DA.GetData(10, ref bake_input))
                return;
            #endregion

            // variables to control states
            bool toSelectRegion = false, toSetMotionControl = false, toSetEEPos = false, toSetAxisDir = false, toAdjustParam = false, toAddLock = false, toRemoveLock = false, toPreview = false, toBake = false;

            #region Input check. This determines how the cell respond to changed params
            // testBodySelBtn, testMotionControlPosSetBtn, testEEPosSetBtn, testMotionAxisDirSetBtn, testPreBtn, testBakeBtn

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

            if (speedLevel == speed_input && distanceLevel == dis_input && energyLevel == energy_input)
            {
                toAdjustParam = false;
            }
            else
            {
                speedLevel = speed_input;
                distanceLevel = dis_input;
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
                    arrowScale = box.Diagonal.Length / 100;
                    center = box.Center;

                    #endregion
                }
            }

            if (toSetMotionControl)
            {
                if (model != null)
                {
                    //Step 1: select the axis (X, Y, or Z axis)
                    selectedAxisIndex = new XYZselection(center, myDoc, redAttribute,greenAttribute,blueAttribute,30).selectedAxis;

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

                        brepCut = Helpers.cutModel(model, skeleton, t1, t2,selectedAxisVector,myDoc);
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
                    motionCtrlPointSelected = new SideSelection(skeleton, myDoc,blueAttribute).motionCtrlPointSelected;

                    #region Step 5: create an instance of Continuous Translation class

                    motion = new ContinuousTranslation(model, direction, innerCavity, motionCtrlPointSelected, speedLevel, distanceLevel, energyLevel, motionControlMethod);

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
                EndEffectorDirectionSelection directionSelection = new EndEffectorDirectionSelection(brepCut, skeleton, direction, motionCtrlPointSelected, t1, t2, myDoc,redAttribute);
                
                kineticUnitDir = directionSelection.kineticUnitDir;
                axelDir = directionSelection.axelDir;
                eeCenPt = directionSelection.eeCenPt;
                
                #region select a position along the diameter that crosses the circle center
                //Construct the cross line 
                Curve crossLineCrv = new Line(eeCenPt - axelDir * int.MaxValue, eeCenPt + axelDir * int.MaxValue).ToNurbsCurve();
                Curve[] crvs;
                Point3d[] pts;
                Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, model, myDoc.ModelAbsoluteTolerance, out crvs, out pts);
                Curve eeLineCrv = new Line(pts[0], pts[1]).ToNurbsCurve();
                //Select the point
                eeLineDotPt = new PointOnLineSelection(eeLineCrv, myDoc, redAttribute).eeLineDotPt;
                

                if ((eeLineDotPt - pts[0]) / eeLineDotPt.DistanceTo(pts[0]) == axelDir)
                {
                    finalGearPositionRatio = eeLineDotPt.DistanceTo(pts[0]) / pts[1].DistanceTo(pts[0]);
                }
                else
                {
                    finalGearPositionRatio = eeLineDotPt.DistanceTo(pts[1]) / pts[1].DistanceTo(pts[0]);
                }

                #endregion

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
                // gear's facewidth is fixed for our project except for the first gear in the gear train
                List<GearTrainScheme> gear_schemes = GenerateGearTrain.GetGearTrainSchemes(direction, axelDir, eeLineDotPt, innerCavityBox, 3.6);

                if(gear_schemes.Count == 0)
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

                    if(paramNum == -1 || schemeNum == -1)
                    {
                        return;
                    }
                    selectedGearTrainParam = gear_schemes[schemeNum].parameters[paramNum];
                    //gear_info.Clear();
                    //gear_info = gear_schemes[schemeNum].parameters[paramNum].parameters;

                    #region generate all the axels and spacers for the gears

                    axel_spacer_entities = helperFun.genAxelsStoppers(selectedGearTrainParam.parameters, model, motionControlMethod, 0.3);
                    foreach(Entity en in axel_spacer_entities)
                    {
                        motion.EntityList.Add(en);
                    }

                    #endregion

                    #region generate all the gears

                    gear_entities = helperFun.genGears(selectedGearTrainParam.parameters, motionControlMethod, 0.4);
                    foreach (Entity en in gear_entities)
                    {
                        motion.EntityList.Add(en);
                    }

                    #endregion
                }
                #endregion
            }

            if (toSetAxisDir)
            {
                #region Select End Effector Moving Direction
                // offer 3 options for user: one arrow referring to movement along the main axis, another 2 perpendicular to the main axis.
                Vector3d mainAxis =Vector3d.Unset,perpAxis;
                BoundingBox Bbox = model.GetBoundingBox(false);
                double x=0, y=0;
                switch (selectedAxisIndex)
                {
                    case 1: mainAxis = Vector3d.XAxis;x = Bbox.Max.Y - Bbox.Min.Y; y = Bbox.Max.Z - Bbox.Min.Z; break;
                    case 2: mainAxis = Vector3d.YAxis; x = Bbox.Max.X - Bbox.Min.X; y = Bbox.Max.Z - Bbox.Min.Z; break;
                    case 3: mainAxis = Vector3d.ZAxis; x = Bbox.Max.X - Bbox.Min.X; y = Bbox.Max.Y - Bbox.Min.Y; break;
                    default: break;
                }
                perpAxis = kineticUnitDir;
                perpAxis.Unitize();
                mainAxis.Unitize();
                //reverse main axis if needed based on user selection of ee side.
                if ((new Vector3d(eeCenPt) - new Vector3d(skeleton.PointAtNormalizedLength(0.5))) * mainAxis < 0)
                    mainAxis = -mainAxis;
                Vector3d otherAxis = new Plane(eeCenPt, mainAxis, perpAxis).Normal;
                otherAxis.Unitize();
                double offsetDistance = Math.Max(x, y) + 10;
                perpAxisArrowUp = GenerateDoubleArrow(eeCenPt +otherAxis*offsetDistance*0.7+perpAxis*offsetDistance/2, perpAxis,false);
                perpAxisArrowDown = GenerateDoubleArrow(eeCenPt + otherAxis * offsetDistance*0.7-perpAxis*offsetDistance/2, -perpAxis,false);
                mainAxisArrow = GenerateDoubleArrow(eeCenPt + perpAxis * offsetDistance*0.7, mainAxis);
                Rhino.Input.Custom.GetPoint gp6 = new Rhino.Input.Custom.GetPoint();
                gp6.SetCommandPrompt("Please select the desired direction for the end effector to move along.");
                gp6.MouseDown += Gp6_MouseDown;
                gp6.MouseMove += Gp6_MouseMove;
                //gp5.AcceptNothing(true);
                Rhino.Input.GetResult r6;
                r6 = gp6.Get(true);
                //do
                //{
                //    r5 = gp5.Get(true);
                //} while (r5 != Rhino.Input.GetResult.Nothing);
                myDoc.Objects.Delete(mainAxisArrow, true);
                myDoc.Objects.Delete(perpAxisArrowUp, true);
                myDoc.Objects.Delete(perpAxisArrowDown, true);
                RhinoApp.WriteLine("The selected end effector moving direction is "+eeMovingDirectionSelection);
                #endregion
                //This is the key structures to be built and the gap need to be cut
                Rack rack;
                List<Brep> constrainingStructure = new List<Brep>();
                Brep connectingStructure;//that connects rack and end effector;
                double gapMiddlePart2EE = 0;
                #region Build Rack and Constraining Structure
                
                //First calculate ee moving distance based on motion params.
                double eeMovingDistance = displacement * selectedGearTrainParam.gearRatio;
                double rackExtraLength = 10;//TODO check this const value.
                if (eeMovingDirectionSelection == 2 || eeMovingDirectionSelection == 3)//The selected moving direction is perpendicular to main direction. i.e. same as user selected orientation
                {
                    //The rack should be linked to last gear
                    //TODO find out the parameter of the last gear and the user selected position (eeLineDotPt)
                    double lastGearRadius = selectedGearTrainParam.parameters.Last().radius;
                    Point3d contactPoint = eeLineDotPt + mainAxis * lastGearRadius;
                    //Calculate rack length based on eeMovingDistance
                    double rackLength = eeMovingDistance + rackExtraLength;
                    //Create rack. It's composed with 3 parts: rack, backbone and connecting bone with ee
                    double rackFaceWidth = 3.6;
                    double rackThickness = 5;
                    double teethHeight = 1.25;
                    double backboneFacewidth = 10,backboneThickness=4;
                    double connectboneFacewidth = 5, connectboneThickness = 5;
                    Point3d contactPointRackBackPoint = contactPoint + mainAxis * (rackThickness + teethHeight/2+0.3);
                    Point3d rackStartPoint , rackEndPoint;
                    if (eeMovingDirectionSelection == 2)
                    {
                        rackStartPoint = contactPointRackBackPoint - perpAxis * eeMovingDistance;
                        rackEndPoint = contactPointRackBackPoint + perpAxis*rackExtraLength;
                    }
                    else
                    {
                        rackStartPoint = contactPointRackBackPoint + perpAxis * eeMovingDistance;
                        rackEndPoint = contactPointRackBackPoint + perpAxis*rackExtraLength;
                    }
                    Vector3d rackVector = new Vector3d(rackEndPoint) - new Vector3d(rackStartPoint);
                    Point3d rackMidPoint = rackStartPoint + rackVector / 2;
                    rack = new Rack(rackMidPoint, rackVector, -mainAxis, rackLength, 1, rackFaceWidth, otherAxis, rackThickness, 20);
                    Plane rackPlane = new Plane(rackMidPoint, otherAxis, perpAxis);
                    Box rackBackboneBox;
                    if (rackPlane.Normal * mainAxis > 0)
                        rackBackboneBox = new Box(rackPlane,new Interval(-backboneFacewidth/2,backboneFacewidth/2),new Interval(-rackLength/2,rackLength/2),new Interval(0,backboneThickness));
                    else
                        rackBackboneBox = new Box(rackPlane, new Interval(-backboneFacewidth / 2, backboneFacewidth / 2), new Interval(-rackLength / 2, rackLength / 2), new Interval(-backboneThickness, 0));
                    Brep rackBackbone = rackBackboneBox.ToBrep();
                    rack.Model = Brep.CreateBooleanUnion(new List<Brep> { rack.Model, rackBackbone },RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];//Potential risk is failed boolean operation
                    //Create holder and connectBone based on input model scale
                    //Use a line intersection to get the scale -> 2 intersection points
                    double maxScale = model.GetBoundingBox(true).Diagonal.Length*2;
                    Line line = new Line(eeLineDotPt-perpAxis*maxScale,perpAxis*maxScale*2);
                    Point3d[] intersections;
                    if(!Rhino.Geometry.Intersect.Intersection.CurveBrep(line.ToNurbsCurve(), model, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,out _, out intersections))
                    {
                        //If intersection with model fails, use inner cavity instead
                        line = new Line(eeLineDotPt - mainAxis, perpAxis);
                        Interval interval;
                        //Calculate the midpoint of axis
                        Point3d edgePoint1,edgePoint2;
                        Rhino.Geometry.Intersect.Intersection.LineBox(line, innerCavity.GetBoundingBox(true), RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out interval);
                        edgePoint1 = line.PointAt(interval.Min)+mainAxis;
                        edgePoint2 = line.PointAt(interval.Max)+mainAxis;
                        intersections = new Point3d[2];
                        intersections[0] = edgePoint1;
                        intersections[1] = edgePoint2;
                    }
                    //Use the 2 intersection points to create holder. holder has 4 parts, 2 from intersection0 to last gear. 2 from las gear to intersection1
                    #region create 4 parts of holder
                    //Figure out the ranges
                    Point3d gearSidePoint1=eeLineDotPt + perpAxis * lastGearRadius;
                    Point3d gearSidePoint2 = eeLineDotPt - perpAxis * lastGearRadius;
                    Point3d[] segmentingPoints = new Point3d[4];
                    segmentingPoints[0] = intersections[0];
                    segmentingPoints[3] = intersections[1];
                    if(intersections[0].DistanceTo(gearSidePoint1)<intersections[0].DistanceTo(gearSidePoint2))
                    {
                        segmentingPoints[1] = gearSidePoint1;
                        segmentingPoints[2] = gearSidePoint2;
                    }
                    else
                    {
                        segmentingPoints[1] = gearSidePoint2;
                        segmentingPoints[2] = gearSidePoint1;
                    }
                    //Each part of holder is formed with 3 boxes.Use a same section line to generate them
                    //Plane holderPlane = new Plane(segmentingPoints[0], otherAxis, perpAxis);
                    List<Point3d> sectionLinePts = new List<Point3d>();
                    sectionLinePts.Add(segmentingPoints[0]);
                    Point3d pt2 = segmentingPoints[0] + otherAxis * (2 + 0.3 + backboneFacewidth / 2);
                    sectionLinePts.Add(pt2);
                    Point3d pt3 = pt2 + mainAxis*(lastGearRadius + 0.3 + teethHeight / 2 + rackThickness + backboneThickness + 0.3 + 2);
                    sectionLinePts.Add(pt3);
                    Point3d pt4 = segmentingPoints[0] + mainAxis * (lastGearRadius + 0.3 + teethHeight / 2 + rackThickness + backboneThickness + 0.3 + 2) + otherAxis * (connectboneFacewidth / 2 + 0.3);
                    sectionLinePts.Add(pt4);
                    Point3d pt5 = pt4 - mainAxis * 2;
                    sectionLinePts.Add(pt5);
                    Point3d pt6 = pt5 + otherAxis * (backboneFacewidth - connectboneFacewidth) / 2;
                    Point3d pt8 = segmentingPoints[0] + mainAxis * (lastGearRadius - 0.3);//Here minus 0.3 is to make sure rack dosen't overlap with holder
                    Point3d pt7 = pt8 + otherAxis * (0.3 + backboneFacewidth / 2);
                    sectionLinePts.Add(pt6);
                    sectionLinePts.Add(pt7);
                    sectionLinePts.Add(pt8);
                    sectionLinePts.Add(segmentingPoints[0]);
                    Polyline section = new Polyline(sectionLinePts);
                    Curve sectionCurve = section.ToNurbsCurve();
                    
                    var sweep = new SweepOneRail();
                    sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
                    sweep.ClosedSweep = false;
                    sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;
                    //The first holder from seg0 to seg1
                    Curve rail1 = new Line(segmentingPoints[0], segmentingPoints[1]).ToNurbsCurve();
                    Brep[] holder1list = sweep.PerformSweep(rail1, sectionCurve);
                    Brep holder1 = holder1list[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                    //Second is a mirror of first
                    Brep holder2= holder1list[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                    holder2.Transform(Transform.Mirror(segmentingPoints[0], otherAxis));
                    //Third from seg2 to seg3
                    Curve rail3 = new Line(segmentingPoints[2], segmentingPoints[3]).ToNurbsCurve();
                    Brep[] holder3list = sweep.PerformSweep(rail3, sectionCurve);
                    Brep holder3 = holder3list[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                    //Fourth is mirrored from 3rd
                    Brep holder4 = holder3list[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                    holder4.Transform(Transform.Mirror(segmentingPoints[0], otherAxis));
                    constrainingStructure = new List<Brep> { holder1, holder2, holder3, holder4 };
                    #endregion

                    //Use the 2 inersection points to create connector
                    Vector3d intersectionVector = (new Vector3d(intersections[1]) - new Vector3d(intersections[0]));
                    Point3d intersectionMidPoint = intersections[0] +intersectionVector/ 2;
                    Box connectboneBox;
                    Plane connectbonePlane = new Plane(intersectionMidPoint+mainAxis*(lastGearRadius+ rackThickness + teethHeight / 2 + 0.3+backboneThickness), otherAxis, perpAxis);
                    if (connectbonePlane.Normal * mainAxis > 0)
                        connectboneBox = new Box(rackPlane, new Interval(-connectboneFacewidth / 2, connectboneFacewidth / 2), new Interval(-rackLength / 2, rackLength / 2), new Interval(0, connectboneThickness));
                    else
                        connectboneBox = new Box(rackPlane, new Interval(-connectboneFacewidth / 2, connectboneFacewidth / 2), new Interval(-rackLength / 2, rackLength / 2), new Interval(-connectboneThickness,0));
                    Brep connectbone = connectboneBox.ToBrep();
                    connectingStructure = connectbone;
                    gapMiddlePart2EE = lastGearRadius + 0.3 + teethHeight / 2 + rackThickness + backboneThickness + connectboneThickness;
                }
                else//The selected moving direction is exactly the main direction, so the rack would get into the model
                {
                    GearParameter lgp = selectedGearTrainParam.parameters.Last();
                    double lastGearRadius = lgp.radius;
                    Point3d contactPoint = eeLineDotPt + perpAxis * lastGearRadius;//TODO select up or down based on gear position!
                    //Calculate rack length based on eeMovingDistance and inner cavity space
                    rackExtraLength = Math.Max(rackExtraLength, lgp.radius + 2);//Make sure rack extra length can outgrow last gear
                    double rackLength = eeMovingDistance + rackExtraLength;
                    double innerCavityMainAxisLength = 0;
                    Box innerCavityBox = new Box(innerCavity.GetBoundingBox(true));
                    switch (selectedAxisIndex)
                    {
                        case 1: innerCavityMainAxisLength = innerCavityBox.X.Length; break;
                        case 2: innerCavityMainAxisLength = innerCavityBox.Y.Length; ; break;
                        case 3: innerCavityMainAxisLength = innerCavityBox.Z.Length; break;
                        default: break;
                    }
                    double rackFaceWidth = 3.6;
                    double rackThickness = 5;
                    double teethHeight = 1.25;
                    double rackHolderWidth = 2;
                    double rackCompoundThickness = rackFaceWidth + 0.3 * 2 + rackHolderWidth * 2;
                    //First just use the end tip of the last gear for the rack contact
                    Point3d lgctAtEnd = lgp.center + lgp.norm * lgp.faceWidth;
                    Point3d rackContactPoint = lgctAtEnd - lgp.radius * perpAxis-lgp.norm*rackCompoundThickness/2;
                    Point3d rackContactPointOnRackBack = rackContactPoint - perpAxis * (0.3 + teethHeight / 2 + rackThickness);
                    Point3d rackStart = rackContactPointOnRackBack + mainAxis * rackExtraLength;
                    Point3d rackEnd = rackContactPointOnRackBack - mainAxis * eeMovingDistance;
                    Vector3d rackVector = new Vector3d(rackEnd) - new Vector3d(rackStart);
                    Point3d rackMidPoint = rackStart + rackVector / 2;
                    rack = new Rack(rackMidPoint, rackVector, -perpAxis, rackLength, 1, rackFaceWidth, otherAxis, rackThickness, 20);
                    //Then make the confining structure, which are 2 bars beside rack and some caps. bars are higher than rack by 0.3 and capped at rack end and farthest position
                    Plane rackPlane = new Plane(rackContactPointOnRackBack, mainAxis, otherAxis);
                    
                    Box bar1Box=new Box(rackPlane,new Interval(-innerCavityMainAxisLength,-(rackExtraLength-2))
                        ,new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, -rackFaceWidth/2-0.3),new Interval(0, 0.3 + teethHeight / 2 + rackThickness));
                    Box bar2Box = new Box(rackPlane, new Interval(-innerCavityMainAxisLength, -(rackExtraLength - 2))
                        , new Interval(rackFaceWidth / 2 + 0.3 , rackFaceWidth / 2 + 0.3+ rackHolderWidth), new Interval(0, 0.3 + teethHeight / 2 + rackThickness));
                    //Add cap
                    Box cap1Box = new Box(rackPlane, new Interval(-rackExtraLength, -(rackExtraLength - 2)),
                        new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, rackFaceWidth / 2 + 0.3 + rackHolderWidth),
                        new Interval(0.3 + teethHeight / 2 + rackThickness, 0.3 + teethHeight / 2 + rackThickness + 2));
                    Box cap2Box = new Box(rackPlane, new Interval(-eeMovingDistance, -(eeMovingDistance - 2)),
                        new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, rackFaceWidth / 2 + 0.3 + rackHolderWidth),
                        new Interval(0.3 + teethHeight / 2 + rackThickness, 0.3 + teethHeight / 2 + rackThickness + 2));
                    Box cap3Box = new Box(rackPlane, new Interval(-innerCavityMainAxisLength, -(innerCavityMainAxisLength - 2)),
                        new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, rackFaceWidth / 2 + 0.3 + rackHolderWidth),
                        new Interval(0.3 + teethHeight / 2 + rackThickness, 0.3 + teethHeight / 2 + rackThickness + 2));
                    if (rackPlane.Normal*perpAxis<0)
                    {
                        bar1Box.Transform(Transform.Mirror(rackPlane));
                        bar2Box.Transform(Transform.Mirror(rackPlane));
                        cap1Box.Transform(Transform.Mirror(rackPlane));
                        cap2Box.Transform(Transform.Mirror(rackPlane));
                        cap3Box.Transform(Transform.Mirror(rackPlane));
                    }
                    Brep bar1 = bar1Box.ToBrep(), bar2 = bar2Box.ToBrep();
                    Brep cap1 = cap1Box.ToBrep(), cap2 = cap2Box.ToBrep(), cap3 = cap3Box.ToBrep();
                    Brep union = Brep.CreateBooleanUnion(new List<Brep> { bar1, cap1, cap2, cap3, bar2 }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                    constrainingStructure.Add(union);
                    gapMiddlePart2EE = rackExtraLength;
                    #region Deal with the case when rack is longer than inner cavity. Just dig a hole out
                    //Just use a box to make space for rack.
                    Brep cuttingBox = new Box(rackPlane, new Interval(-eeMovingDistance, rackExtraLength), new Interval(-0.3 - rackFaceWidth / 2, 0.3 + rackFaceWidth / 2),
                        new Interval(-0.3, 0.3 + teethHeight / 2 + rackThickness)).ToBrep();
                    //TODO cut model mid part and start part with this box 

                    //if (innerCavityMainAxisLength < eeMovingDistance)
                    //{
                        //Then we need to use bounding box of rack to boolean main model

                    //}
                    #endregion
                }
                #endregion

                #region generate the spring

                if (motionControlMethod == 1)
                {
                    // helical spring

                }
                else
                {
                    // spiral spring

                }
                
                spring_entities = helperFun.genSprings(selectedGearTrainParam.parameters, model, motionControlMethod, distanceLevel, energyLevel, eeMovingDirectionSelection, out lockPos);


                #endregion
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

                #region gear test
                //参数列表：3个向量，分别是物体向量，轴向量，以及垂直于这两个向量的方向；最后一个齿轮的位置，123两个方向的空间大小
                //bool isGroove = false;
                //int numTeeth = 9;
                //double stepAngle = 360.0 / numTeeth;
                //double boundary = 90.0 / stepAngle;
                //int floorNum = (int)Math.Floor(boundary);
                //int ceilingNum = (int)Math.Ceiling(boundary);
                //double selfRotationAngle = 0;

                //if (floorNum == ceilingNum)
                //{
                //    // the mating tooth is actually symmetric around the X axis
                //    selfRotationAngle = stepAngle / 2;
                //}
                //else
                //{
                //    double leftoverAngle = 90 - stepAngle * floorNum;
                //    selfRotationAngle = stepAngle / 2 - leftoverAngle;
                //}
                //isGroove = true;

                //Gear temp = new Gear(new Point3d(0, 0, 0), new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), numTeeth, 1, 20, 3.6, selfRotationAngle, true);

                //numTeeth = 20;
                //if (isGroove)
                //{
                //    selfRotationAngle = 90;
                //    if (numTeeth % 2 == 1)
                //    {
                //        isGroove = true;
                //    }
                //    else
                //    {
                //        isGroove = false;
                //    }
                //}
                //else
                //{
                //    stepAngle = 360.0 / numTeeth;
                //    selfRotationAngle = 90 - stepAngle / 2;

                //    if (numTeeth % 2 == 1)
                //    {
                //        isGroove = false;
                //    }
                //    else
                //    {
                //        isGroove = true;
                //    }

                //}
                //Gear temp1 = new Gear(new Point3d(14.8, 0, 0), new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), numTeeth, 1, 20, 3.6, selfRotationAngle, true);

                //numTeeth = 45;
                //if (isGroove)
                //{
                //    selfRotationAngle = 90;
                //    if (numTeeth % 2 == 1)
                //    {
                //        isGroove = true;
                //    }
                //    else
                //    {
                //        isGroove = false;
                //    }
                //}
                //else
                //{
                //    stepAngle = 360.0 / numTeeth;
                //    selfRotationAngle = 90 - stepAngle / 2;

                //    if (numTeeth % 2 == 1)
                //    {
                //        isGroove = false;
                //    }
                //    else
                //    {
                //        isGroove = true;
                //    }

                //}
                //Gear temp2 = new Gear(new Point3d(14.8 + 0.4 + 32.5, 0, 0), new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), numTeeth, 1, 20, 3.6, selfRotationAngle, true);


                //////// the tolerance between two mating gears is 0.4

                //myDoc.Objects.AddBrep(temp.Model);
                //myDoc.Views.Redraw();
                //myDoc.Objects.AddBrep(temp1.Model);
                //myDoc.Views.Redraw();
                //myDoc.Objects.AddBrep(temp2.Model);
                //myDoc.Views.Redraw();

                #endregion

                #region rack test

                //bool isGroove = false;
                //int numTeeth = 9;
                //double stepAngle = 360.0 / numTeeth;
                //double boundary = 90.0 / stepAngle;
                //int floorNum = (int)Math.Floor(boundary);
                //int ceilingNum = (int)Math.Ceiling(boundary);
                //double selfRotationAngle = 0;

                //if (floorNum == ceilingNum)
                //{
                //    // the mating tooth is actually symmetric around the X axis
                //    selfRotationAngle = stepAngle / 2;
                //}
                //else
                //{
                //    double leftoverAngle = 90 - stepAngle * floorNum;
                //    selfRotationAngle = stepAngle / 2 - leftoverAngle;
                //}
                //isGroove = true;

                //Gear temp = new Gear(new Point3d(0, 0, 0), new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), numTeeth, 1, 20, 3.6, selfRotationAngle, true);
                //myDoc.Objects.AddBrep(temp.Model);
                //myDoc.Views.Redraw();

                //Rack tempRack = new Rack(new Point3d(50, 0, 0), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0), 90, 1, 3.6, new Vector3d(0, 0, 1), 3, 20);
                //myDoc.Objects.AddBrep(tempRack.Model);
                //myDoc.Views.Redraw();

                //Spacer spacer = new Spacer(new Point3d(0, 0, 0), 1, 2.2, 3, new Vector3d(0, 0, 1));
                //myDoc.Objects.AddBrep(spacer.Model);
                //myDoc.Views.Redraw();
                #endregion

                #region geneva drive test

                //GenevaDrive tempGD = new GenevaDrive(new Point3d(0, 0, 0), 6, new Vector3d(0, 0, 1), 3.6, new Vector3d(1, 0, 0));

                //foreach (Brep b in tempGD.GenevaModels)
                //{
                //    myDoc.Objects.AddBrep(b);
                //    myDoc.Views.Redraw();
                //}

                #endregion

                #region quick-return test

                //QuickReturn tempQR = new QuickReturn(new Point3d(0, 0, 0), new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), 20, 3.6, 80);
                //QuickReturn tempQR1 = new QuickReturn(new Point3d(0, -100, 0), new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), 20, 3.6, 80);

                //foreach (Brep b in tempQR.QuickReturnModels)
                //{
                //    myDoc.Objects.AddBrep(b);
                //    myDoc.Views.Redraw();
                //}

                //foreach (Brep b in tempQR1.QuickReturnModels)
                //{
                //    myDoc.Objects.AddBrep(b);
                //    myDoc.Views.Redraw();
                //}

                #endregion

                #region crank and slotted lever

                //CrankSlottedLever tempCSL = new CrankSlottedLever(new Point3d(0, 0, 0), new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), 15, 35);

                //foreach (Brep b in tempCSL.CrankSlottedLeverModels)
                //{
                //    myDoc.Objects.AddBrep(b);
                //    myDoc.Views.Redraw();
                //}

                #endregion
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

        
        private Guid GenerateDoubleArrow(Point3d arrowCenter,Vector3d direction,bool isDouble=true)
        {
            double axisRadius = 2;
            Point3d EndPt1 = arrowCenter + direction * 30;
            Point3d EndPt2 = arrowCenter - direction * 30;
            Brep arrow = null;
            if (isDouble)
            {
                Line Ln = new Line(EndPt1, EndPt2);
                Curve Crv = Ln.ToNurbsCurve();
                Brep AxisBrep = Brep.CreatePipe(Crv, axisRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
                Plane ArrowPln1 = new Plane(EndPt1+direction*4.5, -direction);
                Plane ArrowPln2 = new Plane(EndPt2-direction*4.5, direction);
                Cone ArrowTipCone1 = new Cone(ArrowPln1, 5, 2 * axisRadius);
                Cone ArrowTipCone2 = new Cone(ArrowPln2, 5, 2 * axisRadius);
                Brep ArrowTipBrep1 = ArrowTipCone1.ToBrep(true);
                Brep ArrowTipBrep2 = ArrowTipCone2.ToBrep(true);
                arrow = Brep.CreateBooleanUnion(new List<Brep> { AxisBrep, ArrowTipBrep1, ArrowTipBrep2 }, myDoc.ModelAbsoluteTolerance)[0];
            }
            else 
            {
                Line Ln = new Line(arrowCenter, EndPt1);
                Curve Crv = Ln.ToNurbsCurve();
                Brep AxisBrep = Brep.CreatePipe(Crv, axisRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
                Plane ArrowPln1 = new Plane(EndPt1+direction*4.5, -direction);
                Cone ArrowTipCone1 = new Cone(ArrowPln1, 5, 2 * axisRadius);
                Brep ArrowTipBrep1 = ArrowTipCone1.ToBrep(true);
                arrow = Brep.CreateBooleanUnion(new List<Brep> { AxisBrep, ArrowTipBrep1}, myDoc.ModelAbsoluteTolerance)[0];
            }

            Guid ID = myDoc.Objects.AddBrep(arrow, redAttribute);
            myDoc.Views.Redraw();
            return ID;
        }

        
        private void Gp6_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep perpBrepUp = (Brep)myDoc.Objects.Find(perpAxisArrowUp).Geometry;
            Brep perpBrepDown = (Brep)myDoc.Objects.Find(perpAxisArrowDown).Geometry;
            Brep mainBrep = (Brep)myDoc.Objects.Find(mainAxisArrow).Geometry;

            double perp_dis_up = perpBrepUp.ClosestPoint(currPos).DistanceTo(currPos);
            double perp_dis_down = perpBrepDown.ClosestPoint(currPos).DistanceTo(currPos);
            double main_dis = mainBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if (perp_dis_up <= main_dis && perp_dis_up <= perp_dis_down)
            {
                myDoc.Objects.UnselectAll();
                myDoc.Objects.Select(perpAxisArrowUp);
            }
            else if (  main_dis<=perp_dis_up && main_dis <= perp_dis_down)
            {
                myDoc.Objects.UnselectAll();
                myDoc.Objects.Select(mainAxisArrow);
            }
            else if (perp_dis_down <= perp_dis_up && perp_dis_down <= main_dis)
            {
                myDoc.Objects.UnselectAll();
            }
            else
            {
                myDoc.Objects.UnselectAll();
            }
        }

        private void Gp6_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep perpBrepUp = (Brep)myDoc.Objects.Find(perpAxisArrowUp).Geometry;
            Brep perpBrepDown = (Brep)myDoc.Objects.Find(perpAxisArrowDown).Geometry;
            Brep mainBrep = (Brep)myDoc.Objects.Find(mainAxisArrow).Geometry;

            double perp_dis_up = perpBrepUp.ClosestPoint(currPos).DistanceTo(currPos);
            double perp_dis_down = perpBrepDown.ClosestPoint(currPos).DistanceTo(currPos);
            double main_dis = mainBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if (perp_dis_up <= main_dis && perp_dis_up <= perp_dis_down)
            {
                eeMovingDirectionSelection = 2;
            }
            else if (main_dis <= perp_dis_up && main_dis <= perp_dis_down)
            {
                eeMovingDirectionSelection = 1;
            }
            else if(perp_dis_down <= perp_dis_up && perp_dis_down <= main_dis)
            {
                eeMovingDirectionSelection = 3;
            }
            else
            {
                eeMovingDirectionSelection = -1;
            }
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
            get { return new Guid("ec543f91-df12-44f0-ba4a-b478e976c1a9"); }
        }
    }
}