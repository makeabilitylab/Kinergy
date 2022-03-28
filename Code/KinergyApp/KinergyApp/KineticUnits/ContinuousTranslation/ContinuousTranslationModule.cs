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

        List<double> gr_list = new List<double>();
        List<GearTrainScheme> gear_schemes = new List<GearTrainScheme>();
        Point3d rackPos = new Point3d();
        Brep socketBrep = null;

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
            brepCut = new List<Brep>();
            convertedPortion = Guid.Empty;
            reserveBrepID1 = Guid.Empty;
            reserveBrepID2 = Guid.Empty;
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

                    motion = new ContinuousTranslation(model, selectedAxisIndex,direction, innerCavity, motionCtrlPointSelected, speedLevel, distanceLevel, energyLevel, motionControlMethod, helperFun);

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
                //Xia's note: moved eeCenPt by 2mm inward.

                
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
                //Offset inner cavity by 2mm
                innerCavityBox.Inflate(-2);
                // gear's facewidth is fixed for our project except for the first gear in the gear train
                gear_schemes = GenerateGearTrain.GetGearTrainSchemes(direction, axelDir, eeLineDotPt, innerCavityBox, 3.6);

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
                    #endregion

                    #region generate all the gears
                    gears = helperFun.genGears(selectedGearTrainParam.parameters, motionControlMethod, 0.4);
                    #endregion
                    motion.AddGears(gears,axel_spacer_entities, selectedGearTrainParam);
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
                Rhino.Input.Custom.GetPoint gp = new Rhino.Input.Custom.GetPoint();
                gp.SetCommandPrompt("Please select the desired direction for the end effector to move along.");
                gp.MouseDown += Gp_MouseDown;
                gp.MouseMove += Gp_MouseMove;
                //gp5.AcceptNothing(true);
                Rhino.Input.GetResult r;
                r = gp.Get(true);
                //do
                //{
                //    r5 = gp5.Get(true);
                //} while (r5 != Rhino.Input.GetResult.Nothing);
                myDoc.Objects.Delete(mainAxisArrow, true);
                myDoc.Objects.Delete(perpAxisArrowUp, true);
                myDoc.Objects.Delete(perpAxisArrowDown, true);
                RhinoApp.WriteLine("The selected end effector moving direction is "+eeMovingDirectionSelection);
                #endregion

                #region generate the spring
                GearParameter lgp = selectedGearTrainParam.parameters.Last();
                rackPos = lgp.center + lgp.norm * lgp.faceWidth;

                if(eeMovingDirectionSelection == 1)
                    spring_entities = helperFun.genSprings(selectedGearTrainParam.parameters, model, skeleton, mainAxis, motionControlMethod, distanceLevel, energyLevel, eeMovingDirectionSelection, out lockPos, out spiralLockNorm, out spiralLockDir, out socketBrep, gears.ElementAt(0), rackPos);
                else
                    spring_entities = helperFun.genSprings(selectedGearTrainParam.parameters, model, skeleton, mainAxis, motionControlMethod, distanceLevel, energyLevel, eeMovingDirectionSelection, out lockPos, out spiralLockNorm, out spiralLockDir, out socketBrep, gears.ElementAt(0));
                motion.AddSprings(spring_entities);

                #endregion

                //Calculate output displacement!
                double eeMovingDistance = 0;
                if (motionControlMethod == 1)
                {
                    // helical spring
                    Helix h=(Helix)spring_entities[0];
                    eeMovingDistance= h.CompressionDistance*selectedGearTrainParam.gearRatio;
                }
                else
                {
                    // spiral spring
                    Spiral s = (Spiral)spring_entities[0];
                    eeMovingDistance = s.MaxRevolution * selectedGearTrainParam.spiralGearRatio * selectedGearTrainParam.pinionRadius;
                }

                //This is the key structures to be built and the gap need to be cut
                
                motion.BuildEndEffectorRack(eeMovingDistance, selectedGearTrainParam, eeMovingDirectionSelection, eeLineDotPt, mainAxis, perpAxis, otherAxis);

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
                //Remove all 3 cut parts to ake space for actual shells
                myDoc.Objects.Delete(reserveBrepID1,true);
                myDoc.Objects.Delete(reserveBrepID2, true);
                myDoc.Objects.Delete(convertedPortion, true);
                if(motion!=null)
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
                //myDoc.Objects.AddPoints(temp.TeethTips);
                //myDoc.Objects.AddPoints(temp1.TeethTips);
                //myDoc.Objects.AddPoints(temp2.TeethTips);
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
                //List<Point3d> pts = tempRack.GetTipsBtms();
                //myDoc.Objects.AddPoints(pts);
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
                if (motion != null)
                    motion.AdjustParameter(eeMovingDirectionSelection, speedLevel, distanceLevel, energyLevel, gr_list, gear_schemes, spiralLockNorm, spring_entities.ElementAt(0), motionControlMethod, ref lockPos, ref spiralLockNorm, ref spiralLockDir, rackPos);

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

        
        private void Gp_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
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

        private void Gp_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
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