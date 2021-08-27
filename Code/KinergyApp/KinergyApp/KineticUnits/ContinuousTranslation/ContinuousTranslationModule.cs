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
        int distance;   // value of the distance slide bar
        Vector3d direction;             // kinetic unit direction
        ContinuousTranslation motion;
        List<Arrow> lockDirCandidates;
        Arrow p;

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

        ObjectAttributes solidAttribute, orangeAttribute, redAttribute, blueAttribute, greenAttribute;
        Guid xArrowID, yArrowID, zArrowID;
        int selectedAxis;
        Guid guide1, guide2;
        Guid convertedPortion;
        List<Brep> brepCut;
        Guid reserveBrepID1;
        Guid reserveBrepID2;
        Guid motionCtrlPtID1;
        Guid motionCtrlPtID2;
        Point3d motionCtrlPointSelected;

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
            direction = new Vector3d();
            motion = null;
            lockDirCandidates = new List<Arrow>();
            p = null;

            lockState = false;
            min_wire_diamter = 2.8;
            min_coil_num = 3;
            energy = 0.5;
            speed = 4;
            distance = 0;
            arrowScale = 0;
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

            xArrowID = Guid.Empty;
            yArrowID = Guid.Empty;
            zArrowID = Guid.Empty;
            selectedAxis = -1; // 1 - x axis, 2 - y axis, 3 - z axis 
            guide1 = Guid.Empty;
            guide2 = Guid.Empty;
            brepCut = new List<Brep>();
            convertedPortion = Guid.Empty;
            reserveBrepID1 = Guid.Empty;
            reserveBrepID2 = Guid.Empty;
            motionCtrlPtID1 = Guid.Empty;
            motionCtrlPtID2 = Guid.Empty;
            motionCtrlPointSelected = new Point3d();

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
            if (!DA.GetData(9, ref bake_input))
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

            if (speedLevel == speed_input && distanceLevel== dis_input)
            {
                toAdjustParam = false;
            }
            else
            {
                speedLevel = speed_input;
                distanceLevel = dis_input;
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
                    #region Step 1: select the axis (X, Y, or Z axis)

                    //v = Vector3d.XAxis;
                    Rhino.Input.Custom.GetPoint gp1 = new Rhino.Input.Custom.GetPoint();
                    gp1.SetCommandPrompt("Select one translation axis. Press enter to confirm and continue.");
                    gp1.MouseDown += Gp1_MouseDown;
                    gp1.MouseMove += Gp1_MouseMove;
                    gp1.AcceptNothing(true);
                    Rhino.Input.GetResult r1;

                    OperatingArrow = true;
                    do
                    {
                        if (!ArrowGenerated)
                            GenerateArrow();
                        r1 = gp1.Get(true);

                    } while (r1 != Rhino.Input.GetResult.Nothing);
                    OperatingArrow = false;
                    myDoc.Objects.Hide(xArrowID, true);
                    myDoc.Objects.Hide(yArrowID, true);
                    myDoc.Objects.Hide(zArrowID, true);
                    ArrowGenerated = false;

                    #endregion

                    #region Step 2: drag the two planes to decide the portion

                    Rhino.Input.Custom.GetPoint gp2 = new Rhino.Input.Custom.GetPoint();
                    gp2.SetCommandPrompt("Click and drag the partition plane to adjust their position. Press enter to confirm and continue.");
                    gp2.MouseDown += Gp_SelectionMouseDown;
                    gp2.MouseMove += Gp_SelectionMouseMove;

                    //gp.DynamicDraw += Gp_SelectionDynamicDraw;
                    //gp.AcceptEnterWhenDone(true);
                    gp2.AcceptNothing(true);
                    Rhino.Input.GetResult r2;
                    do
                    {
                        if (!PlaneGenerated)
                            GeneratePlanes();
                        r2 = gp2.Get(true);

                    } while (r2 != Rhino.Input.GetResult.Nothing);
                    //RhinoDoc.ActiveDoc.Objects.Delete(ArrowCurve, true);
                    myDoc.Objects.Delete(guide1, true);
                    myDoc.Objects.Delete(guide2, true);
                    myDoc.Views.Redraw();
                    PlaneGenerated = false;
                    PlaneSelected = true;

                    #endregion

                    #region Step 3: calculate and generate the inner cavity

                    brepCut.Clear();
                    if (PlaneSelected)
                    {
                        // Call out the waiting window
                        processingwin.Show();

                        Plane p1Reverse, p2Reverse;
                        if (t1 >= t2)
                        {
                            p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), v);
                            p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), -v);

                            pl1 = new Plane(skeleton.PointAtNormalizedLength(t1), -v);
                            pl2 = new Plane(skeleton.PointAtNormalizedLength(t2), v);
                        }
                        else
                        {
                            p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), v);
                            p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), -v);

                            pl1 = new Plane(skeleton.PointAtNormalizedLength(t2), -v);
                            pl2 = new Plane(skeleton.PointAtNormalizedLength(t1), v);
                        }

                        Brep[] Cut_Brep1 = model.Trim(pl1, myDoc.ModelAbsoluteTolerance);
                        Brep Brep1 = Cut_Brep1[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
                        brepCut.Add(Brep1);

                        Brep[] Cut_Brep1rest = model.Trim(p1Reverse, myDoc.ModelAbsoluteTolerance);
                        Brep BrepRest = null;
                        try
                        {
                            BrepRest = Cut_Brep1rest[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
                        }
                        catch
                        {
                            BrepRest = model;
                        }
                        Brep[] Cut_Brep2 = BrepRest.Trim(pl2, myDoc.ModelAbsoluteTolerance);
                        Brep Brep2 = null;
                        try
                        {
                            Brep2 = Cut_Brep2[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
                        }
                        catch
                        {
                            Brep2 = BrepRest;
                        }

                        Brep[] Cut_Brep3 = BrepRest.Trim(p2Reverse, myDoc.ModelAbsoluteTolerance);
                        Brep Brep3 = Cut_Brep3[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
                        brepCut.Add(Brep3);
                        brepCut.Add(Brep2);

                        //Rhino.Input.Custom.GetPoint ctrl_first_pt_sel = new Rhino.Input.Custom.GetPoint();

                        BoxLike b = new BoxLike(Brep3, v);
                        double volumn = 0;
                        Brep result1 = null;
                        Cylinder result2 = Cylinder.Unset;
                        Brep b2 = null;
                        double v_box = 0.0, v_cylinder = 0.0;

                        // Calculate the volume of the inner box
                        for (double i = 0.2; i <= 0.8; i += 0.1)
                        {
                            if (b.GetInnerEmptySpaceBox(i))
                            {
                                BoundingBox bbox = b.InnerEmptySpaceBbox;
                                if (volumn < bbox.Volume)
                                {
                                    volumn = bbox.Volume;
                                    result1 = b.InnerEmptySpaceBoxBrep;
                                    result1.Transform(b.RotateBack);
                                    v_box = result1.GetVolume();
                                }
                            }
                        }

                        // Calculate the volume of the inner cylinder 
                        if (b.GetInnerEmptySpaceCylinder())
                        {
                            Cylinder c = b.InnerEmptyCylinder;
                            //result2 = c.ToBrep(true,true);
                            result2 = c;
                            b2 = result2.ToBrep(true, true);
                            b2.Transform(b.RotateBack);
                            v_cylinder = b2.GetVolume();
                            //DA.SetData(2, b2);
                        }

                        if (v_box >= v_cylinder)
                            innerCavity = result1;
                        else
                            innerCavity = b2;
                        conBrep = Brep3;
                        direction = v;

                        processingwin.Hide();

                        myDoc.Objects.Hide(selObjId, true);

                        reserveBrepID1 = myDoc.Objects.AddBrep(Brep1);
                        reserveBrepID2 = myDoc.Objects.AddBrep(Brep2);

                        convertedPortion = myDoc.Objects.AddBrep(Brep3);
                        myDoc.Objects.Select(convertedPortion);

                        Transform cavityTranslation = Transform.Translation(Brep3.GetBoundingBox(true).Center - innerCavity.GetBoundingBox(true).Center);
                        innerCavity.Transform(cavityTranslation);
                    }

                    #endregion

                    #region Step 4: ask the user to select which side to add the motion control

                    Rhino.Input.Custom.GetPoint gp3 = new Rhino.Input.Custom.GetPoint();
                    gp3.SetCommandPrompt("Select one side to add the motion control.");
                    gp3.MouseDown += Gp3_MouseDown;
                    gp3.MouseMove += Gp3_MouseMove;
                    gp3.AcceptNothing(true);
                    Rhino.Input.GetResult r3;

                    GenerateMotionControlIndicator();
                    do
                    {
                        r3 = gp3.Get(true);
                    } while (r3 != Rhino.Input.GetResult.Nothing);

                    myDoc.Objects.Hide(motionCtrlPtID1, true);
                    myDoc.Objects.Hide(motionCtrlPtID2, true);

                    #endregion

                    #region Step 5: create an instance of Continuous Translation class

                    motion = new ContinuousTranslation(model, direction, innerCavity, motionCtrlPointSelected, speedLevel, distanceLevel, motionControlMethod);

                    //motion.Set3Parts(t1, t2, brepCut[0], brepCut[1], brepCut[2]);

                    //toBeBaked.Add(reserveBrepID1);
                    //toBeBaked.Add(reserveBrepID2);
                    //myDoc.Views.Redraw();
                    #endregion
                }
            }

            if (toSetEEPos)
            {

            }

            if(toSetAxisDir)
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

                #region gear test

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

                //Gear temp = new Gear(new Point3d(0, 0, 0), new Vector3d(0, 0, 1), numTeeth, 1, 20, 3.6, selfRotationAngle, true);

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
                //Gear temp1 = new Gear(new Point3d(14.8, 0, 0), new Vector3d(0, 0, 1), numTeeth, 1, 20, 3.6, selfRotationAngle, true);

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
                //Gear temp2 = new Gear(new Point3d(14.8 + 0.4 + 32.5, 0, 0), new Vector3d(0, 0, 1), numTeeth, 1, 20, 3.6, selfRotationAngle, true);


                //// the tolerance between two mating gears is 0.4

                //myDoc.Objects.AddBrep(temp.Model);
                //myDoc.Views.Redraw();
                //myDoc.Objects.AddBrep(temp1.Model);
                //myDoc.Views.Redraw();
                //myDoc.Objects.AddBrep(temp2.Model);
                //myDoc.Views.Redraw();

                #endregion

                #region rack test
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

                //foreach (Brep b in tempQR.QuickReturnModels)
                //{
                //    myDoc.Objects.AddBrep(b);
                //    myDoc.Views.Redraw();
                //}

                #endregion

                #region crank and slotted lever

                CrankSlottedLever tempCSL = new CrankSlottedLever(new Point3d(0, 0, 0), new Vector3d(0, 0, 1), new Vector3d(1, 0, 0), 15, 35);

                foreach (Brep b in tempCSL.CrankSlottedLeverModels)
                {
                    myDoc.Objects.AddBrep(b);
                    myDoc.Views.Redraw();
                }

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

        void GenerateMotionControlIndicator()
        {
            Point3d lPt = new Point3d();
            Point3d rPt = new Point3d();

            Point3d skePt1 = skeleton.PointAt(0);
            Point3d skePt2 = skeleton.PointAt(1);

            lPt = skePt1;
            rPt = skePt2;

            motionCtrlPtID1 = myDoc.Objects.AddSphere(new Sphere(lPt, 10), blueAttribute);
            motionCtrlPtID2 = myDoc.Objects.AddSphere(new Sphere(rPt, 10), blueAttribute);

            myDoc.Views.Redraw();

        }
        private void Gp3_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep lPtBrep = (Brep)myDoc.Objects.Find(motionCtrlPtID1).Geometry;
            Brep rPtBrep = (Brep)myDoc.Objects.Find(motionCtrlPtID2).Geometry;

            double l_dis = lPtBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double r_dis = rPtBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if(l_dis <= 30 || r_dis <= 30)
            {
                if (l_dis <= r_dis)
                {
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(motionCtrlPtID1);
                }
                else
                {
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(motionCtrlPtID2);
                }
            }
            else
            {
                myDoc.Objects.UnselectAll();
            }
        }

        private void Gp3_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep lPtBrep = (Brep)myDoc.Objects.Find(motionCtrlPtID1).Geometry;
            Brep rPtBrep = (Brep)myDoc.Objects.Find(motionCtrlPtID2).Geometry;

            double l_dis = lPtBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double r_dis = rPtBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if (l_dis <= 30 || r_dis <= 30)
            {
                if (l_dis <= r_dis)
                {
                    motionCtrlPointSelected = ((Brep)myDoc.Objects.Find(motionCtrlPtID1).Geometry).GetBoundingBox(true).Center;
                }
                else
                {
                    motionCtrlPointSelected = ((Brep)myDoc.Objects.Find(motionCtrlPtID2).Geometry).GetBoundingBox(true).Center;
                }
            }
        }

        private void GenerateArrow()
        {
            ArrowGenerated = true;
            if (xArrowID != Guid.Empty)
            {
                myDoc.Objects.Delete(xArrowID, true);
                xArrowID = Guid.Empty;
            }

            if (yArrowID != Guid.Empty)
            {
                myDoc.Objects.Delete(yArrowID, true);
                yArrowID = Guid.Empty;
            }

            if (zArrowID != Guid.Empty)
            {
                myDoc.Objects.Delete(zArrowID, true);
                zArrowID = Guid.Empty;
            }

            double axisRadius = 1;
            Point3d XEndPt = center + Vector3d.XAxis * 30;
            Point3d YEndPt = center + Vector3d.YAxis * 30;
            Point3d ZEndPt = center + Vector3d.ZAxis * 30;

            Line xLn = new Line(center, XEndPt);
            Curve xCrv = xLn.ToNurbsCurve();
            Brep xAxisBrep = Brep.CreatePipe(xCrv, axisRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Plane xArrowPln = new Plane(XEndPt + Vector3d.XAxis * 5, (-1) * Vector3d.XAxis);
            Cone xAxisArrowTipCone = new Cone(xArrowPln, 5, 2 * axisRadius);
            Brep xAxisArrowTipBrep = xAxisArrowTipCone.ToBrep(true);
            Brep xAxisArrow = Brep.CreateBooleanUnion(new List<Brep> { xAxisBrep, xAxisArrowTipBrep }, myDoc.ModelAbsoluteTolerance)[0];

            Line yLn = new Line(center, YEndPt);
            Curve yCrv = yLn.ToNurbsCurve();
            Brep yAxisBrep = Brep.CreatePipe(yCrv, axisRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Plane yArrowPln = new Plane(YEndPt + Vector3d.YAxis * 5, (-1) * Vector3d.YAxis);
            Cone yAxisArrowTipCone = new Cone(yArrowPln, 5, 2 * axisRadius);
            Brep yAxisArrowTipBrep = yAxisArrowTipCone.ToBrep(true);
            Brep yAxisArrow = Brep.CreateBooleanUnion(new List<Brep> { yAxisBrep, yAxisArrowTipBrep }, myDoc.ModelAbsoluteTolerance)[0];

            Line zLn = new Line(center, ZEndPt);
            Curve zCrv = zLn.ToNurbsCurve();
            Brep zAxisBrep = Brep.CreatePipe(zCrv, axisRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
            Plane zArrowPln = new Plane(ZEndPt + Vector3d.ZAxis * 5, (-1) * Vector3d.ZAxis);
            Cone zAxisArrowTipCone = new Cone(zArrowPln, 5, 2 * axisRadius);
            Brep zAxisArrowTipBrep = zAxisArrowTipCone.ToBrep(true);
            Brep zAxisArrow = Brep.CreateBooleanUnion(new List<Brep> { zAxisBrep, zAxisArrowTipBrep }, myDoc.ModelAbsoluteTolerance)[0];

            xArrowID = myDoc.Objects.AddBrep(xAxisArrow, redAttribute);
            yArrowID = myDoc.Objects.AddBrep(yAxisArrow, greenAttribute);
            zArrowID = myDoc.Objects.AddBrep(zAxisArrow, blueAttribute);
            myDoc.Views.Redraw();
        }

        private void GeneratePlanes()
        {
            PlaneGenerated = true;
            //Delete these before generating new ones
            myDoc.Objects.Delete(guide1, true);
            myDoc.Objects.Delete(guide2, true);

            switch (selectedAxis)
            {
                case 1: v = Vector3d.XAxis; break;
                case 2: v = Vector3d.YAxis; break;
                case 3: v = Vector3d.ZAxis; break;
                default: break;
            }
            if (v != null)
            {
                BoxLike b = new BoxLike(model, v);
                BoundingBox box = b.Bbox;
                box.Transform(b.RotateBack);
                /*Point3d start = box.PointAt(0, 0.5, 0.5);
                Point3d end = box.PointAt(1, 0.5, 0.5);*/ //this doesn't work!

                skeleton = b.Skeleton;
                skeleton.Transform(b.RotateBack);
                skeletonVec = new Vector3d(skeleton.PointAtEnd) - new Vector3d(skeleton.PointAtStart);
                pl1 = new Plane(skeleton.PointAtNormalizedLength(t1), v);
                pl2 = new Plane(skeleton.PointAtNormalizedLength(t2), v);

                Interval plnXInterval;
                Interval plnYInterval;
                Interval plnZInterval;
                if (selectedAxis == 1)
                {
                    // x axis is selected
                    plnYInterval = new Interval(-(box.Max.Y - box.Min.Y) * 0.6, (box.Max.Y - box.Min.Y) * 0.6);
                    plnZInterval = new Interval(-(box.Max.Z - box.Min.Z) * 0.6, (box.Max.Z - box.Min.Z) * 0.6);
                    s1 = new PlaneSurface(pl1, plnYInterval, plnZInterval);
                    s2 = new PlaneSurface(pl2, plnYInterval, plnZInterval);
                }
                else if (selectedAxis == 2)
                {
                    // y axis is selected
                    plnXInterval = new Interval(-(box.Max.X - box.Min.X) * 0.6, (box.Max.X - box.Min.X) * 0.6);
                    plnZInterval = new Interval(-(box.Max.Z - box.Min.Z) * 0.6, (box.Max.Z - box.Min.Z) * 0.6);
                    s1 = new PlaneSurface(pl1, plnXInterval, plnZInterval);
                    s2 = new PlaneSurface(pl2, plnXInterval, plnZInterval);
                }
                else if (selectedAxis == 3)
                {
                    // z axis is selected
                    plnXInterval = new Interval(-(box.Max.X - box.Min.X) * 0.6, (box.Max.X - box.Min.X) * 0.6);
                    plnYInterval = new Interval(-(box.Max.Y - box.Min.Y) * 0.6, (box.Max.Y - box.Min.Y) * 0.6);
                    s1 = new PlaneSurface(pl1, plnXInterval, plnYInterval);
                    s2 = new PlaneSurface(pl2, plnXInterval, plnYInterval);
                }

                guide1 = myDoc.Objects.Add(s1);
                guide2 = myDoc.Objects.Add(s2);
                myDoc.Views.Redraw();
            }
        }

        private void Gp1_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep xBrep = (Brep)myDoc.Objects.Find(xArrowID).Geometry;
            Brep yBrep = (Brep)myDoc.Objects.Find(yArrowID).Geometry;
            Brep zBrep = (Brep)myDoc.Objects.Find(zArrowID).Geometry;

            double x_dis = xBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double y_dis = yBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double z_dis = zBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if (x_dis <= y_dis && x_dis <= z_dis)
            {
                myDoc.Objects.UnselectAll();
                myDoc.Objects.Select(xArrowID);
            }
            else if (y_dis <= x_dis && y_dis <= z_dis)
            {
                myDoc.Objects.UnselectAll();
                myDoc.Objects.Select(yArrowID);
            }
            else if (z_dis <= y_dis && z_dis <= x_dis)
            {
                myDoc.Objects.UnselectAll();
                myDoc.Objects.Select(zArrowID);
            }
            else
            {
                myDoc.Objects.UnselectAll();
            }
        }

        private void Gp1_MouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            Point3d currPos = e.Point;
            Brep xBrep = (Brep)myDoc.Objects.Find(xArrowID).Geometry;
            Brep yBrep = (Brep)myDoc.Objects.Find(yArrowID).Geometry;
            Brep zBrep = (Brep)myDoc.Objects.Find(zArrowID).Geometry;

            double x_dis = xBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double y_dis = yBrep.ClosestPoint(currPos).DistanceTo(currPos);
            double z_dis = zBrep.ClosestPoint(currPos).DistanceTo(currPos);

            if (x_dis <= y_dis && x_dis <= z_dis)
            {
                selectedAxis = 1;
            }
            else if (y_dis <= x_dis && y_dis <= z_dis)
            {
                selectedAxis = 2;
            }
            else if (z_dis <= y_dis && z_dis <= x_dis)
            {
                selectedAxis = 3;
            }
            else
            {
                selectedAxis = -1;
            }
        }

        private void Gp_SelectionMouseDown(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (selected != Guid.Empty)
                selected = Guid.Empty;
            else
            {
                var p = e.Point;
                double dis1 = Math.Abs(pl1.DistanceTo(p)), dis2 = Math.Abs(pl2.DistanceTo(p));

                List<double> distances = new List<double> { dis1, dis2 };
                double min = distances.Min();
                if (min > 5) return;
                else if (min == dis1)
                {
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(guide1);
                    selected = guide1;
                }
                else if (min == dis2)
                {
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(guide2);
                    selected = guide2;
                }
            }

        }

        private void Gp_SelectionMouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (selected == Guid.Empty)
                return;

            double t = 0, tn = 0;
            skeleton.ClosestPoint(e.Point, out t);
            skeleton.NormalizedLengthParameter(t, out tn);
            if (tn < 0)
                tn = 0;
            if (tn > 1)
                tn = 1;
            if (selected == guide1)
            {
                //calculate where is the mouse and change t
                if (Math.Abs(t1 - tn) > 0.01)
                {
                    //move and update t1
                    Transform m = Transform.Translation(skeletonVec * (tn - t1));
                    guide1 = myDoc.Objects.Transform(guide1, m, true);
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(guide1);
                    pl1.Transform(m);
                    t1 = tn;
                    myDoc.Views.Redraw();
                }

            }
            if (selected == guide2)
            {
                //calculate where is the mouse and change t
                if (Math.Abs(t2 - tn) > 0.01)
                {
                    //move and update t2
                    Transform m = Transform.Translation(skeletonVec * (tn - t2));
                    guide2 = myDoc.Objects.Transform(guide2, m, true);
                    myDoc.Objects.UnselectAll();
                    myDoc.Objects.Select(guide2);
                    pl2.Transform(m);
                    t2 = tn;
                    myDoc.Views.Redraw();
                }
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