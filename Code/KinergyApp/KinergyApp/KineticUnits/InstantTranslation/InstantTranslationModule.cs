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

namespace InstTranslation
{
    public class InstantTranslationModule : GH_Component
    {
        // Variables that store information
        Brep model;
        Brep conBrep;
        Brep innerCavity;
        double t1, t2; // the positoins of start point and end point of the segment on the normalized skeleton
        Curve skeleton;     // skeleton
        double energyLevel;         // value of the strength slide bar
        double displacementLevel;   // value of the displacement slide bar
        Vector3d direction;             // motion direction
        InstantTranslation motion;
        List<Guid> lockPosPointIDs;
        Guid endEffectorID;
        double innerSpaceRadius;
        

        // Variables used for different functions
        bool lockState;
        double min_wire_diamter;
        double min_coil_num;
        double energy;
        double displacement;
        int lockPos; // 1: inside (by default); 2: outside
        bool isLockSet;
        Guid selObjId;
        List<Guid> toBeBaked;
        List<Guid> endEffectorCandidates;

        // Region selection related variables
        Point3d center = Point3d.Unset;
        Guid guide1, guide2;
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

        ObjectAttributes solidAttribute, orangeAttribute, redAttribute, blueAttribute, greenAttribute;
        RhinoDoc myDoc;
        Guid xArrowID, yArrowID, zArrowID;
        int selectedAxis;
        List<Point3d> lockPosCandidates;
        Point3d lockPostion;

        bool testBodySelBtn;
        bool testAxisSelBtn;
        bool testEndEffectorBtn;
        bool testPreBtn;
        bool testBakeBtn;

        /// <summary>
        /// Initializes a new instance of the InstantTranslationModule class.
        /// </summary>
        public InstantTranslationModule()
          : base("InstantTranslationModule", "ITMoudle",
              "The instant translation kinetic unit",
              "Kinergy", "KineticUnits")
        {
            model = null;
            conBrep = new Brep();
            innerCavity = new Brep();
            t1 = 0;
            t2 = 1;
            skeleton = null;
            energyLevel = 4;
            displacementLevel = 4;
            direction = new Vector3d();
            motion = null;
            endEffectorID = Guid.Empty;
            innerSpaceRadius = 0;

            lockState = false;
            min_wire_diamter = 2.8;
            min_coil_num = 3;
            energy = 0.5;
            displacement = 0.5;
            arrowScale = 0;
            lockPos = 1;
            isLockSet = false;
            selObjId = Guid.Empty;
            toBeBaked = new List<Guid>();
            endEffectorCandidates = new List<Guid>();
            lockPosPointIDs = new List<Guid>();

            myDoc = RhinoDoc.ActiveDoc;
            xArrowID = Guid.Empty;
            yArrowID = Guid.Empty;
            zArrowID = Guid.Empty;
            selectedAxis = -1; // 1 - x axis, 2 - y axis, 3 - z axis 
            lockPosCandidates = new List<Point3d>();
            lockPostion = new Point3d();
            guide1 = Guid.Empty;
            guide2 = Guid.Empty;

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

            testBodySelBtn = false;
            testAxisSelBtn = false;
            testEndEffectorBtn = false;
            testPreBtn = false;
            testBakeBtn = false;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // User triggers for actions
            pManager.AddBooleanParameter("RegionSelection", "Reg", "Enabling region selection", GH_ParamAccess.item);
            pManager.AddBooleanParameter("AxisSelection", "Axis", "Enabling the selection of the translating axis", GH_ParamAccess.item);
            pManager.AddBooleanParameter("EndeffectorSetting", "EE", "Enabling the selection of the end-effector", GH_ParamAccess.item);
            pManager.AddBooleanParameter("AddLock", "L", "Enabling locking", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Preview", "Pre", "Enabling preview", GH_ParamAccess.item);

            // Value listeners 
            pManager.AddNumberParameter("CompressionDisplacement", "Disp", "Proportion of spring that's able to be compressed", GH_ParamAccess.item);
            pManager.AddNumberParameter("Energy", "E", "Energy of motion", GH_ParamAccess.item);

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
            bool reg_input = false, axis_input = false, end_input = false, addlock_input = false, pre_input = false, bake_input = false;
            double disp_input = 4, energy_input = 4;

            #region input param readings
            if (!DA.GetData(0, ref reg_input))
                return;
            if (!DA.GetData(1, ref axis_input))
                return;
            if (!DA.GetData(2, ref end_input))
                return;
            if (!DA.GetData(3, ref addlock_input))
                return;
            if (!DA.GetData(4, ref pre_input))
                return;
            if (!DA.GetData(5, ref disp_input))
                return;
            if (!DA.GetData(6, ref energy_input))
                return;
            if (!DA.GetData(7, ref bake_input))
                return;
            #endregion

            // variables to control states
            bool toSelectRegion = false, toSetAxis = false, toAdjustParam = false, toSetEndEffector = false, toAddLock = false, toRemoveLock = false, toPreview = false, toBake = false;

            #region Input check. This determines how the cell respond to changed params

            if (!reg_input && testBodySelBtn)//This applies to starting situation and when u change the input model
            {
                toSelectRegion = true;
                testBodySelBtn = false;
            }
            else if (reg_input)
            {
                testBodySelBtn = true;
            }

            if (!axis_input && testAxisSelBtn)
            {
                toSetAxis = true;
                testAxisSelBtn = false;
            }
            else if (axis_input)
            {
                testAxisSelBtn = true;
            }

            if (!end_input && testEndEffectorBtn)
            {
                toSetEndEffector = true;
                testEndEffectorBtn = false;
            }
            else if (end_input)
            {
                testEndEffectorBtn = true;
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
            //if (pre_input)
            //    toPreview = true;

            if (energyLevel == energy_input && displacementLevel == disp_input)
            {
                toAdjustParam = false;
            }
            else
            {
                energyLevel = energy_input;
                displacementLevel = disp_input;
                toAdjustParam = true;
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

            #endregion

            if (toBake)
            {
                if(motion != null)
                {
                    //Guid springID = myDoc.Objects.AddBrep(motion.Spring.GetModelinWorldCoordinate(), orangeAttribute);
                    //toBeBaked.Add(springID);

                    //myDoc.Objects.Hide(springID, true);

                    //foreach (Guid id in toBeBaked)
                    //{
                    //    myDoc.Objects.Show(id, true);
                    //}

                    foreach(Guid id in endEffectorCandidates)
                    {
                        myDoc.Objects.Hide(id, true);
                    }


                    if (motion.EntityList != null)
                    {
                        foreach (Entity b in motion.EntityList)
                        {
                            Brep tempB = b.GetModelinWorldCoordinate();
                            myDoc.Objects.AddBrep(tempB);
                        }
                        if(motion.Spring.SpringDimensions!=null)
                        {
                            foreach(LinearDimension d in motion.Spring.SpringDimensions)
                            {
                                if(myDoc.Objects.AddLinearDimension(d)!=Guid.Empty)
                                {
                                    RhinoApp.WriteLine("Added a dimension!");
                                }
                                else
                                {
                                    RhinoApp.WriteLine("fail to add dimension!");
                                }
                            }
                        }
                        myDoc.Views.Redraw();
                        this.ExpirePreview(true);
                    }
                }
            }

            if (toSelectRegion)
            {
                if(selObjId != Guid.Empty)
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

            if (toSetAxis)
            {
                if(model != null){

                    #region Step 1: select the axis (X, Y, or Z axis)

                    //RhinoApp.KeyboardEvent += RhinoApp_KeyboardEvent;

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
                        //TODO Add notifications here if the planes are too close to ends and there won't be enough space for a lock!
                        double dis1 = 0, dis2 = 0;
                        if(t1<=t2)
                        {
                            dis1 = t1;dis2 = 1 - t2;
                        }
                        else
                        {
                            dis1 = t2; dis2 = 1 - t1;
                        }
                        double minDis = Math.Min(dis1, dis2)*skeleton.GetLength();
                        //if(minDis<)

                    } while (r2 != Rhino.Input.GetResult.Nothing);
                    //RhinoDoc.ActiveDoc.Objects.Delete(ArrowCurve, true);
                    myDoc.Objects.Delete(guide1, true);
                    myDoc.Objects.Delete(guide2, true);
                    myDoc.Views.Redraw();
                    PlaneGenerated = false;
                    PlaneSelected = true;

                    #endregion

                    #region Step 3: calculate and generate the inner cavity

                    if (PlaneSelected)
                    {
                        // Call out the waiting window
                        processingwin.Show();

                        Plane p1Reverse, p2Reverse;
                        if(t1 >= t2)
                        {
                            p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), v);
                            p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), -v);
                        }
                        else
                        {
                            p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), v);
                            p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), -v);
                        }
                        
                        Brep[] Cut_Brep1 = model.Trim(p1Reverse, myDoc.ModelAbsoluteTolerance);
                        Brep BrepRest = Cut_Brep1[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
                        
                        Brep[] Cut_Brep2 = BrepRest.Trim(p2Reverse, myDoc.ModelAbsoluteTolerance);
                        Brep BrepPortion = Cut_Brep2[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                        BoxLike b = new BoxLike(BrepPortion, v);
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
                        conBrep = BrepPortion;
                        direction = v;

                        processingwin.Hide();

                        Transform cavityTranslation = Transform.Translation(BrepPortion.GetBoundingBox(true).Center - innerCavity.GetBoundingBox(true).Center);
                        innerCavity.Transform(cavityTranslation);
                    }

                    #endregion

                    #region Step 4: create an instance of InstantTranslation class

                    #region Parse energy and displacement

                    // Parse the dispalcement (in percentage) based on the spring length and the posible max compression dispacement
                    Point3d ptS = new Point3d();
                    Point3d ptE = new Point3d();

                    if (t1 >= t2)
                    {
                        ptS = skeleton.PointAtNormalizedLength(t2);
                        ptE = skeleton.PointAtNormalizedLength(t1);
                    }
                    else
                    {
                        ptS = skeleton.PointAtNormalizedLength(t1);
                        ptE = skeleton.PointAtNormalizedLength(t2);
                    }
                    
                    double s_len = ptS.DistanceTo(ptE);
                    double maxDisp = Math.Max(s_len - min_wire_diamter * min_coil_num, min_coil_num * 0.6);
                    displacement = (displacementLevel + 1) / 10 * maxDisp / s_len;     // convert the input displacement level into percentage

                    // Parse the energy based on E ~= d^4/n * x^2
                    double x = displacement * s_len;
                    energy = (energyLevel + 1) / 10;

                    #endregion

                    motion = new InstantTranslation(model, false, direction, energy, displacement);      // the second argument represents if the skeleton is curved

                    #endregion

                    #region Step 5: find the position of the helical spring

                    motion.CalculateStraightSkeleton(ptS, ptE, model);
                    Point3d springPos = motion.Skeleton.PointAtNormalizedLength((t1 + t2) / 2);

                    //myDoc.Objects.AddPoint(springPos);
                    //myDoc.Views.Redraw();

                    #endregion

                    #region Step 6: Construct the spring based on the input energy and distance

                    motion.SetSpringPosition(springPos);
                    motion.CutModelForSpring();
                    motion.ConstructSpring(out innerSpaceRadius);

                    // Generate the arrows but reserved for the end-effector step to confirm the lock position
                    
                    foreach (Shape s in motion.ModelCut)
                    {
                        Brep tempB = s.GetModelinWorldCoordinate();
                        Guid bID = myDoc.Objects.AddBrep(tempB);
                        toBeBaked.Add(bID);
                        endEffectorCandidates.Add(bID);
                    }
                    //Guid springID = myDoc.Objects.AddBrep(motion.Spring.GetModelinWorldCoordinate(), orangeAttribute);
                    //toBeBaked.Add(springID);

                    //myDoc.Objects.Hide(springID, true);
                    myDoc.Objects.Hide(selObjId, true);
                    myDoc.Views.Redraw();
                    #endregion
                }
            }

            if (toSetEndEffector)
            {
                if(endEffectorCandidates.Count != 0)
                {
                    // Ask the user to select a Brep
                    ObjRef objSel_ref;
                    Guid selObjId = Guid.Empty;
                    var rc = RhinoGet.GetOneObject("Select a surface or polysurface as the end-effector", false, ObjectType.AnyObject, out objSel_ref);
                    if (rc == Rhino.Commands.Result.Success)
                    {
                        // select a brep
                        selObjId = objSel_ref.ObjectId;
                        endEffectorID = selObjId;

                        #region Parse energy and displacement

                        // Parse the dispalcement (in percentage) based on the spring length and the posible max compression dispacement
                        Point3d ptS = new Point3d();
                        Point3d ptE = new Point3d();

                        Brep eeBrep = (Brep)myDoc.Objects.FindId(endEffectorID).Geometry;
                        double stationarySegLen = 0;
                        double stationaryConstraintLen = 16;

                        Point3d skeStartPt = skeleton.PointAtNormalizedLength(0);
                        Point3d skeEndPt = skeleton.PointAtNormalizedLength(1);

                        if (t1 >= t2)
                        {
                            ptS = skeleton.PointAtNormalizedLength(t2);
                            ptE = skeleton.PointAtNormalizedLength(t1);

                            if (eeBrep.ClosestPoint(ptS).DistanceTo(ptS) <= eeBrep.ClosestPoint(ptE).DistanceTo(ptE))
                            {
                                // ptE is closer to the stationary segment
                                stationarySegLen = skeleton.GetLength() * (1 - t1);

                                if (stationarySegLen < stationaryConstraintLen)
                                {
                                    ptE= skeleton.PointAtLength(skeleton.GetLength() - stationaryConstraintLen);
                                    t1 = ptE.DistanceTo(skeStartPt) / skeStartPt.DistanceTo(skeEndPt);
                                }
                            }
                            else
                            {
                                // ptS is closer to the stationary segment
                                stationarySegLen = skeleton.GetLength() * t2;

                                if (stationarySegLen < stationaryConstraintLen)
                                {
                                    ptS = skeleton.PointAtLength(stationaryConstraintLen);
                                    t2 = ptS.DistanceTo(skeStartPt) / skeStartPt.DistanceTo(skeEndPt);
                                }
                            }
                        }
                        else
                        {
                            ptS = skeleton.PointAtNormalizedLength(t1);
                            ptE = skeleton.PointAtNormalizedLength(t2);

                            if (eeBrep.ClosestPoint(ptS).DistanceTo(ptS) <= eeBrep.ClosestPoint(ptE).DistanceTo(ptE))
                            {
                                // ptE is closer to the stationary segment
                                stationarySegLen = skeleton.GetLength() * (1 - t2);

                                if (stationarySegLen < stationaryConstraintLen)
                                {
                                    ptE = skeleton.PointAtLength(skeleton.GetLength() - stationaryConstraintLen);
                                    t2 = ptE.DistanceTo(skeStartPt) / skeStartPt.DistanceTo(skeEndPt);
                                }
                            }
                            else
                            {
                                // ptS is closer to the stationary segment
                                stationarySegLen = skeleton.GetLength() * t1;

                                if (stationarySegLen < stationaryConstraintLen)
                                {
                                    ptS = skeleton.PointAtLength(stationaryConstraintLen);
                                    t1 = ptS.DistanceTo(skeStartPt) / skeStartPt.DistanceTo(skeEndPt);
                                }
                            }
                        }
                        double s_len = ptS.DistanceTo(ptE);

                        double maxDisp = Math.Max(s_len - min_wire_diamter * min_coil_num, min_coil_num * 0.6);
                        displacement = (displacementLevel + 1) / 10 * maxDisp / s_len;     // convert the input displacement level into percentage

                        // Parse the energy based on E ~= d^4/n * x^2
                        double x = displacement * s_len;
                        energy = (energyLevel + 1) / 10;

                        #endregion

                        motion = new InstantTranslation(model, false, direction, energy, displacement);      // the second argument represents if the skeleton is curved

                        #region update the helical spring

                        motion.CalculateStraightSkeleton(ptS, ptE, model);
                        Point3d springPos = motion.Skeleton.PointAtNormalizedLength((t1 + t2) / 2);

                        motion.SetSpringPosition(springPos);
                        motion.CutModelForSpring();
                        double springDia = 0;
                        motion.ConstructSpring(out springDia);

                        #endregion
                    }
                }
                else
                {
                    RhinoApp.CommandPrompt = "Please select an object first!";
                }
            }

            if (toAddLock)
            {
                
                if( motion != null && endEffectorID != Guid.Empty)
                {
                    #region show the original brep and hide temparily generated parts by GH
                    
                    foreach(Guid id in toBeBaked)
                    {
                        RhinoDoc.ActiveDoc.Objects.Hide(id, true);
                    }
                    RhinoDoc.ActiveDoc.Objects.Show(selObjId, true);
                    RhinoDoc.ActiveDoc.Views.Redraw();

                    #endregion

                    #region Generate the lock position candidates and the user select the postion

                    //motion.SetLockDirection(p);
                    lockPosCandidates.Clear();
                    lockPosCandidates = motion.GetLockPositionCandidates(endEffectorID);

                    foreach(Point3d p in lockPosCandidates)
                    {
                       Guid ptID = myDoc.Objects.AddPoint(p);
                       lockPosPointIDs.Add(ptID);
                    }
                    myDoc.Views.Redraw();

                    Rhino.Input.Custom.GetPoint gp_lock = new Rhino.Input.Custom.GetPoint();
                    gp_lock.SetCommandPrompt("Select a point to confirm the lock button position.");
                    gp_lock.DynamicDraw += Gp_lock_DynamicDraw;
                    gp_lock.MouseMove += Gp_lock_MouseMove;
                    Rhino.Input.GetResult r_lock;
                    r_lock = gp_lock.Get(true);

                    //Point3d pt;
                    //pt = pts[UserSelection.UserSelectPointInRhino(pts, RhinoDoc.ActiveDoc)];
                    
                    #endregion

                    #region Construct the lock based on the set position and type

                    //motion.SetLockPosition(lockPostion);  // is this step redundant? Yes, it is.
                    //motion.CutModelForLock();
                    GH_Document gh_d = this.OnPingDocument();
                    motion.ConstructLock(lockPostion, gh_d, innerSpaceRadius, endEffectorID);  // lockPos=1: inside; lockPos=2: outside

                    myDoc.Objects.Hide(selObjId, true);
                    foreach (Guid id in toBeBaked)
                    {
                        RhinoDoc.ActiveDoc.Objects.Show(id, true);
                    }
                    foreach(Guid id in lockPosPointIDs)
                    {
                        myDoc.Objects.Delete(id, true);
                    }
                    lockPosPointIDs.Clear();
                
                    myDoc.Views.Redraw();
                    #endregion
                }
            }

            if (toRemoveLock)
            {
                if (motion != null && endEffectorID != Guid.Empty)
                {
                    GH_Document gh_d = this.OnPingDocument();
                    motion.DeleteLock(gh_d, endEffectorID);
                }
            }

            if (toAdjustParam)
            {
                #region Parse energy and displacement

                // Parse the dispalcement (in percentage) based on the spring length and the posible max compression dispacement
                Point3d ptS = new Point3d();
                Point3d ptE = new Point3d();

                if (t1 >= t2)
                {
                    ptS = skeleton.PointAtNormalizedLength(t2);
                    ptE = skeleton.PointAtNormalizedLength(t1);
                }
                else
                {
                    ptS = skeleton.PointAtNormalizedLength(t1);
                    ptE = skeleton.PointAtNormalizedLength(t2);
                }
                double s_len = ptS.DistanceTo(ptE);
                double maxDisp = Math.Max(s_len - min_wire_diamter * min_coil_num, min_coil_num * 0.6);
                displacement = (displacementLevel + 1) / 10 * maxDisp / s_len;     // convert the input displacement level into percentage

                // Parse the energy based on E ~= d^4/n * x^2
                double x = displacement * s_len;
                energy = (energyLevel + 1) / 10;

                #endregion

                motion = new InstantTranslation(model, false, direction, energy, displacement);      // the second argument represents if the skeleton is curved

                #region Step 5: find the position of the helical spring

                motion.CalculateStraightSkeleton(ptS, ptE, model);
                Point3d springPos = motion.Skeleton.PointAtNormalizedLength((t1 + t2) / 2);

                #endregion

                #region Step 6: Construct the spring based on the input energy and distance

                motion.SetSpringPosition(springPos);
                motion.CutModelForSpring();
                double springDia = 0;
                motion.ConstructSpring(out springDia);

                // Generate the arrows but reserved for the end-effector step to confirm the lock position
                //lockDirCandidates = motion.GetLockDirectionCandidates();

                #endregion
            }

            DA.SetData(0, motion);
            //DA.SetData(1, model);
            if(motion == null)
                DA.SetDataList(1, null);
            else
                DA.SetDataList(1, motion.GetModel());
            DA.SetData(2, toPreview);
        }

        private void Gp_lock_DynamicDraw(object sender, Rhino.Input.Custom.GetPointDrawEventArgs e)
        {
            e.Display.DrawSphere(new Sphere(lockPostion, 5), Color.FromArgb(16, 150, 206));
        }

        private void Gp_lock_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            foreach(Point3d pt in lockPosCandidates)
            {
                if(e.Point.DistanceTo(pt) <= 3)
                {
                    lockPostion = pt;
                }
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
            RhinoDoc.ActiveDoc.Objects.Delete(guide1, true);
            RhinoDoc.ActiveDoc.Objects.Delete(guide2, true);

            switch (selectedAxis)
            {
                case 1: v = Vector3d.XAxis;break;
                case 2: v = Vector3d.YAxis;break;
                case 3:v = Vector3d.ZAxis;break;
                default: break;
            }
            if(v != null)
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
                else if(selectedAxis == 2)
                {
                    // y axis is selected
                    plnXInterval = new Interval(-(box.Max.X - box.Min.X) * 0.6, (box.Max.X - box.Min.X) * 0.6);
                    plnZInterval = new Interval(-(box.Max.Z - box.Min.Z) * 0.6, (box.Max.Z - box.Min.Z) * 0.6);
                    s1 = new PlaneSurface(pl1, plnXInterval, plnZInterval);
                    s2 = new PlaneSurface(pl2, plnXInterval, plnZInterval);
                }
                else if(selectedAxis == 3)
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
            get { return new Guid("9ebc8af4-9d77-4a75-aabe-a150404e81e6"); }
        }
    }
}