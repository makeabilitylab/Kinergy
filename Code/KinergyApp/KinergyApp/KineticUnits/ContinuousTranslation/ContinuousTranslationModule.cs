using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy.KineticUnit;
using Kinergy.Utilities;
using Rhino;
using Rhino.Geometry;
using Rhino.DocObjects;
using Rhino.Input;
using HumanUIforKinergy.KinergyUtilities;
using Kinergy.Geom;
using System.Linq;

namespace Kinergy.KineticUnit
{
    public class ContinuousTranslationModule : GH_Component
    {
        // Variables that store information
        Brep model;             // the original brep model
        Brep conBrep;           // the Brep that is selected and converted
        Brep innerCavity;
        double t1, t2; // the positoins of start point and end point of the segment on the normalized skeleton
        Curve skeleton;     // skeleton
        double energyLevel;         // value of the strength slide bar
        double speedLevel;
        int distance;   // value of the distance slide bar
        Vector3d direction;             // kinetic unit direction
        ContinuousTranslation motion;
        List<Arrow> lockDirCandidates;
        Vector3d orientationDir;
        Arrow p;
        RhinoDoc myDoc;
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

        /// <summary>
        /// Initializes a new instance of the ContinuousTranslationModule class.
        /// </summary>
        public ContinuousTranslationModule()
          : base("ContinuousTranslationModule", "CTModule",
              "The kinetic unit for continuous translation",
              "Kinergy", "ContinuousTranslation")
        {
            model = null;
            conBrep = new Brep();
            innerCavity = new Brep();
            t1 = 0;
            t2 = 1;
            skeleton = null;
            energyLevel = 0.5;
            speedLevel = 4;
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
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            // User triggers for actions
            pManager.AddBooleanParameter("RegionSelection", "Reg", "Enabling region selection and direction calculation", GH_ParamAccess.item);
            pManager.AddBooleanParameter("EndeffectorSetting", "EE", "Enabling the selection of the end-effector", GH_ParamAccess.item);
            pManager.AddBooleanParameter("AddLock", "L", "Enabling locking", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Preview", "Pre", "Enabling preview", GH_ParamAccess.item);

            // Value listeners 
            pManager.AddNumberParameter("Energy", "E", "Energy of motion", GH_ParamAccess.item);
            pManager.AddNumberParameter("Speed", "S", "Speed of motion", GH_ParamAccess.item);
            pManager.AddTextParameter("Distance", "Dis", "The distance the end-effector can travel with", GH_ParamAccess.item);

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
        int ConvertDistanceToIntType(string disType)
        {
            int result = 0;
            switch (disType)
            {
                case "Short": result = 0; break;
                case "Long": result = 1; break;
                case "Longer": result = 2; break;
            }
            return result;
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool reg_input = false, end_input = false, addlock_input = false, pre_input = false, bake_input = false;
            double energy_input = 4;
            double speed_input = 4;
            string distance_input = "";

            #region input param readings
            if (!DA.GetData(0, ref reg_input))
                return;
            if (!DA.GetData(1, ref end_input))
                return;
            if (!DA.GetData(2, ref addlock_input))
                return;
            if (!DA.GetData(3, ref pre_input))
                return;
            if (!DA.GetData(4, ref energy_input))
                return;
            if (!DA.GetData(5, ref speed_input))
                return;
            if (!DA.GetData(6, ref distance_input))
                return;
            if (!DA.GetData(7, ref bake_input))
                return;
            #endregion

            // variables to control states
            bool toSelectRegion = false, toAdjustParam = false, toSetEndEffector = false, toAddLock = false, toRemoveLock = false, toPreview = false, toBake = false;

            #region Input check. This determines how the cell respond to changed params
            if (reg_input)//This applies to starting situation and when u change the input model
            {
                toSelectRegion = true;
            }
            if (end_input)
            {
                toSetEndEffector = true;
            }
            if (lockState != addlock_input)
            {
                lockState = addlock_input;
                if (lockState)
                    toAddLock = true;
                else
                    toRemoveLock = true;
            }
            if (pre_input)
            {
                toPreview = true;
            }

            if (energyLevel == energy_input && speedLevel == speed_input && distance == ConvertDistanceToIntType(distance_input))
            {
                toAdjustParam = false;
            }
            else
            {
                energyLevel = energy_input;
                speedLevel = speed_input;
                distance = ConvertDistanceToIntType(distance_input);
                toAdjustParam = true;
            }
            if (bake_input)
            {
                toBake = true;
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
                RhinoApp.KeyboardEvent += RhinoApp_KeyboardEvent;

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

                    #region Pre-process #2: get the ininital inner cavity of the selected brep

                    BoundingBox box = model.GetBoundingBox(true);
                    box.Inflate(-2.0);
                    box.Transform(Transform.Scale(box.Center, 2));
                    arrowScale = box.Diagonal.Length / 100;
                    center = box.Center;

                    #endregion

                    #region Step 1: adjust the orientation of two planes to chop the original brep

                    v = Vector3d.XAxis;
                    Rhino.Input.Custom.GetPoint gp1 = new Rhino.Input.Custom.GetPoint();
                    gp1.SetCommandPrompt(@"Press AS, ZX, or QW to rotate the partition planes around X, Y, or Z axis (CW and CCW). Press 'Enter' to continue.");
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

                    #endregion

                    #region Step 2: drag the two planes to decide the portion

                    Rhino.Input.Custom.GetPoint gp2 = new Rhino.Input.Custom.GetPoint();
                    gp2.SetCommandPrompt("Click and drag the partition plane to adjust their position. Press enter to confirm and move on");
                    gp2.MouseDown += Gp2_MouseDown;
                    gp2.MouseMove += Gp2_MouseMove;

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
                    PlaneSelected = true;

                    #endregion

                    #region Step 3: calculate and generate the inner cavity

                    RhinoDoc.ActiveDoc.Objects.Delete(ArrowCurve, true);
                    RhinoDoc.ActiveDoc.Objects.Delete(guid1, true);
                    RhinoDoc.ActiveDoc.Objects.Delete(guid2, true);
                    PlaneGenerated = false;
                    ArrowGenerated = false;

                    if (PlaneSelected)
                    {
                        // Call out the waiting window
                        processingwin.Show();

                        Plane p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), -v);
                        //p1Reverse.ExtendThroughBox(box, out _, out _);
                        Plane p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), -v);

                        //p2Reverse.ExtendThroughBox(box, out _, out _);
                        /*Brep[] Cut_Brep1 = m.Trim(pl1, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                        Brep Brep1 = Cut_Brep1[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);*/
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
                        Brep[] Cut_Brep2 = BrepRest.Trim(pl2, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                        Brep Brep2 = null;
                        try
                        {
                            Brep2 = Cut_Brep2[0];
                        }
                        catch
                        {
                            Brep2 = BrepRest;
                        }
                        try
                        {
                            Brep2 = Cut_Brep2[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                        }
                        catch
                        { }
                        /*Brep[] Cut_Brep3 = Cut_Brep1rest[0].Trim(p2Reverse, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                        Brep Brep3 = Cut_Brep3[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                        Rhino.Input.Custom.GetPoint ctrl_first_pt_sel = new Rhino.Input.Custom.GetPoint();*/

                        BoxLike b = new BoxLike(Brep2, v);
                        double volumn = 0;
                        Brep result1 = null;
                        Cylinder result2 = Cylinder.Unset;
                        Brep b2 = null;
                        double v_box = 0.0, v_cylinder = 0.0;
                        //if (type == 1)
                        //{

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
                                    // DA.SetData(1, result1);
                                }
                            }
                        }
                        //}
                        //else if (type == 2)
                        //{

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
                        //}
                        //else
                        //    throw new Exception("Invalid type");

                        if (v_box >= v_cylinder)
                            innerCavity = result1;
                        else
                            innerCavity = b2;
                        conBrep = Brep2;
                        direction = v;
                        //DA.SetData(0, Brep2);
                        //DA.SetData(2, skeleton);
                        //DA.SetData(3, v);
                        processingwin.Hide();

                        Transform cavityTranslation = Transform.Translation(Brep2.GetBoundingBox(true).Center - innerCavity.GetBoundingBox(true).Center);
                        innerCavity.Transform(cavityTranslation);
                    }

                    #endregion


                }
            }

            if (toSetEndEffector)
            {
                // Ask the user to select a Brep and calculate the orientation of the embedded kinetic unit
                ObjRef objSel_ref;
                Guid selObjId1 = Guid.Empty;
                var rc = RhinoGet.GetOneObject("Select a surface or polysurface as the end-effector", false, ObjectType.AnyObject, out objSel_ref);
                if (rc == Rhino.Commands.Result.Success)
                {
                    // select a brep
                    selObjId1 = objSel_ref.ObjectId;
                    ObjRef currObj = new ObjRef(selObjId1);

                    Brep endeffector = currObj.Brep();

                    Point3d ptS = skeleton.PointAtNormalizedLength(t1);
                    Point3d ptE = skeleton.PointAtNormalizedLength(t2);

                    Point3d tarPt = new Point3d();
                    Point3d springPosPt = new Point3d();
                    BoundingBox eeBoundingBox = endeffector.GetBoundingBox(true);
                    Point3d ee_center = eeBoundingBox.Center;
                    if (ee_center.DistanceTo(ptS) >= ee_center.DistanceTo(ptE))
                    {
                        tarPt = ptE;
                        springPosPt = ptS;
                    }
                    else
                    {
                        tarPt = ptS;
                        springPosPt = ptE;
                    }

                    Plane eeDirPlane = new Plane(tarPt, new Vector3d(ptS - ptE));
                    Point3d ptOnEE = endeffector.ClosestPoint(tarPt);
                    Point3d dirPt = eeDirPlane.ClosestPoint(ptOnEE);

                    Vector3d rawDir = new Vector3d(dirPt - tarPt);

                    // Currently, Kinergy only identifies four directions: eeDirPlane's XAxis (+/-) and YAxis (+/-)
                    double angleXPos = Vector3d.VectorAngle(rawDir, eeDirPlane.XAxis);
                    double angleXNeg = Vector3d.VectorAngle(rawDir, (-1) * eeDirPlane.XAxis);
                    double angleYPos = Vector3d.VectorAngle(rawDir, eeDirPlane.YAxis);
                    double angleYNeg = Vector3d.VectorAngle(rawDir, (-1) * eeDirPlane.YAxis);

                    if (angleXPos <= angleXNeg && angleXPos <= angleYPos && angleXPos <= angleYNeg)
                    {
                        orientationDir = eeDirPlane.XAxis;
                    }
                    else if (angleXNeg <= angleXPos && angleXNeg <= angleYPos && angleXNeg <= angleYNeg)
                    {
                        orientationDir = (-1) * eeDirPlane.XAxis;
                    }
                    else if (angleYPos <= angleXPos && angleYPos <= angleXNeg && angleYPos <= angleYNeg)
                    {
                        orientationDir = eeDirPlane.YAxis;
                    }
                    else if (angleYNeg <= angleXPos && angleYNeg <= angleXNeg && angleYNeg <= angleYPos)
                    {
                        orientationDir = (-1) * eeDirPlane.YAxis;
                    }

                    //Curve[] intersectTar;
                    //Point3d[] intersectTarPts;
                    //if (model == null) return;
                    //Rhino.Geometry.Intersect.Intersection.BrepPlane(model, eeDirPlane, myDoc.ModelAbsoluteTolerance, out intersectTar, out intersectTarPts);
                    //Curve strCrv = intersectTar[0];


                    #region Step 5: create an instance of Continuous Translation class

                    #region Parse energy and the speed

                    // Parse the dispalcement (in percentage) based on the spring length and the posible max compression dispacement
                    //Point3d ptS = skeleton.PointAtNormalizedLength(t1);
                    //Point3d ptE = skeleton.PointAtNormalizedLength(t2);
                    //double s_len = ptS.DistanceTo(ptE);
                    //double maxDisp = Math.Max(s_len - min_wire_diamter * min_coil_num, min_coil_num * 0.6);
                    //displacement = (displacementLevel + 1) / 10 * maxDisp / s_len;     // convert the input displacement level into percentage

                    // Parse the speed to gear ratio of the entire gear train
                    //if (speedLevel <= 4)
                    //{
                    //    speed = 5 - speedLevel;
                    //}
                    //else if (speedLevel < 9)
                    //{
                    //    speed = 1 - (speedLevel - 4) * 0.2;
                    //}
                    //else
                    //{
                    //    speed = 0.1;
                    //}
                    speed = speedLevel;
                    // Parse the energy to 0.1-1
                    energy = (energyLevel + 1) / 10;

                    #endregion
                    myDoc.Objects.Hide(selObjId, true);

                    motion = new ContinuousTranslation(model, direction, energy, speed, distance, innerCavity);      // the second argument represents if the skeleton is curved

                    #endregion

                    #region Create the gear train and the spring

                    #region Using orientationDir and tarPt as the axis of the last gear

                    // Transform from the current orientation and direction to X axis
                    Point3d startPoint = springPosPt;

                    Transform dirToXRotation = Transform.Rotation(direction, new Vector3d(1, 0, 0), startPoint);
                    Point3d projectedSpringPosPt = new Point3d(startPoint.X, 0, 0);
                    Transform dirToXTranlation = Transform.Translation(new Vector3d(projectedSpringPosPt - startPoint));

                    // Transform back from X axis to the current kinetic unit orientation
                    Transform dirToXTranlationBack = Transform.Translation(new Vector3d(startPoint - projectedSpringPosPt));
                    Transform dirToXRotationBack = Transform.Rotation(new Vector3d(1, 0, 0), direction, startPoint);

                    // Last step, rotate back to the pose of the kinetic unit
                    Vector3d originalYVector = new Vector3d(0, 1, 0);
                    originalYVector.Transform(dirToXTranlationBack);
                    originalYVector.Transform(dirToXRotationBack);
                    Transform yToPoseRotation = Transform.Rotation(originalYVector, orientationDir, startPoint);

                    // Start transform
                    startPoint.Transform(dirToXRotation);
                    startPoint.Transform(dirToXTranlation);

                    Point3d endPt = new Point3d(tarPt);
                    endPt.Transform(dirToXRotation);
                    endPt.Transform(dirToXTranlation);

                    double xEnd = endPt.X;
                    Brep modelDup = innerCavity.DuplicateBrep();
                    modelDup.Transform(dirToXRotation);
                    modelDup.Transform(dirToXTranlation);

                    double outDiameter = Math.Abs(modelDup.GetBoundingBox(true).Max.Z - modelDup.GetBoundingBox(true).Min.Z);
                    //double outDiameter = Double.MaxValue;

                    //foreach(var v in modelDup.Vertices)
                    //{
                    //    if (Math.Abs(v.Location.Z) < outDiameter / 2)
                    //        outDiameter = Math.Abs(v.Location.Z) * 2;
                    //}
                    double totalThickness = Math.Abs(modelDup.GetBoundingBox(true).Max.Y - modelDup.GetBoundingBox(true).Min.Y);
                    //double totalThickness = Double.MaxValue;

                    //foreach(var v in modelDup.Vertices)
                    //{
                    //    if (Math.Abs(v.Location.Y) < totalThickness / 2)
                    //        totalThickness = Math.Abs(v.Location.Y) * 2;
                    //}

                    double xSpaceEnd = modelDup.GetBoundingBox(true).Max.X;

                    motion.ConstructGearTrain(startPoint, xEnd, outDiameter, totalThickness, xSpaceEnd,
                        dirToXTranlationBack, dirToXRotationBack, yToPoseRotation,endeffector);

                    //// transform the generated brep of the gear train back to the orientation and placement of the kinetic unit
                    //foreach(Brep g in brepGearTrain)
                    //{
                    //    g.Transform(dirToXTranlationBack);
                    //    g.Transform(dirToXRotationBack);
                    //    g.Transform(yToPoseRotation);
                    //}

                    #endregion

                    #endregion
                }
            }
                if (toAddLock)
            {

            }

            if (toPreview)
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
        

        

        private void Gp2_MouseMove(object sender, Rhino.Input.Custom.GetPointMouseEventArgs e)
        {
            if (selected == Guid.Empty)
                return;
            //在这个函数里，e.Point是当前的鼠标所在的位置的对应的Rhino里的3D的点  
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
            //在这个函数里，e.Point是当前的鼠标所在的位置的对应的Rhino里的3D的点  
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

        private void GenerateArrow()
        {
            ArrowGenerated = true;
            RhinoDoc.ActiveDoc.Objects.Delete(ArrowCurve, true);
            Arrow a = new Arrow(v, center, arrowScale);
            ArrowCurve = RhinoDoc.ActiveDoc.Objects.Add(a.ArrowCurve);

            RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();
        }
        private void GeneratePlanes()
        {
            PlaneGenerated = true;
            //Delete these before generating new ones
            RhinoDoc.ActiveDoc.Objects.Delete(guid1, true);
            RhinoDoc.ActiveDoc.Objects.Delete(guid2, true);

            BoxLike b = new BoxLike(model, v);
            BoundingBox box = b.Bbox;
            Interval yInterval = new Interval(-(box.Max.Y - box.Min.Y) * 0.6, (box.Max.Y - box.Min.Y) * 0.6), zInterval = new Interval(-(box.Max.Z - box.Min.Z) * 0.6, (box.Max.Z - box.Min.Z) * 0.6);
            box.Transform(b.RotateBack);
            /*Point3d start = box.PointAt(0, 0.5, 0.5);
            Point3d end = box.PointAt(1, 0.5, 0.5);*/ //this doesn't work!

            skeleton = b.Skeleton;
            skeleton.Transform(b.RotateBack);
            skeletonVec = new Vector3d(skeleton.PointAtEnd) - new Vector3d(skeleton.PointAtStart);
            pl1 = new Plane(skeleton.PointAtNormalizedLength(t1), v);
            pl2 = new Plane(skeleton.PointAtNormalizedLength(t2), v);
            s1 = new PlaneSurface(pl1, yInterval, zInterval);
            s2 = new PlaneSurface(pl2, yInterval, zInterval);
            guid1 = RhinoDoc.ActiveDoc.Objects.Add(s1);
            guid2 = RhinoDoc.ActiveDoc.Objects.Add(s2);
            RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();
        }

        private void RhinoApp_KeyboardEvent(int key)
        {
            if (!OperatingArrow)
                return;

            if (key == 0x51)//Q
            {
                v.Transform(Transform.Rotation(Math.PI / 180 * 2.5, Vector3d.ZAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x57)//W
            {
                v.Transform(Transform.Rotation(-Math.PI / 180 * 2.5, Vector3d.ZAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x41)//A
            {
                v.Transform(Transform.Rotation(Math.PI / 180 * 2.5, Vector3d.XAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x53)//S
            {
                v.Transform(Transform.Rotation(-Math.PI / 180 * 2.5, Vector3d.XAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x5A)//Z
            {
                v.Transform(Transform.Rotation(Math.PI / 180 * 2.5, Vector3d.YAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
            }
            else if (key == 0x58)//X
            {
                v.Transform(Transform.Rotation(-Math.PI / 180 * 2.5, Vector3d.YAxis, Point3d.Origin));
                PlaneGenerated = false;
                GenerateArrow();
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