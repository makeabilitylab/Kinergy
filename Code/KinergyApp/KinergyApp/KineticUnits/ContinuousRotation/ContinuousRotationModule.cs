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

namespace HumanUIforKinergy.KineticUnits.ContinuousRotation
{
    public class ContinuousRotationModule : GH_Component
    {
        // Variables that store information
        Brep model;             // the original brep model
        Brep conBrep;           // the Brep that is selected and converted
        Brep innerCavity;
        double t1, t2; // the positoins of start point and end point of the segment on the normalized skeleton
        Curve skeleton;     // skeleton
        double energyLevel;         // value of the strength slide bar
        double speedLevel;   // value of the speed slide bar
        Vector3d direction;             // kinetic unit direction
        InstantRotation motion;
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
        /// Initializes a new instance of the ContinuousRotationModule class.
        /// </summary>
        public ContinuousRotationModule()
          : base("ContinuousRotationModule", "CRModule",
              "The kinetic unit for continuous rotation",
              "Kinergy", "ContinuousRotation")
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
            speed = 0.5;
            arrowScale = 0;
            isLockSet = false;
            selObjId = Guid.Empty;
            toBeBaked = new List<Guid>();
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
            pManager.AddNumberParameter("Speed", "V", "The speed/velocity of the output gear (RPM)", GH_ParamAccess.item);

            // Confirm and bake all components
            pManager.AddBooleanParameter("ComponentsBake", "Bk", "comfirm and bake all components", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Kinetic unit", "KU", "Kinetic Unit instance for instant translation", GH_ParamAccess.item);
            pManager.AddBrepParameter("Original brep", "Brep", "The target model to move", GH_ParamAccess.item);
            pManager.AddBrepParameter("Models", "M", "", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Preview launcher", "Pre", "enable the preview", GH_ParamAccess.item);
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
            if (!DA.GetData(6, ref bake_input))
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

            if (energyLevel == energy_input && speed == speed_input)
            {
                toAdjustParam = false;
            }
            else
            {
                energyLevel = energy_input;
                speed = speed_input;
                toAdjustParam = true;
            }
            if (bake_input)
            {
                toBake = true;
            }
            #endregion

            if (toBake)
            {
            }

            if (toSelectRegion)
            {

            }

            if (toSetEndEffector)
            {

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
            DA.SetData(1, model);
            if (motion == null)
                DA.SetDataList(2, null);
            else
                DA.SetDataList(2, motion.GetModel());
            DA.SetData(3, toPreview);
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