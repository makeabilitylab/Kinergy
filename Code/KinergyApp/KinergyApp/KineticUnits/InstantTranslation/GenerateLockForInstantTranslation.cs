using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Kinergy.KineticUnit;
using Kinergy.Utilities;
using Rhino;
using Rhino.Geometry;

namespace InstTranslation
{
    public class GenerateLockForInstantTranslation : GH_Component
    {
        InstantTranslation motion;
        bool isLockSet = false;
        Arrow direction;
        int lockPos = 1; // 1: inside (by default); 2: outside
        bool toStart;
        /// <summary>
        /// Initializes a new instance of the GenerateLockForInstantTranslation class.
        /// </summary>
        public GenerateLockForInstantTranslation()
          : base("GenerateLockForInstantTranslation", "AddITLock",
              "Construct the lock for the instant translation",
              "Kinergy", "InstantTranslation")
        {
            toStart = false;
            motion = null;
            direction = null;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable the lock for instant translation", GH_ParamAccess.item);
            pManager.AddGenericParameter("Kinetic unit", "KU", "Kinetic Unit instance for instant translation", GH_ParamAccess.item);
            pManager.AddGenericParameter("Selected End-effector direction", "DIR", "SelectedEndEffector", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Kinetic unit", "KU", "Kinetic Unit instance for instant translation", GH_ParamAccess.item);
            pManager.AddBrepParameter("Models", "M", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool isStart = false;
            
            
            List<Point3d> pts = new List<Point3d>();

            if (!DA.GetData(0, ref isStart)) { return; }
            if (isStart == false){ toStart = false; }
            else{ toStart = true;  }

            if (!DA.GetData(1, ref motion)) {  }
            //if (motion == null)
            //{ return; }
            if (!DA.GetData(2, ref direction)) {  }

            if (toStart && motion != null && direction != null)
            {
                #region Set the lock inside or outside
                RhinoApp.KeyboardEvent += RhinoApp_KeyboardEvent;
                Rhino.Input.Custom.GetPoint gp1 = new Rhino.Input.Custom.GetPoint();
                gp1.SetCommandPrompt("Press \'i\' or \'o\' to set the lock inside or outside the body. Press enter to continue.");
                gp1.AcceptNothing(true);
                Rhino.Input.GetResult lock_pos_res;
                isLockSet = true;
                do
                {
                    lock_pos_res = gp1.Get(true);
                } while (lock_pos_res != Rhino.Input.GetResult.Nothing);
                isLockSet = false;
                #endregion

                #region Generate the lock position candidates and the user select the postion
                motion.SetLockDirection(direction);
                pts = motion.GetLockPositionCandidates();

                Point3d p;
                p = pts[UserSelection.UserSelectPointInRhino(pts, RhinoDoc.ActiveDoc)];
                #endregion

                #region Construct the lock based on the set position and type
                motion.SetLockPosition(p);  // is this step redundant?
                motion.CutModelForLock();
                motion.ConstructLock(lockPos);  // lockPos=1: inside; lockPos=2: outside

                #endregion

            }

            if (motion != null)
                DA.SetData(0, (KineticUnit)motion);
            else
                DA.SetData(0, null);

            if (motion != null)
                DA.SetDataList(1, motion.GetModel());
            else
                DA.SetDataList(1, null);
        }

        private void RhinoApp_KeyboardEvent(int key)
        {
            if (!isLockSet) return;

            if (key == 73 || key == 105) //"I" or "i"
            {
                // set the lock inside
                lockPos = 1;
            }
            else if (key == 79 || key == 111) //"O" or "o"
            {
                // set the lock outside
                lockPos = 2;
            }
            else
            {
                // set the lock by default--inside the body
                lockPos = 1;
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
            get { return new Guid("2aa17e88-c2ab-4918-bf00-408179f6a0bb"); }
        }
    }
}