using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace HumanUIforKinergy.InterfaceLogic
{
    public class BtnSelectionComponent : GH_Component
    {
        int motionType;
        //bool testBtnClick_InsTrans;
        //bool testBtnClick_InsRot;
        //bool testBtnClick_ConTrans;
        //bool testBtnClick_ConRot;
        //bool testBtnClick_Reci;
        //bool testBtnClick_InterOscil;
        //bool testBtnClick_InterRot;

        /// <summary>
        /// Initializes a new instance of the BtnSelectionComponent1cs class.
        /// </summary>
        public BtnSelectionComponent()
          : base("BtnSelectionComponent", "MotionSelection",
              "Detect which motion is selected",
              "Kinergy", "InterfaceLogic")
        {
            //testBtnClick_InsTrans = false;
            //testBtnClick_InsRot = false;
            //testBtnClick_ConTrans = false;
            //testBtnClick_ConRot = false;
            //testBtnClick_Reci = false;
            //testBtnClick_InterOscil = false;
            //testBtnClick_InterRot = false;
            motionType = 0;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("InsTransBtnClicked", "InsTrans", "Instant translation button clicked", GH_ParamAccess.item);
            pManager.AddBooleanParameter("InsRotationBtnClicked", "InsRot", "Instant rotation button clicked", GH_ParamAccess.item);
            pManager.AddBooleanParameter("ConTransBtnClicked", "ConTrans", "Continuous translation button clicked", GH_ParamAccess.item);
            pManager.AddBooleanParameter("ConRotBtnClicked", "ConRot", "Continuous rotation button clicked", GH_ParamAccess.item);
            pManager.AddBooleanParameter("RecipBtnClicked", "Recip", "Reciprocation button clicked", GH_ParamAccess.item);
            pManager.AddBooleanParameter("InterOscillationBtnClicked", "InterOscil", "Intermittent oscillation button clicked", GH_ParamAccess.item);
            pManager.AddBooleanParameter("InterRotBtnClicked", "InterRot", "Intermittent rotation button clicked", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBooleanParameter("InsTransPanel", "InsTransPanel", "Instant translation panel actvated", GH_ParamAccess.item);
            pManager.AddBooleanParameter("InsRotationPanel", "InsRotPanel", "Instant rotation panel activated", GH_ParamAccess.item);
            pManager.AddBooleanParameter("ConTransPanel", "ConTransPanel", "Continuous translation panel activated", GH_ParamAccess.item);
            pManager.AddBooleanParameter("ConRotPanel", "ConRotPanel", "Continuous rotation panel activated", GH_ParamAccess.item);
            pManager.AddBooleanParameter("RecipPanel", "RecipPanel", "Reciprocation panel activated", GH_ParamAccess.item);
            pManager.AddBooleanParameter("InterOscillationPanel", "InterOscilPanel", "Intermittent oscillation panel activated", GH_ParamAccess.item);
            pManager.AddBooleanParameter("InterRotPanel", "InterRotPanel", "Intermittent rotation panel activated", GH_ParamAccess.item);
            pManager.AddBooleanParameter("DefaultPanel", "DefaultPanel", "Default panel activated", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool btnClick_instrans = false;
            bool btnClick_insrot = false;
            bool btnClick_contrans = false;
            bool btnClick_conrot = false;
            bool btnClick_recip = false;
            bool btnClick_interosci = false;
            bool btnClick_interrot = false;

            #region read the button clicks

            if (!DA.GetData(0, ref btnClick_instrans))
                return;

            if (!DA.GetData(1, ref btnClick_insrot))
                return;

            if (!DA.GetData(2, ref btnClick_contrans))
                return;

            if (!DA.GetData(3, ref btnClick_conrot))
                return;

            if (!DA.GetData(4, ref btnClick_recip))
                return;

            if (!DA.GetData(5, ref btnClick_interosci))
                return;

            if (!DA.GetData(6, ref btnClick_interrot))
                return;

            #endregion

            
            if (motionType == 1 && !btnClick_instrans)
            {
                DA.SetData(0, true);
                DA.SetData(1, false);
                DA.SetData(2, false);
                DA.SetData(3, false);
                DA.SetData(4, false);
                DA.SetData(5, false);
                DA.SetData(6, false);
                DA.SetData(7, false);
            }
            else if (btnClick_instrans && !btnClick_insrot && !btnClick_contrans && !btnClick_conrot &&
                !btnClick_recip && !btnClick_interosci && !btnClick_interrot)
            {
                motionType = 1;
            }

            if (motionType == 2 && !btnClick_insrot)
            {
                DA.SetData(0, false);
                DA.SetData(1, true);
                DA.SetData(2, false);
                DA.SetData(3, false);
                DA.SetData(4, false);
                DA.SetData(5, false);
                DA.SetData(6, false);
                DA.SetData(7, false);
            }
            else if (!btnClick_instrans && btnClick_insrot && !btnClick_contrans && !btnClick_conrot &&
                !btnClick_recip && !btnClick_interosci && !btnClick_interrot)
            {
                motionType = 2;
            }

            if (motionType == 3 && !btnClick_contrans)
            {
                DA.SetData(0, false);
                DA.SetData(1, false);
                DA.SetData(2, true);
                DA.SetData(3, false);
                DA.SetData(4, false);
                DA.SetData(5, false);
                DA.SetData(6, false);
                DA.SetData(7, false);
            }
            else if (!btnClick_instrans && !btnClick_insrot && btnClick_contrans && !btnClick_conrot &&
                !btnClick_recip && !btnClick_interosci && !btnClick_interrot)
            {
                motionType = 3;
            }

            if (motionType == 4 && !btnClick_conrot)
            {
                DA.SetData(0, false);
                DA.SetData(1, false);
                DA.SetData(2, false);
                DA.SetData(3, true);
                DA.SetData(4, false);
                DA.SetData(5, false);
                DA.SetData(6, false);
                DA.SetData(7, false);
            }
            else if (!btnClick_instrans && !btnClick_insrot && !btnClick_contrans && btnClick_conrot &&
                !btnClick_recip && !btnClick_interosci && !btnClick_interrot)
            {
                motionType = 4;
            }

            if (motionType == 5 && !btnClick_recip)
            {
                DA.SetData(0, false);
                DA.SetData(1, false);
                DA.SetData(2, false);
                DA.SetData(3, false);
                DA.SetData(4, true);
                DA.SetData(5, false);
                DA.SetData(6, false);
                DA.SetData(7, false);
            }
            else if (!btnClick_instrans && !btnClick_insrot && btnClick_contrans && !btnClick_conrot &&
                btnClick_recip && !btnClick_interosci && !btnClick_interrot)
            {
                motionType = 5;
            }

            if (motionType == 6 && !btnClick_interosci)
            {
                DA.SetData(0, false);
                DA.SetData(1, false);
                DA.SetData(2, false);
                DA.SetData(3, false);
                DA.SetData(4, false);
                DA.SetData(5, true);
                DA.SetData(6, false);
                DA.SetData(7, false);
            }
            else if (!btnClick_instrans && !btnClick_insrot && btnClick_contrans && !btnClick_conrot &&
                !btnClick_recip && btnClick_interosci && !btnClick_interrot)
            {
                motionType = 6;
            }

            if (motionType == 7 && !btnClick_interrot)
            {
                DA.SetData(0, false);
                DA.SetData(1, false);
                DA.SetData(2, false);
                DA.SetData(3, false);
                DA.SetData(4, false);
                DA.SetData(5, false);
                DA.SetData(6, true);
                DA.SetData(7, false);
            }
            else if (!btnClick_instrans && !btnClick_insrot && btnClick_contrans && !btnClick_conrot &&
                !btnClick_recip && !btnClick_interosci && btnClick_interrot)
            {
                motionType = 7;
            }
            
            if(motionType == 0)
            {
                DA.SetData(0, false);
                DA.SetData(1, false);
                DA.SetData(2, false);
                DA.SetData(3, false);
                DA.SetData(4, false);
                DA.SetData(5, false);
                DA.SetData(6, false);
                DA.SetData(7, true);
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
            get { return new Guid("864f67e0-ba82-465f-bdb2-40a64eeb3b6d"); }
        }
    }
}