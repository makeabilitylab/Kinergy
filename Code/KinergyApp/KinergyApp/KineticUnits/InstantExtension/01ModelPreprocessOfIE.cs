using System;
using System.Collections.Generic;
using Kinergy.KineticUnit;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace InstExtension
{
    public class _01ModelPreprocessOfIE : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the _01ModelPreprocessOfIE class.
        /// </summary>
        public _01ModelPreprocessOfIE()
          : base("_01ModelPreprocessOfIE", "PreprocessIE",
              "Description",
              "Kinergy", "InstantExtension")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("Kinetic Unit", "KU", "Kinetic Unit instance", GH_ParamAccess.item);
            
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Kinetic Unit", "KU", "Motion instance", GH_ParamAccess.item);
            pManager.AddPointParameter("SpringPositionCandidates", "P", "Candidate points of available point position", GH_ParamAccess.list);
            pManager.AddCurveParameter("ModelSkeleton", "S", "", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            InstantExtension motion = null;
            if (!DA.GetData(0, ref motion)) { return; }
            if(motion==null)
            { return; }
            if(motion.Curved)
            {
                motion.CalculateCurvedSkeleton();
            }
            else
            {
                motion.CalculateStraightSkeleton();
            }

            List<Point3d> pts = motion.GetSpringPositionCandidates();
            DA.SetData(0, motion);
            DA.SetDataList(1, pts);
            DA.SetData(2,motion.Skeleton);
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
            get { return new Guid("490db32e-60bf-42ed-b97c-0b1f4265ef93"); }
        }
    }
}