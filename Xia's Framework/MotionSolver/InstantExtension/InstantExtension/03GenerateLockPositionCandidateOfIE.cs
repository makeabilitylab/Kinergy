using System;
using System.Collections.Generic;
using Kinergy.Motion;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy.Utilities;
namespace InstExtension
{
    public class _03GenerateLockPositionCandidateOfIE : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the _03GenerateLockPositionCandidateOfIE class.
        /// </summary>
        public _03GenerateLockPositionCandidateOfIE()
          : base("_03GenerateLockPositionCandidateOfIE", "Nickname",
              "Description",
              "Kinergy", "InstantExtension")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("Motion", "M", "Motion instance of IE motion", GH_ParamAccess.item);
            pManager.AddScriptVariableParameter("LockDirection", "D", "Arrow of selected lock direction", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Motion", "M", "Motion instance of IE", GH_ParamAccess.item);
            pManager.AddPointParameter("LockPositionCandidates", "L", "Available point positions of lock", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            InstantExtension motion = null;
            Arrow direction = null;
            List<Point3d> pts=new List<Point3d>();
            if (!DA.GetData(0, ref motion)) { return; }
            if (motion == null)
            { return; }
            if (!DA.GetData(1, ref direction)) { return; }
            motion.SetLockDirection(direction);
            pts=motion.GetLockPositionCandidates();
            DA.SetData(0, motion);
            DA.SetDataList(1,pts);
            
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
            get { return new Guid("25996350-a448-4352-9a83-42472bffe6a2"); }
        }
    }
}