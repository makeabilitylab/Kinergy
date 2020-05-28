using System;
using System.Collections.Generic;
using Kinergy.Motion;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace InstantExtension
{
    public class _04ConstructLockOfIE : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the _04ConstructLockOfIE class.
        /// </summary>
        public _04ConstructLockOfIE()
          : base("_04ConstructLockOfIE", "ConstructLock",
              "Construct lock for IE motion",
              "Kinergy", "InstantExtension")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("Motion", "M", "Motion instance of IE motion", GH_ParamAccess.item);
            pManager.AddPointParameter("LockPosition", "P", "Position of lock", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Motion", "M", "Motion instance of IE", GH_ParamAccess.item);
            pManager.AddBrepParameter("Models", "M", "", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            HelicalSpring motion = null;
            Point3d position = Point3d.Unset;
            if (!DA.GetData(0, ref motion)) { return; }
            if (motion == null)
            { return; }
            if (!DA.GetData(1, ref position)) { return; }
            motion.SetLockPosition(position);
            motion.CutModelForLock();
            motion.ConstructLock();
            DA.SetData(0, motion);
            DA.SetDataList(1, motion.GetModel());
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
            get { return new Guid("048e9430-f894-407d-a3cb-ad54b1ddee5a"); }
        }
    }
}