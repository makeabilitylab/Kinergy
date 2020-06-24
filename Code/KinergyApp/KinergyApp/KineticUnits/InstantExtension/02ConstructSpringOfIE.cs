using System;
using System.Collections.Generic;
using Kinergy.KineticUnit;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy.Utilities;
namespace InstExtension
{
    public class _02ConstructSpringOfIE : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the _02ConstructSpringOfIE class.
        /// </summary>
        public _02ConstructSpringOfIE()
          : base("_02ConstructSpringOfIE", "ConstructSpring",
              "Construct spring for IE motion",
              "Kinergy", "InstantExtension")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("KineticUnit", "KU", "Kinetic Unit instance of IE motion", GH_ParamAccess.item);
            pManager.AddPointParameter("SpringPosition", "P", "Position of spring", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Kinetic Unit", "KU", "Kinetic Unit instance of IE", GH_ParamAccess.item);
            pManager.AddGenericParameter("LockDirectionCandidates", "DC", "Available directions of lock as arrows. Discard this if you don't need lock", GH_ParamAccess.list);
            pManager.AddGenericParameter("Spring", "S", "spring entity", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            InstantExtension motion = null;
            Point3d pos = Point3d.Unset;
            if (!DA.GetData(0, ref motion)) { return; }
            if (!DA.GetData(1, ref pos)) { return; }
            if(motion==null)
            { return; }
            if(pos==Point3d.Unset)
            { return; }
            motion.SetSpringPosition(pos);
            motion.CutModelForSpring();
            motion.ConstructSpring();
            List<Arrow> directionCandidates;
            directionCandidates = motion.GetLockDirectionCandidates();
            DA.SetData(0, motion);
            DA.SetDataList(1, directionCandidates);
            DA.SetData(2, motion.Spring);
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
            get { return new Guid("a2ce1f14-7b44-4e94-ac27-8f5f97ef23da"); }
        }
    }
}