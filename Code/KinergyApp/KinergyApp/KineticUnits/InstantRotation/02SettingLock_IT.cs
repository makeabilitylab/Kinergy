using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Kinergy.KineticUnit;
using Rhino.Geometry;

namespace InstTwisting
{
    public class _02SettingLock_IT : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the _02SettingLock_IT class.
        /// </summary>
        public _02SettingLock_IT()
          : base("_02SettingLock_IT", "Nickname",
              "Description",
              "Kinergy", "InstantTwisting")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("KineticUnit", "KU", "The generated kinetic unit", GH_ParamAccess.item);
            pManager.AddIntegerParameter("LockType", "LT", "The type of lock, only 1 is supported now", GH_ParamAccess.item);
            pManager[1].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Kinetic Unit", "KU", "Kinetic Unit instance of IE motion", GH_ParamAccess.item);
            pManager.AddBrepParameter("Lock Components", "LC", "Kinetic components generated in this cell", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            InstantRotation IT =null;
            int type = 1;
            if (!DA.GetData(0, ref IT))
                return;
            DA.GetData(1, ref type);
            IT.SetLockPosition_Old();
            IT.ConstructLocks_Old();
            DA.SetData(0, IT);
            DA.SetDataList(1, IT.GetModel(new List<string> { "Locks"}));
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
            get { return new Guid("a0482f62-6bd0-4aed-b908-3cd68f6f468a"); }
        }
    }
}