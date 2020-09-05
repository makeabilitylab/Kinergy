using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Kinergy.KineticUnit;
using Rhino.Geometry;

namespace InstTwisting
{
    public class _03EndEffector_IT : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the _03EndEffector_IT class.
        /// </summary>
        public _03EndEffector_IT()
          : base("_03EndEffector_IT", "Nickname",
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
            pManager.AddBrepParameter("EndEffectorModel", "EEM", "The end effector model. Pl place it along the given main direction", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Kinetic Unit", "KU", "Kinetic Unit instance of IE motion", GH_ParamAccess.item);
            pManager.AddBrepParameter("End Effector Components", "EEC", "Kinetic components generated in this cell", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep endEffectorModel = null;
            InstantTwisting IT = null;
            if (!DA.GetData(0, ref IT))
                return;
            if (!DA.GetData(1, ref endEffectorModel))
                return;
            IT.SetEndEffector(endEffectorModel);
            DA.SetData(0, IT);
            DA.SetDataList(1, IT.GetModel(new List<string> { "EndEffector" }));
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
            get { return new Guid("16cd4752-fdc9-4790-8433-324bef4658c2"); }
        }
    }
}