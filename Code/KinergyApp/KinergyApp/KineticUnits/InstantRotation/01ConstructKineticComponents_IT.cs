using System;
using System.Collections.Generic;
using Kinergy.KineticUnit;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System.Net.Configuration;

namespace InstTwisting
{
    public class _01ConstructKineticComponents_IT : GH_Component
    {
        InstantRotation IT =null;
        /// <summary>
        /// Initializes a new instance of the _01ConstructKineticComponents_IT class.
        /// </summary>
        public _01ConstructKineticComponents_IT()
          : base("_01ConstructKineticComponents_IT", "CKC_IT",
              "Construcct an instant twisting kinetic unit with given parameters.",
              "Kinergy", "InstantTwisting")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Model", "M", "Primitive input model", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction", "D", "The main direction of model.", GH_ParamAccess.item);
            pManager.AddGeometryParameter("InnerCylinder", "C", "The calculated inner cylinder for spiral spring", GH_ParamAccess.item);
            pManager.AddNumberParameter("Energy", "E", "The energy stored by this kinetic unit", GH_ParamAccess.item);
            pManager.AddNumberParameter("MaxAngle", "A", "The maximum spinning angle of knob", GH_ParamAccess.item);
            //pManager.AddVectorParameter("KnobDirection", "KD", "The knob direction. Should be same or reverse of main direction.", GH_ParamAccess.item); //No need to give. just let user set it
            pManager.AddBooleanParameter("AdjustingParameter", "Adjust", "Toggle for adjusting parameters", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Kinetic Unit", "KU", "Kinetic Unit instance of IE motion", GH_ParamAccess.item);
            pManager.AddBrepParameter("Kinetic Components", "KC", "Kinetic components generated in this cell", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep model = null;
            Vector3d direction = Vector3d.Unset;
            Brep innerCylinder = null;
            double energy = 0, maxAngle = 0;
            bool adjustParameter = false;
            if (!DA.GetData(0, ref model))
                return;
            if (!DA.GetData(1, ref direction))
                return;
            if (!DA.GetData(2, ref innerCylinder))
                return;
            if (!DA.GetData(3, ref energy))
                return;
            if (!DA.GetData(4, ref maxAngle))
                return;
            if (!DA.GetData(5, ref adjustParameter))
                return;
            if(IT!=null && adjustParameter)
            {
                //adjust parameter and update model
                //ExpireSolution(false);//TODO check if this works
                IT.AdjustParameter(energy,maxAngle);
                //DA.SetData(0, IT);
                DA.SetDataList(1, IT.GetModel(new List<string> { "Spiral","Axis"}));
            }
            else if(IT!=null)
            {
                //Construct the instant twisting object
                //IT = new InstantTwisting(model, direction, innerCylinder, energy, maxAngle);
                DA.SetData(0, IT);
                DA.SetDataList(1, IT.GetModel(new List<string> { "Spiral", "Axis" }));
            }
            else
            {
                IT = new InstantRotation(model, direction, innerCylinder, energy, maxAngle);
                DA.SetData(0, IT);
                DA.SetDataList(1, IT.GetModel(new List<string> { "Spiral", "Axis" }));
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
            get { return new Guid("67430225-0674-4e63-b70d-c1483251af2b"); }
        }
    }
}