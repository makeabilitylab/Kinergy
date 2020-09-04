using System;
using System.Collections.Generic;
using Kinergy.KineticUnit;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System.Net.Configuration;
using Kinergy.Utilities;
namespace InstTwisting
{
    public class InstantTwistingComponent : GH_Component
    {
        InstantTwisting IT=null;
        Brep model = null;
        Vector3d mainDirection=Vector3d.Unset;
        Brep innerCylinder=null;
        bool addLock=false;
        bool addEE=false;
        double strength = 0, displacement = 0;
        /// <summary>
        /// Initializes a new instance of the _01ConstructKineticComponents_IT class.
        /// </summary>
        public InstantTwistingComponent()
          : base("InstantTwistingComponent", "InsTwi",
              "Construct an instant twisting kinetic unit with given parameters.",
              "Kinergy", "InstantTwisting")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Model", "M", "Primitive input model", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction", "D", "The main direction of model. Should be the output of preprocessing", GH_ParamAccess.item);
            pManager.AddGeometryParameter("InnerCylinder", "C", "The calculated inner cylinder for spiral spring", GH_ParamAccess.item);
            pManager.AddNumberParameter("Energy", "E", "The energy stored by this kinetic unit", GH_ParamAccess.item);
            pManager.AddNumberParameter("MaxAngle", "A", "The maximum spinning angle of knob", GH_ParamAccess.item);
            //pManager.AddVectorParameter("KnobDirection", "KD", "The knob direction. Should be same or reverse of main direction.", GH_ParamAccess.item); //No need to give. just let user set it
            pManager.AddBooleanParameter("AddLock", "L", "Whether to add lock. " +
                "Please be careful that everytime this value is turned positive, the lock would be reset.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("AddEndEffector", "EE", "Whether to add end effector. " +
                "Please be careful that everytime this value is turned positive, the end effector would be reset.", GH_ParamAccess.item);

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
            Brep model_input = null;
            Vector3d direction_input = Vector3d.Unset;
            Brep innerCylinder_input = null;
            double strength_input = 0, displacement_input = 0;
            bool adjustParameter = false,addLock_input=false,addEE_input=false;
            if (!DA.GetData(0, ref model_input))
                return;
            if (!DA.GetData(1, ref direction_input))
                return;
            if (!DA.GetData(2, ref innerCylinder_input))
                return;
            if (!DA.GetData(3, ref strength_input))
                return;
            if (!DA.GetData(4, ref displacement_input))
                return;
            if (!DA.GetData(5, ref addLock_input))
                return;
            if (!DA.GetData(6 , ref addEE_input))
                return;
            bool toGenerate = false, toAdjustParam = false, toSetInputHandler = false, toSetEndEffector = false, toAddLock = false, toRemoveLock = false, toRegenCam = false;
            #region input check. This determines how the cell respond to changed params
            if (model != model_input)//This applies to starting situation and when u change the input model
            {
                model = model_input;
                toGenerate = true;
            }
            if (mainDirection == Vector3d.Unset)
            {
                mainDirection = direction_input;
                toGenerate = true;
            }
            else if (mainDirection.IsParallelTo(direction_input) != 1)//This applies to starting situation and when u change the input direction
            {
                mainDirection = direction_input;
                toGenerate = true;
            }
            if (!toGenerate && !GeometryMethods.IfCylinderBrepIsEqual(innerCylinder, innerCylinder_input, mainDirection))
            {
                innerCylinder = innerCylinder_input;
                toGenerate = true;
            }
            if (addLock)
            {
                toSetInputHandler = true;
            }
            if (SE_input && SE != SE_input)
            {
                toSetEndEffector = true;
            }
            if (addLock != addLock_input)
            {
                addLock = addLock_input;
                if (addLock)
                    toAddLock = true;
                else
                    toRemoveLock = true;
            }
            if (strength_input == strength && displacement == displacement_input)//Params stays same, nothing happens
            {
                toAdjustParam = false;
            }
            else
            {
                strength = strength_input;
                displacement = displacement_input;
                toAdjustParam = true;
            }
            if (camType != camType_input)
            {
                camType = camType_input;
                if (!toGenerate)
                    toRegenCam = true;
            }
            #endregion
            if (IT!=null && adjustParameter)
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
                IT = new InstantTwisting(model, direction, innerCylinder, energy, maxAngle);
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