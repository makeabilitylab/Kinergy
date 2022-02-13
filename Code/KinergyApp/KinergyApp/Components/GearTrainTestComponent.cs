using System;
using System.Collections.Generic;
using KinergyUtilities;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy.Geom;

namespace HumanUIforKinergy.Components
{
    public class GearTrainTestComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public GearTrainTestComponent()
          : base("GearTrain", "GT",
              "Testing cell for gear train",
              "Kinergy", "Components")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddVectorParameter("Axis Direction", "Axis", "The axis direction, i.e. norm direction of all gears.", GH_ParamAccess.item);
            pManager.AddPointParameter("First Gear Center", "FGCT", "The center position of the first gear in gear train. Note that it would be exactly in this place without any offset.", GH_ParamAccess.item);
            pManager.AddPointParameter("Last Gear Center", "LGCT", "The center position of the last gear in gear train. Note that it could have offset along axis direction.", GH_ParamAccess.item);
            pManager.AddBoxParameter("Inner Cavity Box", "ICB", "Inner cavity of shape, i.e. allowed space for gear train. As a box that has world axis",GH_ParamAccess.item);
            pManager.AddNumberParameter("Gear Face Width", "FW", "The thickness of gear", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Gear Ratio User Selection", "GRUS", "A integer between 1-10 indicating the gear ratio that user selected.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("Models", "M", "The brep models of gears", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Vector3d axis=Vector3d.Unset;
            if (!DA.GetData(0, ref axis)) { return; }
            Point3d fgct = Point3d.Unset;
            if (!DA.GetData(1, ref fgct)) { return; }
            Point3d lgct = Point3d.Unset;
            if (!DA.GetData(2, ref lgct)) { return; }
            Box innerCavity = Box.Unset;
            if (!DA.GetData(3, ref innerCavity)) { return; }
            double faceWidth = 0;
            if (!DA.GetData(4, ref faceWidth)) { return; }
            int selection = 0;
            if (!DA.GetData(5, ref selection)) { return; }
            List<GearParameter> parameters = GenerateGearTrain.GetGearTrainByParameter(axis, fgct, lgct, innerCavity, faceWidth, selection);
            //Generate all the gears and output
            List<Brep> models = new List<Brep>();
            for(int i=0;i<parameters.Count;i++)
            {
                GearParameter p = parameters[i];
                Gear newGear = new Gear(p.center, p.norm, p.xDirection,(int)Math.Floor(p.radius*2), 1, 20, faceWidth, 0, false);
                newGear.Generate();
                models.Add(newGear.Model);
            }
            DA.SetDataList(0, models);
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
            get { return new Guid("db5c8544-a9e9-4f19-b065-06fb009d54d6"); }
        }
    }
}