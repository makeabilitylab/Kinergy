using System;
using System.Collections.Generic;
using Kinergy.Utilities;
using Kinergy.Motion;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy;
namespace KinergyUtilities
{
    public class MotionSimulator : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MotionSimulator class.
        /// </summary>
        public MotionSimulator()
          : base("MotionSimulator", "MS",
              "Start the simulation of a motion",
             "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("Motion", "M", "The motion to simulate", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Start", "S", "Start simulating", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Motion", "M", "The motion to simulate", GH_ParamAccess.item);
            pManager.AddBrepParameter("Model", "M", "The simulating models", GH_ParamAccess.list);
        }
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Motion m = null;
            bool start = false;
            if (!DA.GetData(0, ref m)) { return; }
            if (m == null)
            { return; }
            if (!DA.GetData(1, ref start)) { return; }
            if(start==false)
            { return; }
            DA.SetData(0, m);
            if(m.LoadMotion())
            {
                Movement move;
                do
                {
                    move = m.Simulate(this);
                    DA.SetDataList(1, m.GetModel());
                } while (move.MovementValue > 0.01);
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
            get { return new Guid("190535c7-fa0c-4641-b503-d2d71aa9910c"); }
        }
    }
}