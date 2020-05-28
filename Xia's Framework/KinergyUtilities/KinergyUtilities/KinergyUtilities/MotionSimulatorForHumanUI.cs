using System;
using System.Collections.Generic;
using Kinergy.Motion;
using Kinergy;
using Grasshopper.Kernel;
using Rhino.Geometry;

using System.Threading;
namespace KinergyUtilities
{
    public class MotionSimulatorForHumanUI : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MotionSimulatorForHumanUI class.
        /// </summary>
        public MotionSimulatorForHumanUI()
          : base("MotionSimulatorForHumanUI", "MSHUI",
              "Simulate the given motion. You could ",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("Motion", "M", "The motion to simulate", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Count", "C", "Count of iteration times", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Start", "S", "Start simulating", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Motion", "M", "The motion to simulate", GH_ParamAccess.item);
            //pManager.AddGenericParameter("Models", "M", "The models", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Converge", "C", "Whether the simulation iteration converge", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Motion m = null;
            int interval = 100;
            bool start = false;
            int count=0;
            if (!DA.GetData(0, ref m)) { return; }
            if (m == null)
            { return; }
            if (!DA.GetData(1, ref count)) {  return; }
            if (!DA.GetData(2, ref start)) {  return; }
            if(start==false && count==0)
            {
                DA.SetData(0, m);// DA.SetData(1, true); 
                return;
            }
            if(m.Loaded==false)
            {
                DA.SetData(0, m); DA.SetData(1, true);
                return;
            }
            if(count==0)
            {
                m.TriggerWithoutInteraction();
            }
            Rhino.RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();//This line redraws view
            Rhino.RhinoApp.Wait();//This line make sure that the rhino interface is operatable and always rendering.
            Movement move;
            string body = string.Format("The simulation run for {0} time",count+1);
            Rhino.RhinoApp.WriteLine(body);
            move = m.Simulate(interval);
            if (move.Converge == true)
            {
                Rhino.RhinoApp.Wait();
                Thread.Sleep(1000);
                m.ResetMotion();
            }
            DA.SetData(0, m);
            //DA.SetData(1, move.Converge);
            DA.SetData(1, false);
            //this.ExpireSolution(true);

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
            get { return new Guid("41dd6531-f44f-4755-98f0-ae297432fe32"); }
        }
    }
}