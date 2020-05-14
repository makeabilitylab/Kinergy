using System;
using System.Collections.Generic;
using Kinergy.Utilities;
using Kinergy.Motion;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.DocObjects;
using Kinergy;
using System.Threading;
namespace KinergyUtilities
{
    public class MotionSimulator : GH_Component
    {
        
        Motion m = null;
        int interval = 100;
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
            //pManager.AddIntegerParameter("Interval", "I", "Interval of simulation, in millimeters. Default value is 20ms.", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            //pManager.AddGenericParameter("Motion", "M", "The motion to simulate", GH_ParamAccess.item);
            pManager.AddGenericParameter("Models", "M", "The models", GH_ParamAccess.list);
        }
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            
            bool start = false;
            if (!DA.GetData(0, ref m)) { return; }
            if (m == null)
            { return; }
            if (!DA.GetData(1, ref start)) { return; }
            //if (!DA.GetData(2, ref interval)) { interval=20; }
            if (start==false)
            {
                DA.SetDataList(0, m.GetModel());
                return; 
            }
            /*if(interval<30)
            { interval=30; }
            if (interval > 100)
            { interval = 100; }*/
            m.LoadMotion();
            DA.SetDataList(0, m.GetModel());
            //Refresh the window
            Rhino.RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();//This line redraws view
            Rhino.RhinoApp.Wait();
            if (m.Trigger())
            {
                int times = 0;
                Movement move;
                do
                {
                    times++;
                    string body = string.Format("The simulation run for {0} time",times);
                    Rhino.RhinoApp.WriteLine(body);
                    move = m.Simulate(interval);
                    ClearData();
                    DA.SetDataList(0, m.GetModel());
                    //Refresh the window
                    Rhino.RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();//This line redraws view
                    Rhino.RhinoApp.Wait();//This line make sure that the rhino interface is operatable and always rendering.
                    //Thread.Sleep(interval-30);//wait for some time to match the actual frame speed. Since calculation takes time, the sleeping time is 30ms shorter than interval.

                } while (move.Converge==false);
                Rhino.RhinoApp.Wait();
                Thread.Sleep(1000);
                m.ResetMotion();
                ClearData();
                DA.SetDataList(0, m.GetModel());
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