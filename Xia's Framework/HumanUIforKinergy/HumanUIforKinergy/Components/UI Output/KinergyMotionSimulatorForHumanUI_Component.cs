using System;
using System.Collections.Generic;
using Kinergy.Motion;
using Kinergy;
using System.Threading;
using Grasshopper.Kernel;
using Rhino.Geometry;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;
namespace HumanUIforKinergy.Components.UI_Output
{
    public class KinergyMotionSimulatorForHumanUI : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the KinergyMotionSimulatorForHumanUI class.
        /// </summary>
        public KinergyMotionSimulatorForHumanUI()
          : base("KinergyMotionSimulatorForHumanUI", "KMS",
              "Allows you to simulate a motion and show it in an existing 3D view",
              "Human UI", "UI Output")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("3D View", "V", "The 3D view to adopt", GH_ParamAccess.item);
            pManager.AddGenericParameter("Motion to simulate", "M", "The motion to simulate in the viewport", GH_ParamAccess.item);
            pManager.AddColourParameter("Mesh Colors", "C", "The color with which to display the mesh.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Load", "L", "Set this to true to release the motion and see the simulation", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Release", "R", "Set this to true to release the motion and see the simulation", GH_ParamAccess.item);
            
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            object o = null;
            Motion m = null;
            List<Mesh> models;
            bool release = false;
            bool load = false;
            List<System.Drawing.Color> cols = new List<System.Drawing.Color>();
            if (!DA.GetData<object>("3D View", ref o)) return;
            DA.GetData(1, ref m);
            DA.GetDataList<System.Drawing.Color>("Mesh Colors", cols);
            DA.GetData(3, ref load);
            DA.GetData(4, ref release);

            HelixViewport3D vp3 = HUI_Util.GetUIElement<HelixViewport3D>(o);
            ModelVisual3D mv3 = GetModelVisual3D(vp3);
            List<ModelVisual3D> mv3s = GetModels(vp3);

            if (load && m.Loaded==false)
            {
                m.LoadMotion();
                models = m.GetMeshModel();
                vp3.Children.Clear();
                vp3.Children.Add(new SunLight());

                mv3.Content = new _3DViewModel(models, cols).Model;

                vp3.Children.Add(mv3);
                return;
            }
            if(m.Loaded==false)
            { return; }
            if (release==false)
            { return; }
            
            
            //iterate the simulation loop. Update the view every frame
            m.TriggerWithoutInteraction();
            int times = 0;
            int interval = 100;
            Movement move;
            do
            {
                times++;
                string body = string.Format("The simulation run for {0} time", times);
                Rhino.RhinoApp.WriteLine(body);
                move = m.Simulate(interval);
                models = m.GetMeshModel();

                vp3.Children.Clear();
                vp3.Children.Add(new SunLight());

                mv3.Content = new _3DViewModel(models, cols).Model;
                
                vp3.Children.Add(mv3);

                Rhino.RhinoDoc.ActiveDoc.Views.ActiveView.Redraw();//This line redraws view
                Rhino.RhinoApp.Wait();//This line make sure that the rhino interface is operatable and always rendering.
                                      //Thread.Sleep(interval-30);//wait for some time to match the actual frame speed. Since calculation takes time, the sleeping time is 30ms shorter than interval.

            } while (move.Converge == false);
            Rhino.RhinoApp.Wait();
            Thread.Sleep(1000);
            m.ResetMotion();
            models = m.GetMeshModel();

            vp3.Children.Clear();
            vp3.Children.Add(new SunLight());
            mv3.Content = new _3DViewModel(models, cols).Model;
            vp3.Children.Add(mv3);
        }
        ModelVisual3D GetModelVisual3D(HelixViewport3D vp3)
        {
            foreach (Visual3D v in vp3.Children)
            {
                if (v is ModelVisual3D)
                {
                    return v as ModelVisual3D;
                }
            }
            return null;

        }

        List<ModelVisual3D> GetModels(HelixViewport3D vp3)
        {
            List<ModelVisual3D> models = new List<ModelVisual3D>();
            foreach (Visual3D v in vp3.Children)
            {
                if (v is ModelVisual3D)
                {
                    models.Add(v as ModelVisual3D);
                }

            }
            return models;

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
            get { return new Guid("766f6a4c-2e82-4655-b283-215f105f0095"); }
        }
    }
}