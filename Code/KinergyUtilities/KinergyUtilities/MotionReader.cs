using System;
using System.Collections.Generic;
using Kinergy.Motion;
using Kinergy.Geom;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace KinergyUtilities
{
    public class MotionReader : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MotionReader class.
        /// </summary>
        public MotionReader()
          : base("MotionReader", "MR",
              "Read entities from a motion and extract their brep models",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("Motion", "M", "Motion instance to read and extract",GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("Entities", "E", "processed input model", GH_ParamAccess.list);
            pManager.AddBrepParameter("Components", "C", "component entities", GH_ParamAccess.list);
            pManager.AddBrepParameter("Locks", "L", "locks", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Motion motion=null;
            List<Brep> E = new List<Brep>();
            List<Brep> C = new List<Brep>();
            List<Brep> L = new List<Brep>();
            if (!DA.GetData(0, ref motion)) { return; }
            if(motion==null)
            { return; }
            if(motion.EntityList.Count==0)
            { return; }
            
            foreach (Entity e in motion.EntityList)
            {
                if(e.GetType()==typeof(Spring) || e.GetType() == typeof(Gear) || e.GetType() == typeof(Spiral) || e.GetType() == typeof(Rack))
                {
                    C.Add(e.GetModelinWorldCoordinate());
                }
                else if(e.GetType() == typeof(Lock))
                {
                    L.Add(e.GetModelinWorldCoordinate());
                }
                else
                {
                    E.Add(e.GetModelinWorldCoordinate());
                }
            }
            DA.SetDataList(0,E);
            DA.SetDataList(1,C);
            DA.SetDataList(2, L);
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
            get { return new Guid("fb635901-6925-45e3-aabc-a6f6f34500c8"); }
        }
    }
}