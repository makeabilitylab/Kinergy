using System;
using System.Collections.Generic;
using Kinergy.Utilities;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino;
namespace KinergyUtilities
{
    public class SelectDirection : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SelectDirection class.
        /// </summary>
        public SelectDirection()
          : base("SelectDirection", "SD",
              "Select one arrow among a list",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddScriptVariableParameter("DirectionCandidates", "D", "Candidate directions as arrows", GH_ParamAccess.list);
            
            pManager.AddBooleanParameter("Select", "S", "Whether to select", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("SelectedArrow", "A", "SelectedArrow", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Arrow> arrows = new List<Arrow>();
            bool start = false;
            Arrow p = null;
            if (!DA.GetDataList(0, arrows)) { return; }
            if(arrows.Count==0)
            { return; }
            DA.GetData(1, ref start);
            if (start == false)
            { return; }
            else
            {
                List<Curve> arrowCurves = new List<Curve>();
                foreach(Arrow a in arrows)
                {
                    arrowCurves.Add(a.ArrowCurve);
                }
                p = arrows[UserSelection.UserSelectCurveInRhino(arrowCurves, RhinoDoc.ActiveDoc)];
            }
            
                DA.SetData(0, p);
            

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
            get { return new Guid("5e1b64df-aa49-43b0-be34-fb17b1f253ac"); }
        }
    }
}