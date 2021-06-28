using System;
using System.Collections.Generic;
using KinergyUtilities;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino;
namespace KinergyUtilities
{
    public class SelectPosition : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the SelectPosition class.
        /// </summary>
        public SelectPosition()
          : base("SelectPosition", "SP",
              "Select one Point position among a list",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("PositionCandidates", "P", "List of points to select from", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Select", "S", "Whether to select", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("SelectedPoint", "P", "SelectedPoint", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Point3d> pts=new List<Point3d>();
            bool start = false;
            Point3d p;
            if (!DA.GetDataList(0, pts)) { return; }
            if(pts.Count==0)
            { return; }
            DA.GetData(1, ref start);
            if(start==false)
            { return; }
            else
            {
                p=pts[UserSelection.UserSelectPointInRhino(pts, RhinoDoc.ActiveDoc)];
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
            get { return new Guid("9e6e905e-2b75-4d40-bbf1-6477d3c1cb52"); }
        }
    }
}