using System;
using System.Collections.Generic;
using KinergyUtilities;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino;
using Rhino.DocObjects;
using Rhino.Input;

namespace KinergyUtilities
{
    public class EndEffectorSelectionForHelical : GH_Component
    {
        bool toStart;
        bool valueObtained;
        List<Arrow> arrows;
        /// <summary>
        /// Initializes a new instance of the EndEffectorSelectionForHelical class.
        /// </summary>
        public EndEffectorSelectionForHelical()
          : base("EndEffectorSelectionForHelical", "End-EffectorSelection",
              "Select the end-effector for a helical spring",
              "Kinergy", "Utilities")
        {
            toStart = false;
            valueObtained = false;
            arrows = new List<Arrow>();
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Select", "S", "Whether to select the end-effector", GH_ParamAccess.item);
            pManager.AddGenericParameter("DirectionCandidates", "D", "Candidate directions as arrows", GH_ParamAccess.list);
            //pManager.AddBooleanParameter("Enabled", "EN", "Whether or not values passed to this battery", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("SelectedEndEffectorDirection", "DIR", "SelectedEndEffector", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            
            bool start = false;
            Arrow p = null;

            if (!DA.GetDataList(1, arrows)) { return; }

            //if (!DA.GetData(2, ref valueObtained)) { return; }
            //if (valueObtained == false) { return; }

            if (!DA.GetData(0, ref start)) { return; }
            if (start == true) { toStart = true; }
            else { toStart = false; }

            //if (start == true && valueObtained == true){ toStart = true; }
            //else{ toStart = false; }
            
            if (toStart)
            {
                List<Curve> arrowCurves = new List<Curve>();
                foreach (Arrow a in arrows)
                {
                    arrowCurves.Add(a.ArrowCurve);
                }

                // Ask the user to select a Brep and calculate which arrow is closer to the selected Brep
                // p = arrows[UserSelection.UserSelectCurveInRhino(arrowCurves, RhinoDoc.ActiveDoc)];

                ObjRef objSel_ref;
                Guid selObjId = Guid.Empty;
                var rc = RhinoGet.GetOneObject("Select a surface or polysurface as the end-effector", false, ObjectType.AnyObject, out objSel_ref);
                if (rc == Rhino.Commands.Result.Success)
                {
                    // select a brep
                    selObjId = objSel_ref.ObjectId;
                    ObjRef currObj = new ObjRef(selObjId);

                    Brep endeffector = currObj.Brep();

                    Point3d ee_cen = endeffector.GetBoundingBox(true).Center;
                    Point3d arrow1_cen = arrowCurves[0].GetBoundingBox(true).Center;
                    Point3d arrow2_cen = arrowCurves[1].GetBoundingBox(true).Center;

                    if (ee_cen.DistanceTo(arrow1_cen) <= ee_cen.DistanceTo(arrow2_cen))
                    {
                        // select arrow2
                        p = arrows[1];
                    }
                    else
                    {
                        // select arrow1
                        p = arrows[0];
                    }
                }
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
            get { return new Guid("f2dd9fd7-1d81-4ab4-8943-efd852dce153"); }
        }
    }
}