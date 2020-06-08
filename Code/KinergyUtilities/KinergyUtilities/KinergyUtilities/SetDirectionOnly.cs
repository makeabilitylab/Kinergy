using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;

namespace KinergyUtilities
{
    public class SetDirectionOnly : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public SetDirectionOnly()
          : base("SetDirectionOnly", "SetDirOnly",
              "Set only the direction in Rhino",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Select", "S", "Listen to a click on a button", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("DirVector", "DV", "The vector indicates the direction", GH_ParamAccess.item);
            pManager.AddCurveParameter("ArrowCrv", "AC", "The arrow is added if the direction is set", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Point3d pt1 = new Point3d(0, 0, 0);
            Point3d pt2 = new Point3d(20, 0, 0);
            Point3d pt3 = new Point3d(20, 4, 0);
            Point3d pt4 = new Point3d(28, 0, 0);
            Point3d pt5 = new Point3d(20, -4, 0);
            Point3d pt6 = new Point3d(20, 0, 0);
            List<Point3d> pts = new List<Point3d>();
            pts.Add(pt1); pts.Add(pt2); pts.Add(pt3); pts.Add(pt4); pts.Add(pt5); pts.Add(pt6);

            Polyline polyln = new Polyline(pts);
            Curve arrowcrv = polyln.ToNurbsCurve();

            Vector3d dirVec = new Vector3d();
            bool isCrv = false;
            Curve inputArrow = arrowcrv;

            // Set the arrow and direction vector
            if (!DA.GetData<bool>(0, ref isCrv))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "fuck you");
                return;
            }
            if (isCrv == false)
            {
                return;
            }

            // Set the arrow and direction vector
            Rhino.RhinoDoc myDoc = Rhino.RhinoDoc.ActiveDoc;

            // Select the first point in Rhino
            Rhino.Input.Custom.GetPoint gp = new Rhino.Input.Custom.GetPoint();
            gp.SetCommandPrompt("Set the first point for the direction");
            gp.Get();
            Point3d pt_start = gp.Point();
            myDoc.Objects.AddPoint(pt_start);

            // Select the second point in Rhino to form the direction
            gp.SetCommandPrompt("Set the second point for the direction");
            gp.SetBasePoint(pt_start, false);
            gp.DrawLineFromPoint(pt_start, true);
            gp.Get();
            Point3d pt_end = gp.Point();
            myDoc.Objects.AddPoint(pt_end);

            dirVec = pt_end - pt_start;
            inputArrow.Transform(Transform.Translation(new Vector3d(pt_start)));
            inputArrow.Transform(Transform.Rotation(Vector3d.XAxis, dirVec, pt_start));
            myDoc.Objects.AddCurve(inputArrow);
            DA.SetData(0, dirVec);
            DA.SetData(1, inputArrow);
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
            get { return new Guid("9a8fe5bb-ae8c-42e1-9f9c-cefbd45efa21"); }
        }
    }
}