using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino;
using Rhino.DocObjects;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace Simulation
{
    public class SimulationComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public SimulationComponent()
          : base("Simulation", "Simulation",
              "simulation of pull_back car",
              "brep", "kinetic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("double", "w0", "initial angular velocity of the axis", GH_ParamAccess.item);
            pManager.AddBrepParameter("Brep", "axis", "axis to rotate", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("double", "v0", "initial linear velocity of the model", GH_ParamAccess.item);
            pManager.AddGenericParameter("double", "a", "accelaration of the movement", GH_ParamAccess.item);
            pManager.AddBrepParameter("Brep", "rotate", "objects to rotate in the model", GH_ParamAccess.item);
            pManager.AddBrepParameter("Brep", "move", "objects to move forward in the model", GH_ParamAccess.list);
            pManager.AddGenericParameter("double", "radius", "max radius of rotate object", GH_ParamAccess.item);
        }
        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double angleVelocity = 0;
            Brep axis = null;
            if(!DA.GetData(0,ref angleVelocity)) { return; }
            if (!DA.GetData(1, ref axis)) { return; }
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            List<Brep> move = new List<Brep>();
            Brep rotate = axis;
            //double volume = 0;
            //g/mm3
            //double density = 1210;
            //smooth wood
            double k = 0.1;
            double g = 9.8;
            //Transform objects in docs into breps in grasshopper
            foreach (RhinoObject obj in myDoc.Objects)
            {
                ObjRef obj_ref=new ObjRef(obj);
                Brep brep = obj_ref.Brep();
                if(brep!=null)
                {                  
                    //intersect curves with axis
                    Curve[] curves;
                    Point3d[] points;
                    Rhino.Geometry.Intersect.Intersection.BrepBrep(brep, rotate, RhinoMath.SqrtEpsilon, out curves, out points);
                    if(curves.Length!=0)
                    {
                        Brep[] merge = { rotate, brep };
                        rotate = Brep.CreateBooleanUnion(merge, RhinoMath.SqrtEpsilon)[0];
                    }
                    else
                        move.Add(brep);
                }
            }
            //compute a
            double a = k * g;
            //compute initial linear velocity
            BoundingBox rotateBox = rotate.GetBoundingBox(true);
            double maxRadius = (rotateBox.Max.Z - rotateBox.Min.Z)/2;
            double initialVelocity = angleVelocity * maxRadius;
            DA.SetData(0, initialVelocity);
            DA.SetData(1, a);
            DA.SetData(2, rotate);
            DA.SetDataList(3, move);
            DA.SetData(4, maxRadius);
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("7dcbc886-93c3-42c9-b50f-685d2f441824"); }
        }
    }
}
