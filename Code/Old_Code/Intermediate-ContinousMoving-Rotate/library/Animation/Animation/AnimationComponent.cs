using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace Animation
{
    public class AnimationComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public AnimationComponent()
          : base("Animation", "Animation",
              "animate movement",
              "brep", "kinetic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            //inputs for motion compute
            pManager.AddGenericParameter("double", "v0", "initial linear velocity", GH_ParamAccess.item);
            pManager.AddGenericParameter("double", "a", "accelaration", GH_ParamAccess.item);
            pManager.AddGenericParameter("double", "t", "time", GH_ParamAccess.item);
            //input for different breps to move
            pManager.AddBrepParameter("Brep", "gears", "gears", GH_ParamAccess.list);
            pManager.AddBrepParameter("Brep", "rotateAxis", "object with axis to rotate", GH_ParamAccess.item);
            pManager.AddBrepParameter("Brep", "rotateGear", "object with main gear to rotate", GH_ParamAccess.list);
            pManager.AddBrepParameter("Brep", "move", "objects to move forward", GH_ParamAccess.list);
            pManager.AddGenericParameter("double", "w", "angular velocity", GH_ParamAccess.list);
            pManager.AddVectorParameter("Vector3d", "direct", "direction to move forward", GH_ParamAccess.item);
            pManager.AddGenericParameter("double", "maxRadius", "max radius of axis", GH_ParamAccess.item);
            pManager.AddVectorParameter("Vector3d", "gearNormal", "gear normal", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBrepParameter("Brep", "animate", "breps in transform", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double v0=0;
            double a=0;
            double t=0;
            List<Brep> gears = new List<Brep>();
            Brep rotateAxis = null;
            List<Brep> rotateGear = new List<Brep>();
            List<Brep> move = new List<Brep>();
            List<double> w = new List<double>();
            Vector3d direct = Vector3d.Unset;
            double maxRadius = 0;
            Vector3d gearNormal = Vector3d.Unset;
            if(!DA.GetData(0,ref v0)) { return; }
            if(!DA.GetData(1,ref a)) { return; }
            if(!DA.GetData(2,ref t)) { return; }
            if (!DA.GetDataList(3, gears)) { return; }
            if (!DA.GetData(4, ref rotateAxis)) { return; }
            if(!DA.GetDataList(5, rotateGear)) { return; }
            if (!DA.GetDataList(6, move)) { return; }
            if (!DA.GetDataList(7, w)) { return; }
            if(!DA.GetData(8,ref direct)) { return; }
            if(!DA.GetData(9,ref maxRadius)) { return; }
            if(!DA.GetData(10,ref gearNormal)) { return; }
            //rotate between model and X
            var xrotate = Transform.Rotation(direct, Vector3d.XAxis, Point3d.Origin);
            var xrotateBack = Transform.Rotation(Vector3d.XAxis, direct, Point3d.Origin);
            //output
            List<Brep> animate = new List<Brep>();
            double s;
            double w0;
            v0 = Math.Abs(v0);
            //Step 1: Rotate knob
            double t0 = 3.5;
            if (t < t0)
            {
                double wrotate = Math.PI/2;
                rotateGear[0].Transform(xrotate);
                Point3d rotateCenter = rotateGear[0].GetBoundingBox(true).Center;
                rotateGear[0].Transform(xrotateBack);
                rotateCenter.Transform(xrotateBack);
                var xr = Transform.Rotation(wrotate * t0, gearNormal, rotateCenter);
                foreach (Brep brep in rotateGear)
                {
                    brep.Transform(xr);
                }
            }
            //Step 2: the car moves forward           
            else if (t < v0 / a + t0)
            {
                double w00 = w[w.Count - 1];
                t -= t0;
                s = v0 * t - a * t * t / 2;
                w0 = s / maxRadius;
                var xf = Transform.Translation(direct / direct.Length * s);
                for (int i = 0; i < gears.Count; i++)
                {
                    Brep gear = gears[i];
                    Point3d Center = gear.GetBoundingBox(true).Center;
                    var xr = Transform.Rotation(w[i] / w00 * w0, gearNormal, Center);
                    if (i == 0)
                    {
                        foreach (Brep brep in rotateGear)
                        {
                            brep.Transform(xr);
                            //brep.Transform(xrotate);
                            brep.Transform(xf);
                            //brep.Transform(xrotateBack);
                            animate.Add(brep);
                        }
                    }
                    else if (i == gears.Count - 1)
                    {
                        rotateAxis.Transform(xr);
                        //rotateAxis.Transform(xrotate);
                        rotateAxis.Transform(xf);
                        //rotateAxis.Transform(xrotateBack);
                        animate.Add(rotateAxis);
                    }
                    gear.Transform(xf);
                    //gear.Transform(xrotate);                
                    //gear.Transform(xr);
                    //gear.Transform(xrotateBack);
                    animate.Add(gear);
                }
                foreach (Brep brep in move)
                {
                    brep.Transform(xf);
                    animate.Add(brep);
                }
            }
            //Step3: stop
            else
            {
                s = v0 * v0 / a / 2;
                var xf = Transform.Translation(direct / direct.Length * s);
                foreach (Brep gear in gears)
                {
                    gear.Transform(xf);
                    animate.Add(gear);
                }
                foreach (Brep brep in rotateGear)
                {
                    brep.Transform(xf);
                    animate.Add(brep);
                }
                rotateAxis.Transform(xf);
                animate.Add(rotateAxis);
                foreach (Brep brep in move)
                {
                    brep.Transform(xf);
                    animate.Add(brep);
                }
            }
            DA.SetDataList(0, animate);
            return;
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
            get { return new Guid("ab4761a8-df77-4850-adf6-dd597db84de8"); }
        }
    }
}
