using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Kinergy.KineticUnit;
using Kinergy.Utilities;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace InstTranslation
{
    public class InstantTranslationComponent : GH_Component
    {
        bool toStart;
        Brep model;
        Brep oriModel;
        Vector3d direction;
        double t1, t2;
        List<Arrow> lockDirCandidates;
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public InstantTranslationComponent()
          : base("InstantTranslation", "IT",
              "Solve instant translation motion",
              "Kinergy", "InstantTranslation")
        {
            lockDirCandidates = null;
            toStart = false;
            model = null;
            oriModel = null;
            direction = Vector3d.Unset;
            t1 = 0;
            t2 = 1;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Start", "S", "Whether to start calculating", GH_ParamAccess.item);
            pManager.AddBrepParameter("Model", "M", "Model to process", GH_ParamAccess.item);
            pManager.AddVectorParameter("MotionDirection", "MD", "Direction of Motion", GH_ParamAccess.item);
            pManager.AddBooleanParameter("CurvedModel", "CM", "If the model should be calculated as curved shape", GH_ParamAccess.item);
            pManager.AddNumberParameter("Energy", "E", "Energy of motion", GH_ParamAccess.item);
            pManager.AddNumberParameter("Distance", "D", "Proportion of spring that's able to be compressed", GH_ParamAccess.item);
            pManager.AddNumberParameter("SkeletonStartPos", "SPos", "the start of the selected segment", GH_ParamAccess.item);
            pManager.AddNumberParameter("SkeletonEndPos", "EPos", "the end of the selected segment", GH_ParamAccess.item);
            pManager.AddBrepParameter("OriginalBrep", "OriB", "original brep", GH_ParamAccess.item);
            pManager.AddCurveParameter("Ske", "Skeleton", "the skeleton of the original body", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("InstantTranslationKineticUnit", "KU", "Motion instance generated in motion solver.", GH_ParamAccess.item);
            pManager.AddGenericParameter("LockDirectionCandidates", "DC", "Available directions of lock as arrows", GH_ParamAccess.list);
            pManager.AddBooleanParameter("End-EffectorReady", "EN", "enable the end-effector battery", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool EE_trigger = false;
            DA.SetData(2, EE_trigger);
            double min_wire_diamter = 2.8;
            double min_coil_num = 3;

           
            bool start = false;
            double energyLevel=0;
            double displacementLevel=0;
            double energy = 0;
            double displacement = 0;
            bool curved = false;
            Curve skt = null;
            if (!DA.GetData(0, ref start)) { return; }
            if (start == false) { }
            else { toStart = true; }
            if (!DA.GetData(2, ref direction)) { return; }
            if (!DA.GetData(3, ref curved)) { return; }
            if (!DA.GetData(4, ref energyLevel)) { return; }
            if (!DA.GetData(5, ref displacementLevel)) { return; }
            if (!DA.GetData(1, ref model)) { return; }
            if (!DA.GetData(6, ref t1)) { return; }
            if (!DA.GetData(7, ref t2)) { return; }
            if (!DA.GetData(8, ref oriModel)) { return; }
            if (!DA.GetData(9, ref skt)) { return; }

            if (toStart)
            {
                #region Step 1: Create an instance of InstantTranslation class

                #region Parse energy and displacement

                // Parse the dispalcement (in percentage) based on the spring length and the posible max compression dispacement
                Point3d ptS = skt.PointAtNormalizedLength(t1);
                Point3d ptE = skt.PointAtNormalizedLength(t2);
                double s_len = ptS.DistanceTo(ptE);
                double maxDisp = Math.Max(s_len - min_wire_diamter * min_coil_num, min_coil_num * 0.6);
                displacement = (displacementLevel + 1) / 10 * maxDisp / s_len;     // convert the input displacement level into percentage

                // Parse the energy based on E ~= d^4/n * x^2
                double x = displacement * s_len;
                energy = (energyLevel + 1) / 10;

                #endregion
                InstantTranslation motion = new InstantTranslation(oriModel, curved, direction, energy, displacement);
                #endregion

                #region Step 2: Find the position of the helical spring
                if (curved)
                {
                    motion.CalculateCurvedSkeleton();
                }
                else
                {
                    motion.CalculateStraightSkeleton(t1, t2, model);
                }

                // List<Point3d> pts = motion.GetSpringPositionCandidates();
                //double sp_X = 0, sp_Y = 0, sp_Z = 0;
                //foreach(Point3d pt in pts)
                //{
                //    sp_X += sp_X + pt.X;
                //    sp_Y += sp_Y + pt.Y;
                //    sp_Z += sp_Z + pt.Z;
                //}

                //sp_X = sp_X / pts.Count;
                //sp_Y = sp_Y / pts.Count;
                //sp_Z = sp_Z / pts.Count;

                //Point3d midPt = new Point3d(sp_X, sp_Y, sp_Z);
                //double dis = 1000000000000;
                //foreach (Point3d pt in pts)
                //{
                //    if (pt.DistanceTo(midPt) <= dis)
                //    {
                //        dis = pt.DistanceTo(midPt);
                //        sp_X = pt.X;
                //        sp_Y = pt.Y;
                //        sp_Z = pt.Z;
                //    }

                //}

                Point3d springPos = motion.Skeleton.PointAtNormalizedLength((t1 + t2) / 2);
                #endregion
                //Rhino.RhinoDoc myDoc = Rhino.RhinoDoc.ActiveDoc;
                //myDoc.Objects.AddCurve(motion.Skeleton);
                //myDoc.Views.Redraw();

                #region Step 3: Construct the spring based on the input energy and distance
                motion.SetSpringPosition(springPos);
                motion.CutModelForSpring();
                motion.ConstructSpring();

                // Generate the arrows but reserved for the end-effector step to confirm the lock position

                lockDirCandidates = motion.GetLockDirectionCandidates();
                #endregion

                DA.SetData(0, motion);
                DA.SetDataList(1, lockDirCandidates);
                EE_trigger = true;
                DA.SetData(2, EE_trigger);
            }
            else
            {
                DA.SetData(0, null);
                DA.SetDataList(1, null);
                EE_trigger = false;
                DA.SetData(2, EE_trigger);
            }
            
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
            get { return new Guid("da709922-b416-496f-b555-4a735eb0fe36"); }
        }
    }
}
