using System;
using System.Collections.Generic;
using Rhino;
using Rhino.DocObjects;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Rhino.Geometry;
using System.Diagnostics;
namespace KinergyUtilities
{
    public class CalculateSkeletonOfBrep : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CalculateSkeletonOfBrep class.
        /// </summary>
        public CalculateSkeletonOfBrep()
          : base("CalculateSkeletonOfBrep", "Nickname",
              "Description",
              "Kinergy", "Utilities")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBrepParameter("Mesh", "M", "The mesh to be calculated", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Skeleton", "S", "The calculated skeleton curve", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Brep m = null;
            if (!DA.GetData(0, ref m)) return;

            // Limitations: if there are sharp cornners existing on the geometry, the generated medial axis is not accurate. 
            //              In other word, we should fillet the edges of the model if possible.
            // Convert all objects in Rhino to mesh and save as stl files in the current directory

            string dir = @"C:\KinergyTest\";
            //string dir = "\\Mac/Home/Desktop/kinetic_tool/Kinergy-master/Kinergy/Code/MotionSolver/InstantExtension/InstantExtension/bin";
            #region Bake and export the brep model as stl file
            Guid mybrepguid = Guid.Empty;
            string layername = "new_layer";
            // layer to bake the objects to
            InstExtension.Utilities.create_layer(layername);
            //create a directory to store the stl files
            InstExtension.Utilities.create_dir(dir);
            var doc = Rhino.RhinoDoc.ActiveDoc;

            //declare the objects attributes
            ObjectAttributes attr = new Rhino.DocObjects.ObjectAttributes();
            //set attributes
            var l = doc.Layers.FindName(layername);
            attr.LayerIndex = l.Index;

            Guid id = Guid.Empty;

            //bake the mesh

            id = doc.Objects.AddBrep(m, attr);

            // add the mesh to the guid
            if (id.ToString().Length > 0) mybrepguid = id;

            // select the breps in Rhino to successfully export them
            doc.Objects.Select(mybrepguid, true);
            //where to save
            string oldSTLFile = dir + @"/temp_stl.stl";
            if (File.Exists(oldSTLFile)) File.Delete(oldSTLFile);
            //and export them
            Rhino.RhinoApp.RunScript("-_Export\n\"" + oldSTLFile + "\"\n _Enter\n _Enter", true);

            //delete the breps after exporting them
            doc.Objects.Delete(mybrepguid, true);
            /*ObjRef objSel_ref;
            Guid sufObjId = Guid.Empty;
            var rc = RhinoGet.GetOneObject("Select surface or polysurface to mesh", false, ObjectType.Brep, out objSel_ref);
            if (rc == Rhino.Commands.Result.Success)
            {
                String str1 = "_ExportFileAs=_Binary ";
                String str2 = "_ExportUnfinishedObjects=_Yes ";
                String str3 = "_UseSimpleDialog=_No ";
                String str4 = "_UseSimpleParameters=_Yes ";

                String str5 = "_Enter _DetailedOptions ";
                String str6 = "_JaggedSeams=_No ";
                String str7 = "_PackTextures=_No ";
                String str8 = "_Refine=_Yes ";
                String str9 = "_SimplePlane=_Yes ";
                String str10 = "_Weld=_No ";
                String str11 = "_AdvancedOptions ";
                String str12 = "_Angle=15 ";
                String str13 = "_AspectRatio=0 ";
                String str14 = "_Distance=0.01 ";
                String str15 = "_Grid=16 ";
                String str16 = "_MaxEdgeLength=0 ";
                String str17 = "_MinEdgeLength=0.0001 ";
                String str18 = "_Enter _Enter";

                String str = str1 + str2 + str3 + str4 + str18;
                //String str = str1 + str18;
                //String str = str1 + str2 + str3 + str4 + str5 + str6 + str7 + str8 + str9 + str10 + str11 + str12 +
                //str13 + str14 + str15 + str16 + str17 + str18;
                //String str = str18;

                var stlScript = string.Format("-_Export "+oldSTLFile+str);// _ - Export \\\Mac / Home / Desktop / kinetic tool / Kinergy - master / Kinergy / Code / MotionSolver / InstantExtension / InstantExtension / bin / temp_stl.stl
                success=RhinoApp.RunScript(stlScript, true);
                model = objSel_ref.Brep();
            }
            else
            { return false; }*/
            #endregion

            List<Curve> cvs = new List<Curve>();
            Curve joined = null;

            // clean old files
            string oldFile1 = dir + @"/temp_off_skeleton.txt";
            string oldFile2 = dir + @"/temp_off.off";
            string oldFile3 = dir + @"/temp_off_convert.off";
            string oldFile4 = dir + @"/temp_off_skeleton.off";

            if (File.Exists(oldFile1)) File.Delete(oldFile1);
            if (File.Exists(oldFile2)) File.Delete(oldFile2);
            if (File.Exists(oldFile3)) File.Delete(oldFile3);
            if (File.Exists(oldFile4)) File.Delete(oldFile4);

            //var brep_mesh = Mesh.CreateFromBrep(model, MeshingParameters.FastRenderMesh)[0];

            #region Using meshlab server to convert the mesh into off file
            Process meshCompiler = new Process();
            ProcessStartInfo meshStartInfo = new ProcessStartInfo();
            meshStartInfo.CreateNoWindow = true;
            meshStartInfo.UseShellExecute = false;

            meshStartInfo.FileName = @"meshlabserver/meshlabserver.exe";

            // Note: unifying duplicated vertices is necessary
            meshStartInfo.Arguments = @" -i " + dir + @"/temp_stl.stl -o " + dir + @"/temp_off.off -s " + @"meshlabserver/clean.mlx";

            meshCompiler.StartInfo = meshStartInfo;
            meshCompiler.Start();
            meshCompiler.WaitForExit();
            #endregion

            #region call the medial axis generation cmd
            Process matCompiler = new Process();
            ProcessStartInfo startInfo = new ProcessStartInfo();
            startInfo.CreateNoWindow = true;
            //startInfo.CreateNoWindow = false;
            startInfo.UseShellExecute = false;
            startInfo.FileName = @"skeletonization/skeletonization.exe";

            startInfo.Arguments = dir + @"/temp_off.off --debug";

            matCompiler.StartInfo = startInfo;
            matCompiler.Start();
            matCompiler.WaitForExit();
            //Process.Start(startInfo);


            string curFile = dir + @"/temp_off_skeleton.txt";
            int ctrlPtNum = 0;
            //System.Threading.Thread.Sleep(10000);
            List<Point3d> maPoints = new List<Point3d>();
            string line;

            //Pass the file path and file name to the StreamReader constructor
            StreamReader sr = new StreamReader(curFile);

            //Read the first line of text
            line = sr.ReadLine();
            maPoints.Clear();

            do
            {
                // if there is only one number skip this line,
                // otherwise store those points
                string[] dots = line.Split('\t');
                if (dots.Length == 1 && maPoints.Count != 0)
                {

                    //foreach (Point3d p in maPoints)
                    //{
                    //    myDoc.Objects.AddPoint(p);
                    //}

                    Curve ma = Rhino.Geometry.Curve.CreateControlPointCurve(maPoints, 9);
                    cvs.Add(ma);
                    maPoints.Clear();
                }
                else if (dots.Length == 3)
                {
                    Point3d tempPt = new Point3d();
                    tempPt.X = Convert.ToDouble(dots[0]);
                    tempPt.Y = Convert.ToDouble(dots[1]);
                    tempPt.Z = Convert.ToDouble(dots[2]);
                    maPoints.Add(tempPt);
                    ctrlPtNum++;
                }

                line = sr.ReadLine();
            } while (line != null);
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            if (maPoints.Count != 0)
            {

                //foreach (Point3d p in maPoints)
                //{
                //    myDoc.Objects.AddPoint(p);
                //}

                Curve ma = Curve.CreateControlPointCurve(maPoints, 9);
                cvs.Add(ma);
                joined = Curve.JoinCurves(cvs)[0];

            }
            //close the file
            sr.Close();
            Curve c = joined;
            #endregion
            DA.SetData(0, c);
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
            get { return new Guid("b5f0ec5a-e619-4a65-8816-ab99f0fd0bfd"); }
        }
    }
}