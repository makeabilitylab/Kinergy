using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Rhino;
using Rhino.Geometry;
using Rhino.DocObjects;
using Rhino.Display;
using Rhino.Input;
using System.Drawing;
using System.Diagnostics;
using Rhino.Geometry.Intersect;

namespace EnergyPlugin
{ 
    public interface RhinoModel
    {
        void AddRandomGeometry();
        void ExportModel();
        void Voxelize();
    }

    public class IncRhinoModel : RhinoModel
    {
        #region Global variances 
        ProcessingWarningWindow processingwindow = new ProcessingWarningWindow();
        #endregion

        #region declarations of local variables for all features
        RhinoDoc myDoc = null;
        ObjectAttributes redAttribute, whiteAttribute, greenAttribute, orangeAttribute, blueAttribute;
        #endregion

        public IncRhinoModel()
        {
            myDoc = PluginBarCommand.rhinoDoc;
            if (myDoc == null)
            {
                myDoc = RhinoDoc.ActiveDoc;
            }

            #region color definitions
            int redIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material redMat = myDoc.Materials[redIndex];
            redMat.DiffuseColor = System.Drawing.Color.Red;
            redMat.SpecularColor = System.Drawing.Color.Red;
            redMat.CommitChanges();
            redAttribute = new ObjectAttributes();
            redAttribute.LayerIndex = 1;
            redAttribute.MaterialIndex = redIndex;
            redAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            redAttribute.ObjectColor = Color.Red;
            redAttribute.ColorSource = ObjectColorSource.ColorFromObject;

            int whiteIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material whiteMat = myDoc.Materials[whiteIndex];
            whiteMat.DiffuseColor = System.Drawing.Color.White;
            whiteMat.SpecularColor = System.Drawing.Color.White;
            whiteMat.Transparency = 0;
            whiteMat.CommitChanges();
            whiteAttribute = new ObjectAttributes();
            whiteAttribute.LayerIndex = 2;
            whiteAttribute.MaterialIndex = whiteIndex;
            whiteAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            whiteAttribute.ObjectColor = Color.White;
            whiteAttribute.ColorSource = ObjectColorSource.ColorFromObject;

            int blueIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material blueMat = myDoc.Materials[blueIndex];
            blueMat.DiffuseColor = System.Drawing.Color.FromArgb(16,150,206);
            blueMat.SpecularColor = System.Drawing.Color.FromArgb(16, 150, 206);
            blueMat.Transparency = 0.7f;
            blueMat.TransparentColor = System.Drawing.Color.FromArgb(16, 150, 206);
            blueMat.CommitChanges();
            blueAttribute = new ObjectAttributes();
            blueAttribute.LayerIndex = 5;
            blueAttribute.MaterialIndex = blueIndex;
            blueAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            blueAttribute.ObjectColor = Color.FromArgb(16, 150, 206);
            blueAttribute.ColorSource = ObjectColorSource.ColorFromObject;

            int greenIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material greenMat = myDoc.Materials[greenIndex];
            greenMat.DiffuseColor = System.Drawing.Color.Green;
            greenMat.SpecularColor = System.Drawing.Color.Green;
            greenMat.Transparency = 0.7f;
            greenMat.CommitChanges();
            greenAttribute = new ObjectAttributes();
            greenAttribute.LayerIndex = 3;
            greenAttribute.MaterialIndex = greenIndex;
            greenAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            greenAttribute.ObjectColor = Color.Green;
            greenAttribute.ColorSource = ObjectColorSource.ColorFromObject;

            int orangeIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material orangeMat = myDoc.Materials[orangeIndex];
            orangeMat.DiffuseColor = System.Drawing.Color.Orange;
            orangeMat.Transparency = 0.3;
            orangeMat.SpecularColor = System.Drawing.Color.Orange;
            orangeMat.CommitChanges();
            orangeAttribute = new ObjectAttributes();
            orangeAttribute.LayerIndex = 4;
            orangeAttribute.MaterialIndex = orangeIndex;
            orangeAttribute.MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject;
            orangeAttribute.ObjectColor = Color.Orange;
            orangeAttribute.ColorSource = ObjectColorSource.ColorFromObject;

            #endregion

            myDoc.Views.Redraw();
        }
       
        public void AddRandomGeometry()
        {
            processingwindow.Show();
            processingwindow.Refresh();

            // Generate a radom number between 1-5
            Random rnd = new Random();
            int num = rnd.Next(1, 6);

            switch (num)
            {
                case 1:
                    {
                        // add a red small sphere
                        myDoc.Objects.AddSphere(new Sphere(new Point3d(0, 0, 0), 10), redAttribute);
                    }
                    break;
                case 2:
                    {
                        // add a white small box
                        Box b = new Box(new BoundingBox(0, 0, 0, 20, 20, 20));
                        myDoc.Objects.AddBrep(b.ToBrep(), whiteAttribute);
                    }
                    break;
                case 3:
                    {
                        // add a blue midium sphere
                        myDoc.Objects.AddSphere(new Sphere(new Point3d(0, 0, 0), 20), blueAttribute);
                    }
                    break;
                case 4:
                    {
                        // add a gree large box
                        Box b = new Box(new BoundingBox(0, 0, 0, 40, 40, 40));
                        myDoc.Objects.AddBrep(b.ToBrep(), greenAttribute);
                    }
                    break;
                case 5:
                    {
                        // add an orange large sphere
                        myDoc.Objects.AddSphere(new Sphere(new Point3d(0, 0, 0), 30), orangeAttribute);
                    }
                    break;
                default:break;
            }

            myDoc.Views.Redraw();

            processingwindow.Hide();
            processingwindow.Refresh();

        }

        public void ExportModel()
        {
            // Ask the user to select one object
            ObjRef objSel_ref;
            var rcommand = RhinoGet.GetOneObject("Select surface or polysurface to export", false, ObjectType.AnyObject, out objSel_ref);
            if (rcommand == Rhino.Commands.Result.Success)
            {
                string currentPath = System.IO.Directory.GetCurrentDirectory();
                string STLFileName = currentPath + @"\output.stl";

                Rhino.RhinoApp.RunScript("-Export " + STLFileName + " y=n Enter Enter", false);
            }
        }

        public void Voxelize()
        {
            processingwindow.Show();
            processingwindow.Refresh();

            // Ask the user to select one object
            ObjRef objSel_ref;
            var rcommand = RhinoGet.GetOneObject("Select surface or polysurface to export", false, ObjectType.AnyObject, out objSel_ref);
            if (rcommand == Rhino.Commands.Result.Success) { 
            // Find the object already selected
            //Rhino.Input.Custom.GetObject go = new Rhino.Input.Custom.GetObject();
            //go.AlreadySelectedObjectSelect = true;
            //go.GeometryFilter = Rhino.DocObjects.ObjectType.Surface | ObjectType.PolysrfFilter;
            //Rhino.DocObjects.ObjRef objref = go.Object(0);
                RhinoObject obj = objSel_ref.Object();         
                var brep = objSel_ref.Brep();
                if (brep != null)
                {
                    // Convert brep to mesh
                    var default_mesh_params = MeshingParameters.Default;
                    var meshes = Mesh.CreateFromBrep(brep, default_mesh_params);
                    if (meshes != null)
                    {
                        var brep_mesh = new Mesh();
                        foreach (var mesh in meshes)
                            brep_mesh.Append(mesh);
                        //Compute the bounding box of brep_mesh
                        var bbox = brep_mesh.GetBoundingBox(false);
                        var dMin=3; //Set the size of each voxel(Can be changed)
                        //Select ray with direction(0,0,1).
                        Vector3d ray = new Vector3d(0, 0, 1);
                        //Compute origin points in the x-y planes
                        List < Vector3d > origins= new List<Vector3d>();
                        for (var i = bbox.Min.X; i < bbox.Max.X + dMin; i += dMin)
                        {
                            for (var j = bbox.Min.Y; j < bbox.Max.Y + dMin; j += dMin)
                            {
                                origins.Add(new Vector3d(i, j, 0));
                            }
                        }
                        // Count Intersection times of different voxels.
                        int VoxelZCount = (int)(bbox.Max.Z - bbox.Min.Z) / dMin + 1;
                        int[,] voxelArrays= new int[origins.Count(), VoxelZCount];
                        for(int i=0;i< origins.Count(); i++)
                        {
                            for(int j=0;j < VoxelZCount; j++)
                            {
                                voxelArrays[i,j] = 0;
                            }
                        }
                        // Triangulate the mesh
                        brep_mesh.Faces.ConvertQuadsToTriangles();
                        foreach (var triangle in brep_mesh.Faces)
                        {
                            // Get Vertice Coordinates
                            var a = brep_mesh.Vertices.Point3dAt(triangle.A);
                            var b = brep_mesh.Vertices.Point3dAt(triangle.B);
                            var c = brep_mesh.Vertices.Point3dAt(triangle.C);
                            var pa = new Vector3d(a);
                            var pb = new Vector3d(b);
                            var pc = new Vector3d(c);
                            // Compute Normals and d
                            var normal = Vector3d.CrossProduct((pb - pa), (pc - pa));
                            var n = Vector3d.Divide(normal, normal.Length);
                            var d = Vector3d.Multiply(n, pa);
                            //Compute the intersections of rays and triangles
                            var nd = Vector3d.Multiply(n, ray);
                            if (nd != 0) { 
                                foreach (var origin in origins)                                         
                                {
                                    var t = (d - Vector3d.Multiply(n, origin)) / nd;
                                    Vector3d pq = new Vector3d(origin.X, origin.Y, t);
                                    if (Vector3d.Multiply(Vector3d.CrossProduct((pb - pa), (pq - pa)), n) >= 0)
                                    {
                                        if (Vector3d.Multiply(Vector3d.CrossProduct((pc - pb), (pq - pb)), n) >= 0)
                                        {
                                            if (Vector3d.Multiply(Vector3d.CrossProduct((pa - pc), (pq - pc)), n) >= 0)
                                            {
                                                //Q is inside triangle ABC
                                                int intersect_z = (int)((pq.Z - bbox.Min.Z)/dMin)+1;                                               
                                                for (int i= intersect_z; i < VoxelZCount; i++)
                                                {
                                                    voxelArrays[origins.IndexOf(origin), i]++;
                                                }
                                                
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        //Voxelize the model
                        for (int i = 0; i < origins.Count(); i++)
                        {
                            for (int j = 0; j < VoxelZCount; j++)
                            {
                                //If the interaction times before ray reaches voxel is odds
                                if(voxelArrays[i, j] % 2 != 0)
                                {
                                    var x = origins[i].X;
                                    var y = origins[i].Y;
                                    var z = bbox.Min.Z + j * dMin;
                                    Box b = new Box(new BoundingBox(x,y,z,x+dMin,y+dMin,z+dMin));
                                    myDoc.Objects.AddBrep(b.ToBrep(), whiteAttribute);
                                }
                            }
                        }                   
    
                    }
                }
                    
            }
            myDoc.Views.Redraw();

            processingwindow.Hide();
            processingwindow.Refresh();

        }
    }
}