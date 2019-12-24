using System;
using System.Linq;
using Rhino;
using Rhino.Geometry;
using Rhino.DocObjects;
using Rhino.Input;
using System.Drawing;
using Rhino.Input.Custom;

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
        readonly ObjectAttributes redAttribute, whiteAttribute, greenAttribute, orangeAttribute, blueAttribute;
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
            redAttribute = new ObjectAttributes
            {
                LayerIndex = 1,
                MaterialIndex = redIndex,
                MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject,
                ObjectColor = Color.Red,
                ColorSource = ObjectColorSource.ColorFromObject
            };

            int whiteIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material whiteMat = myDoc.Materials[whiteIndex];
            whiteMat.DiffuseColor = System.Drawing.Color.White;
            whiteMat.SpecularColor = System.Drawing.Color.White;
            whiteMat.Transparency = 0;
            whiteMat.CommitChanges();
            whiteAttribute = new ObjectAttributes
            {
                LayerIndex = 2,
                MaterialIndex = whiteIndex,
                MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject,
                ObjectColor = Color.White,
                ColorSource = ObjectColorSource.ColorFromObject
            };

            int blueIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material blueMat = myDoc.Materials[blueIndex];
            blueMat.DiffuseColor = System.Drawing.Color.FromArgb(16, 150, 206);
            blueMat.SpecularColor = System.Drawing.Color.FromArgb(16, 150, 206);
            blueMat.Transparency = 0.7f;
            blueMat.TransparentColor = System.Drawing.Color.FromArgb(16, 150, 206);
            blueMat.CommitChanges();
            blueAttribute = new ObjectAttributes
            {
                LayerIndex = 5,
                MaterialIndex = blueIndex,
                MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject,
                ObjectColor = Color.FromArgb(16, 150, 206),
                ColorSource = ObjectColorSource.ColorFromObject
            };

            int greenIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material greenMat = myDoc.Materials[greenIndex];
            greenMat.DiffuseColor = System.Drawing.Color.Green;
            greenMat.SpecularColor = System.Drawing.Color.Green;
            greenMat.Transparency = 0.7f;
            greenMat.CommitChanges();
            greenAttribute = new ObjectAttributes
            {
                LayerIndex = 3,
                MaterialIndex = greenIndex,
                MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject,
                ObjectColor = Color.Green,
                ColorSource = ObjectColorSource.ColorFromObject
            };

            int orangeIndex = myDoc.Materials.Add();
            Rhino.DocObjects.Material orangeMat = myDoc.Materials[orangeIndex];
            orangeMat.DiffuseColor = System.Drawing.Color.Orange;
            orangeMat.Transparency = 0.3;
            orangeMat.SpecularColor = System.Drawing.Color.Orange;
            orangeMat.CommitChanges();
            orangeAttribute = new ObjectAttributes
            {
                LayerIndex = 4,
                MaterialIndex = orangeIndex,
                MaterialSource = Rhino.DocObjects.ObjectMaterialSource.MaterialFromObject,
                ObjectColor = Color.Orange,
                ColorSource = ObjectColorSource.ColorFromObject
            };

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
                default: break;
            }

            myDoc.Views.Redraw();

            processingwindow.Hide();
            processingwindow.Refresh();

        }

        public void ExportModel()
        {
            // Ask the user to select one object
            var rcommand = RhinoGet.GetOneObject("Select surface or polysurface to export", false, ObjectType.AnyObject, out ObjRef objSel_ref);
            if (rcommand == Rhino.Commands.Result.Success)
            {
                string currentPath = System.IO.Directory.GetCurrentDirectory();
                string STLFileName = currentPath + @"\output.stl";

                Rhino.RhinoApp.RunScript("-Export " + STLFileName + " y=n Enter Enter", false);
            }
        }

        public void Voxelize()
        {
            const int side = 5;

            for (int x = -side; x < side; x++)
            {
                for (int y = -side; y < side; y++)
                {
                    for (int z = -side; z < side; z++)
                    {
                        var center = new Point3d(x, y, z);
                        var voxel = new Sphere(center, 0.5);
                        var radius = IsVoxelOnSurface(voxel) ? 0.5 : 0.2;
                        myDoc.Objects.AddSphere(new Sphere(center, radius));
                    }
                }
            }

            myDoc.Views.Redraw();
        }

        private bool IsVoxelOnSurface(Sphere voxel)
        {
            var model = GetSelectedModel();

            var meshes = Mesh.CreateFromBrep(model, MeshingParameters.Default);
            foreach (var mesh in meshes)
            {
                mesh.Faces.ConvertQuadsToTriangles();
            }

            return meshes.Any(mesh => IsVoxelOnSurface(voxel, mesh));
        }

        private Brep GetSelectedModel()
        {
            var go = new GetObject
            {
                GeometryFilter = ObjectType.Brep
            };
            go.EnablePreSelect(true, true);
            go.EnablePostSelect(false);
            go.Get();

            if (go.ObjectCount > 0)
            {
                return go.Object(0).Brep();
            }
            else
            {
                return null;
            }
        }

        private bool IsVoxelOnSurface(Sphere voxel, Mesh mesh)
        {
            for (int i = 0; i < mesh.Faces.Count; i++)
            {
                mesh.Faces.GetFaceVertices(i, out var a, out var b, out var c, out var d);
                if (IsVoxelOnSurface(voxel, a, b, c))
                {
                    return true;
                }
            }

            return false;
        }

        #region Face Intersection
        private bool IsVoxelOnSurface(Sphere voxel, Point3d a, Point3d b, Point3d c)
        {
            return IsVoxelOnSurface(voxel, new Vector3d(1, 0, 0), a, b, c)
                || IsVoxelOnSurface(voxel, new Vector3d(0, 1, 0), a, b, c)
                || IsVoxelOnSurface(voxel, new Vector3d(0, 0, 1), a, b, c);
        }

        private bool IsVoxelOnSurface(Sphere voxel, Vector3d direction, Point3d a, Point3d b, Point3d c)
        {
            var ray = new Ray3d(voxel.Center, direction);
            var plane = SupportingPlane(a, b, c);

            if (Vector3d.CrossProduct(plane, ray.Direction).IsTiny())
            {
                var q = PointOfIntersection(ray, plane, a);
                return FaceContainsPoint(a, b, c, plane, q) && IsPointWithinVoxel(voxel, q);
            }
            else
            {
                return false;
            }
        }

        private Vector3d SupportingPlane(Point3d a, Point3d b, Point3d c) => Vector3d.CrossProduct(b - a, c - a);

        private Point3d PointOfIntersection(Ray3d ray, Vector3d plane, Point3d a)
        {
            var d = plane * (Vector3d)a;
            var t = (d - (plane * (Vector3d)ray.Position)) / (plane * ray.Direction);
            return ray.Position + (t * ray.Direction);
        }

        private bool FaceContainsPoint(Point3d a, Point3d b, Point3d c, Vector3d plane, Point3d point) => Vector3d.CrossProduct(b - a, point - a) * plane >= 0
            && Vector3d.CrossProduct(c - b, point - b) * plane >= 0
            && Vector3d.CrossProduct(a - c, point - c) * plane >= 0;

        private bool IsPointWithinVoxel(Sphere voxel, Point3d point) => (voxel.Center - point).SquareLength <= (voxel.Radius * voxel.Radius);
        #endregion
    }
}