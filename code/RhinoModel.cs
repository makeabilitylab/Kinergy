using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Rhino;
using Rhino.Geometry;
using Rhino.DocObjects;
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
    }

}