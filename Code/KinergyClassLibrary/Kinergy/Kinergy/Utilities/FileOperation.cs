using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel.Components;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.DocObjects;
using Rhino.Collections;
using Rhino.Input.Custom;
using Rhino;
using Kinergy.Geom;

namespace Kinergy.Utilities
{
    /// <summary>
    /// This namespace contains convenient file operations
    /// </summary>
    
        
        public class FileOperation
        {
            public static String FindComponentFolderDirectory()
            {//TODO This directory isn't right!
                string dir = Environment.CurrentDirectory;
                System.IO.DirectoryInfo pathInfo = new System.IO.DirectoryInfo(dir);
                string newPath = pathInfo.Parent.FullName;
                return newPath + "\\Plug-ins\\Grasshopper\\Components";

            }
            public static String FindCurrentFolderResourceDirectory()
            {
            string path = System.IO.Directory.GetCurrentDirectory();
            System.IO.DirectoryInfo pathInfo = new System.IO.DirectoryInfo(path);
            string newPath = pathInfo.Parent.FullName;
            
            //Brep Lock_head = FileOperation.SingleBrepFromResourceFile(newPath+"\\Resources\\lockHead.3dm");
            return newPath + "\\Resources";

            }
            /// <summary>
            /// This method returns the single brep from file
            /// </summary>
            public static Brep SingleBrepFromResourceFile(string fileDirectory)
            {
                Rhino.FileIO.File3dm file = Rhino.FileIO.File3dm.Read(fileDirectory);
                List<Brep> bps = new List<Brep>();
                foreach (Rhino.FileIO.File3dmObject obj in file.Objects)
                {

                    Brep brep = null;
                    if (GH_Convert.ToBrep(obj.Geometry, ref brep, GH_Conversion.Both))
                    {
                        bps.Add(brep.DuplicateBrep());
                    }
                }
                file.Dispose();
            
                return bps[0];
            }
            /// <summary>
            /// This method returns multiple breps from file as a list
            /// </summary>
            public static List<Brep> BrepsFromResourceFile(string fileDirectory)
            {
                Rhino.FileIO.File3dm file = Rhino.FileIO.File3dm.Read(fileDirectory);
                List<Brep> bps = new List<Brep>();
                foreach (Rhino.FileIO.File3dmObject obj in file.Objects)
                {

                    Brep brep = null;
                    if (GH_Convert.ToBrep(obj.Geometry, ref brep, GH_Conversion.Both))
                    {
                        bps.Add(brep.DuplicateBrep());
                    }
                }
                file.Dispose();
                return bps;
            }
        }
    
}
