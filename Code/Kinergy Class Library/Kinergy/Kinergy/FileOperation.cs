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

namespace Kinergy
{
    /// <summary>
    /// This namespace contains convenient file operations
    /// </summary>
    
        
        public class FileOperation
        {
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
