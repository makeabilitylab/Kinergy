using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.DocObjects;
using System.IO;
namespace InstTranslation
{
    public class Utilities
    {
        public static void create_layer(string l)
        {
            var doc = Rhino.RhinoDoc.ActiveDoc;
            var layerslist = doc.Layers.FindName(l);
            if (layerslist==null)
            {
                Rhino.DocObjects.Layer l0 = new Rhino.DocObjects.Layer();
                l0.Name = l;
                doc.Layers.Add(l0);
            }
        }
        public static void create_dir(string path)
        {
            string directoryName = Path.GetDirectoryName(path);
            if ((directoryName.Length > 0) && (!Directory.Exists(directoryName)))
            {
                Directory.CreateDirectory(directoryName);
            }
        }
    }
}
