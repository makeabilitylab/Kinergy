using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using Grasshopper.Kernel.Components;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.DocObjects;
using Rhino.Collections;
using Rhino.Input.Custom;
using Rhino;
using Kinergy.Geom;
using Rhino.FileIO;
using System.Diagnostics;
using Rhino.Commands;

using Rhino.Runtime;
namespace KinergyUtilities
{
    /// <summary>
    /// This namespace contains convenient file operations
    /// </summary>
    
        
    public class FileOperation
    {
        /// <summary>
        /// This method is deprecated. Instead of finding file directories, please use resources manager to add resources files into the Kinergy class library project.
        /// </summary>
        /// <returns></returns>
        public static String FindComponentFolderDirectory()
            {//TODO This directory isn't right!
                string dir = Environment.CurrentDirectory;
                System.IO.DirectoryInfo pathInfo = new System.IO.DirectoryInfo(dir);
                string newPath = pathInfo.Parent.FullName;
                return newPath + "\\Plug-ins\\Grasshopper\\Components";

            }
        /// <summary>
        /// This method is deprecated. Instead of finding file directories, please use resources manager to add resources files into the Kinergy class library project.
        /// </summary>
        /// <returns></returns>
        public static String FindCurrentFolderResourceDirectory()
            {
            string path = System.IO.Directory.GetCurrentDirectory();
            System.IO.DirectoryInfo pathInfo = new System.IO.DirectoryInfo(path);
            string newPath = pathInfo.Parent.FullName;
            
            return newPath + "\\Resources";

            }
        /// <summary>
        /// This method returns the single brep from file directory
        /// </summary>
        public static Brep SingleBrepFromResourceFileDirectory(string fileDirectory)
        {
            Rhino.FileIO.File3dm file = Rhino.FileIO.File3dm.Read(fileDirectory);
            //Rhino.FileIO.File3dm file = Create3dmFileFromBytes( Properties.Resources.LockBaseContinuousMovingPress);
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
        public static Point3d SinglePointFromResourceFileDirectory(string fileDirectory)
        {
            Rhino.FileIO.File3dm file = Rhino.FileIO.File3dm.Read(fileDirectory);
            List<Point3d> pts = new List<Point3d>();
            foreach (Rhino.FileIO.File3dmObject obj in file.Objects)
            {

                Point3d pt =Point3d.Unset;
                if (GH_Convert.ToPoint3d(obj.Geometry, ref pt, GH_Conversion.Both))
                {
                    pts.Add(pt);
                }
            }
            file.Dispose();

            return pts[0];
        }
        /// <summary>
        /// This method returns multiple breps from file directory as a list
        /// </summary>
        public static List<Brep> BrepsFromResourceFileDirectory(string fileDirectory)
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
        /// <summary>
        /// This method takes binary 3dm file and returns the first brep model in it.
        /// </summary>
        /// <param name="bytes">The binary formatted of 3dm file. Please use resources manager to add 3dm files in this project. Then it could be accessed as binary format</param>
        /// <returns>First brep in file</returns>
        /*public static Brep SingleBrepFromResourceFile(byte[] bytes)
        {
            Rhino.FileIO.File3dm file = Create3dmFileFromBytes( bytes);
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
        /// This method takes binary 3dm file and returns the first point3d in it.
        /// </summary>
        /// <param name="bytes">The binary formatted of 3dm file. Please use resources manager to add 3dm files in this project. Then it could be accessed as binary format</param>
        /// <returns>First point3d in file</returns>
        public static Point3d SinglePointFromResourceFile(byte[] bytes)
        {
            Rhino.FileIO.File3dm file = Create3dmFileFromBytes(bytes);
            List<Point3d> pts = new List<Point3d>();
            foreach (Rhino.FileIO.File3dmObject obj in file.Objects)
            {

                Point3d pt = Point3d.Unset;
                if (GH_Convert.ToPoint3d(obj.Geometry, ref pt, GH_Conversion.Both))
                {
                    pts.Add(pt);
                }
            }
            file.Dispose();

            return pts[0];
        }
        /// <summary>
        /// This method takes binary 3dm file and returns the brep models in it as a list.
        /// </summary>
        /// <param name="bytes">The binary formatted of 3dm file. Please use resources manager to add 3dm files in this project. Then it could be accessed as binary format</param>
        /// <returns>Brep models</returns>
        public static List<Brep> BrepsFromResourceFile(byte[] bytes)
        {
            Rhino.FileIO.File3dm file = Create3dmFileFromBytes(bytes);
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
        /// <summary>
        /// This method takes binary formatted 3dm file and return it as File3dm object.
        /// </summary>
        /// <param name="bytes">Binary formatted 3dm file.Please use resources manager to add 3dm files in this project. Then it could be accessed as binary format</param>
        /// <returns></returns>
        public static Brep CreateBrepFromBytes(byte[] bytes)
        {
            if (null == bytes || 0 == bytes.Length)
                return null;

            File3dm file = null;
            try
            {
                using (var stream = new MemoryStream())
                {
                    var formatter = new BinaryFormatter();
                    stream.Write(bytes, 0, bytes.Length);
                    stream.Seek(0, SeekOrigin.Begin);
                    var obj = formatter.Deserialize(stream) as CommonObject;
                    if (null != obj)
                        file = obj as File3dm;
                }
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.Message);
            }

            return file;
        }*/
    }
    
}
