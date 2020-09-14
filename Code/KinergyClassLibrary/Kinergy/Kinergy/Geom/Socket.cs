using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Kinergy.Utilities;
using Kinergy;
using Rhino.Geometry;
namespace Kinergy.Geom
{
    public class Socket:Entity
    {
        Point3d centerPoint;
        Vector3d direction;
        double radius;
        public Socket(Point3d p, Vector3d v,double r)
        {
            center = p;
            direction = v;
            radius = r;
            double scale = r / 10;
            Brep primitiveModel = Utilities.FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\SocketRadius10mm.3dm");
            model = primitiveModel.DuplicateBrep();
            model.Transform(Transform.Scale(Point3d.Origin, scale));
            model.Transform(Transform.Rotation(Vector3d.XAxis, direction, Point3d.Origin));
            model.Transform(Transform.Translation(new Vector3d(p)));
        }
    }
}
