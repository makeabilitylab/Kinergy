using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KinergyUtilities;
using Kinergy;
using Rhino.Geometry;
namespace Kinergy.Geom
{
    public class Socket:Entity
    {
        Point3d centerPoint;
        Vector3d direction;
        public Socket(Point3d p, Vector3d v)
        {
            center = p;
            direction = v/v.Length;
            Brep primitiveModel = KinergyUtilities.FileOperation.SingleBrepFromResourceFileDirectory(FileOperation.FindCurrentFolderResourceDirectory() + "\\RevoluteJointSocket.3dm");
            model = primitiveModel.DuplicateBrep();
            model.Transform(Transform.Rotation(Vector3d.XAxis, direction, model.GetBoundingBox(true).Center));
            model.Transform(Transform.Translation(new Vector3d(p - model.GetBoundingBox(true).Center)));
        }
    }
}
