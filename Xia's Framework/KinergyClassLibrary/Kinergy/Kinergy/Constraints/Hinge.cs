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
using Kinergy.Utilities;

namespace Kinergy.Constraints
{
    public class Hinge:Constraint
    {
        Point3d[] centerPoint ;
        Vector3d axis = Vector3d.Unset;
        public Point3d[] CenterPoint { get => centerPoint; set => centerPoint = value; }
        public Vector3d Axis { get => axis; set => axis = value; }

        public Hinge(Entity obj1,Entity obj2,Point3d[] CP,Vector3d AX):base(obj1,obj2)
        {
            centerPoint = CP;
            axis = AX;
        }
        public override bool Move(Movement move)
        {
            if (move.Type == 1)
            {
                return false;
            }
            if (move.Type == 2 )
            {
                return true;
            }
            if (move.Type == 3)
            {
                throw new Exception("Movement of type3 trainsmitted to a hinge constraint.");
            }
            if (move.Type == 4)
            {
                throw new Exception("Movement of type4 trainsmitted to a hinge constraint.");
            }
            return false;
        }

    }
}
