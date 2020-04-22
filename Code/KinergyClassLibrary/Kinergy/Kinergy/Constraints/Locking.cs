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
namespace Kinergy
{
    namespace Constraints
    { 
        public class Locking:Constraint
        {
            public Locking (Lock obj1,Lock obj2 ):base(obj1,obj2)
            {

            }
            public override bool Move(Movement move)
            {
                return false;
            }
            public void Release()
            {
                this.obj1.Constraints.Remove(this);
                this.obj2.Constraints.Remove(this);
            }
        }
    }
}
