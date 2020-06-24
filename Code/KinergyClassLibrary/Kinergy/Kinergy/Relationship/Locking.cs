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
    namespace Relationship
    { 
        public class Locking:Relationship
        {
            public Locking (Lock obj1,Lock obj2 ):base(obj1,obj2)
            {

            }
            public override bool Move(Movement move)
            {
                return false;
            }
            
        }
    }
}
