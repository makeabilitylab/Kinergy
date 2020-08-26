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
    namespace Relationships
    { 
        public class Locking:Relationship
        {
            Lock l1, l2;
            public Locking (Lock obj1,Lock obj2 ):base(obj1,obj2)
            {
                l1 = obj1;
                l2 = obj2;
            }
            public override bool Move(Movement move)
            {
                if (l1.Locked || l2.Locked)
                {
                    string body = string.Format("The locking is locked!");
                    Rhino.RhinoApp.WriteLine(body);
                    return false;
                }
                else
                    return true;
            }
            
        }
    }
}
