﻿using System;
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
        public class Engagement:Relationship
        {
            public Engagement(Component obj1, Component obj2):base(obj1,obj2)
            {
                //First, do type check. Engagement can only be between 2 gears or gear and rack. 
                Type type1 = obj1.GetType();
                Type type2 = obj2.GetType();
                if((type1!=typeof(Gear) && type1!=typeof(Rack))||(type2 != typeof(Gear) && type2 != typeof(Rack)))
                {
                    throw new Exception("Illegal entity given to engagement constraint!");
                }
                if(type1==typeof(Rack) && type2 == typeof(Rack))
                {
                    throw new Exception("It is illegal to set up engagement between two racks!");
                }
                //Then do the IsEngaged check
                //if(obj1.IsEngaged(obj2)==false)
                //{
                //    throw new Exception("Entities not engaged!");
                //}

            }
            public override bool Move(Movement move)
            {
                Gear g1 = (Gear)move.Obj, g2 = (Gear)base.TheOtherEntity(move.Obj);
                Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 2, -move.MovementValue * g1.Z / g2.Z);
                return transmittedMovement.Activate();
            }
        }
    }
}
