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
        public class Fixation:Constraint
        {
            private int contactPosition = 0;
            
            public int ContactPosition { get => contactPosition; private set => contactPosition = value; }
            public Fixation(Entity object1, Entity object2):base(object1,object2)
            {
                //Type check here.
                if(object1.GetType()==typeof(Spring)|| object1.GetType() == typeof(Spiral))
                {
                    contactPosition = object1.GetContactPosition(object2);
                }
                if (object2.GetType() == typeof(Spring) || object2.GetType() == typeof(Spiral))
                {
                    contactPosition = object2.GetContactPosition(object1);
                }
            }

            

            public override bool Move(Movement move)
            {
                if(move.Type==1)
                {
                    Movement transmittedMovement=new Movement(base.TheOtherEntity(move.Obj),1,move.Trans);
                    return transmittedMovement.Activate();
                }
                if(move.Type==2 && base.TheOtherEntity(move.Obj).GetType()==typeof(Gear))
                {
                    Gear g1=(Gear) move.Obj,g2=(Gear)base.TheOtherEntity(move.Obj);
                    Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 2, move.MovementValue);
                    return transmittedMovement.Activate();
                }
                if(move.Type==3)
                {
                    Spring s =(Spring) move.Obj;
                    Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 1, Transform.Translation(s.Direction/s.Direction.Length*move.MovementValue));
                    return transmittedMovement.Activate();
                }
                if(move.Type==4)
                {
                    //TODO move spiral
                }
                return false;
            }
        }
    }
}
