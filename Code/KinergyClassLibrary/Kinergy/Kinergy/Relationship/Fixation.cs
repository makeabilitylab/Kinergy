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
        public class Fixation:Relationship
        {
            private int contactPosition =0;
            /// <summary>
            /// Indicating where the fixation is at when connected to spring or spiral. 0:unset; 1:at fixed end; 2:at moving end;
            /// </summary>
            public int ContactPosition { get => contactPosition; private set => contactPosition = value; }
            public Fixation(Entity object1, Entity object2):base(object1,object2)
            {
                //Type check here.
                if(object1.GetType()==typeof(Helix)|| object1.GetType() == typeof(Spiral))
                {
                    contactPosition = object1.GetContactPosition(object2);
                }
                if (object2.GetType() == typeof(Helix) || object2.GetType() == typeof(Spiral))
                {
                    contactPosition = object2.GetContactPosition(object1);
                }
            }
            public void ReverseContactPosition()
            {
                if (contactPosition == 0)
                {
                    throw new Exception("Contact position is not registered!");
                }
                else if (contactPosition == 1)
                {
                    contactPosition = 2;
                }
                else if (contactPosition == 2)
                {
                    contactPosition = 1;
                }
                else
                {
                    throw new Exception("Invalid contact position indicator!");
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
                    if(contactPosition==0)
                    {
                        throw new Exception("Contact position is not registered!");
                    }
                    else if(contactPosition==1)
                    {
                        return true;
                    }
                    else if(contactPosition==2)
                    {
                        Helix s =(Helix) move.Obj;
                        //Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 1, Transform.Translation(s.Direction/s.Direction.Length*move.MovementValue));
                        Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 1, move.Trans);
                        return transmittedMovement.Activate();
                    }
                    else
                    {
                        throw new Exception("Invalid contact position indicator!");
                    }
                    
                }
                if(move.Type==4)
                {
                    //TODO move spiral
                    if (contactPosition == 0)
                    {
                        throw new Exception("Contact position is not registered!");
                    }
                    else if (contactPosition == 1)
                    {
                        return true;
                    }
                    else if (contactPosition == 2)
                    {
                        Spiral s = (Spiral)move.Obj;
                        //Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 1, Transform.Translation(s.Direction / s.Direction.Length * move.MovementValue));
                        //return transmittedMovement.Activate();
                        throw new NotImplementedException();
                    }
                    else
                    {
                        throw new Exception("Invalid contact position indicator!");
                    }
                }
                return false;
            }
        }
    }
}
