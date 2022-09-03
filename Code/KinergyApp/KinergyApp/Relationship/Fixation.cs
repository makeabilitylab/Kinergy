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
                    Movement transmittedMovement=new Movement(base.TheOtherEntity(move.Obj),1,move.MovementValue,move.Trans);
                    return transmittedMovement.Activate();
                }
                if(move.Type==2 && base.TheOtherEntity(move.Obj).GetType()==typeof(Gear) && move.Obj.GetType() == typeof(Gear))
                {
                    Gear g1=(Gear) move.Obj,g2=(Gear)base.TheOtherEntity(move.Obj);
                    Transform rotation = Transform.Rotation(move.MovementValue, g2.Direction, g2.CenterPoint);
                    Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 2, move.MovementValue,rotation);
                    return transmittedMovement.Activate();
                }
                else if (move.Type == 2 && base.TheOtherEntity(move.Obj).GetType() == typeof(Gear))
                {
                    Entity g1 = move.Obj;
                    Gear g2 = (Gear)base.TheOtherEntity(move.Obj);
                    Transform rotation = Transform.Rotation(move.MovementValue, g2.Direction, g2.CenterPoint);
                    Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 2, move.MovementValue,rotation);
                    return transmittedMovement.Activate();
                }
                else if (move.Type == 2 && move.Obj.GetType() == typeof(Gear))
                {
                    Gear g1 =(Gear) move.Obj;
                    Entity obj2 = base.TheOtherEntity(move.Obj);
                    Transform rotation = Transform.Rotation(move.MovementValue, g1.Direction, g1.CenterPoint);
                    Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 2, move.MovementValue, rotation);
                    return transmittedMovement.Activate();
                }
                if (move.Type == 2 && base.TheOtherEntity(move.Obj).GetType() == typeof(Spiral))
                {
                    Spiral s = (Spiral)base.TheOtherEntity(move.Obj);
                    Movement transmittedMovement = new Movement(s, 4, move.MovementValue);
                    return transmittedMovement.Activate();
                }
                if(move.Type == 2)
                {
                    Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 2, move.MovementValue,move.Trans);
                    return transmittedMovement.Activate();
                }
                if (move.Type==3)
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
                        Vector3d dir = s.Direction / s.Direction.Length;
                        Transform translation = Transform.Translation(-dir * move.MovementValue);
                        Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 1, -move.MovementValue,translation);
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
                        Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 2,move.MovementValue, move.Trans );
                        return transmittedMovement.Activate();
                        //throw new NotImplementedException();
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
