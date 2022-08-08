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
        public class Engagement:Relationship
        {
            public Engagement(Component obj1, Component obj2):base(obj1,obj2)
            {
                //First, do type check. Engagement can only be between 2 gears or gear and rack. 
                Type type1 = obj1.GetType();
                Type type2 = obj2.GetType();

                if(!((type1.Equals(typeof(Gear)) && type2.Equals(typeof(Rack))) ||
                    (type1.Equals(typeof(Rack)) && type2.Equals(typeof(Gear))) ||
                    (type1.Equals(typeof(Gear)) && type2.Equals(typeof(Gear)))||
                    (type1.Equals(typeof(DrivingWheel)) && type2.Equals(typeof(GenevaDrivenWheel)))||
                    (type1.Equals(typeof(GenevaDrivenWheel)) && type2.Equals(typeof(DrivingWheel)))||
                    (type1.Equals(typeof(DrivingWheel)) && type2.Equals(typeof(YokeSlider)))||
                    (type1.Equals(typeof(YokeSlider)) && type2.Equals(typeof(DrivingWheel))) ||
                    (type1.Equals(typeof(DrivingWheel)) && type2.Equals(typeof(Lever))) ||
                    (type1.Equals(typeof(Lever)) && type2.Equals(typeof(DrivingWheel)))))
                {
                    throw new Exception("Illegal engagement between these two objects!");
                }
          
                //Then do the IsEngaged check
                //if(obj1.IsEngaged(obj2)==false)
                //{
                //    throw new Exception("Entities not engaged!");
                //}

            }
            public override bool Move(Movement move)
            {
                if(move.Obj.GetType()==typeof(Gear) && base.TheOtherEntity(move.Obj).GetType()==typeof(Gear))
                {
                    Gear g1 = (Gear)move.Obj, g2 = (Gear)base.TheOtherEntity(move.Obj);
                    Transform rotation = Transform.Rotation(-move.MovementValue * g1.NumTeeth / g2.NumTeeth, g2.Direction, g2.CenterPoint);
                    Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 2, -move.MovementValue * g1.NumTeeth / g2.NumTeeth,rotation); // ToDo: update this
                    return transmittedMovement.Activate();
                }
                else if(move.Obj.GetType() == typeof(Gear) && base.TheOtherEntity(move.Obj).GetType() == typeof(Rack))
                {
                    Gear g = (Gear)move.Obj;
                    Rack r = (Rack) base.TheOtherEntity(move.Obj);
                    double distance=move.MovementValue* ((g.RootRadius + g.TipRadius) / 2);
                    Transform translation = Transform.Translation(r.RackDirection * distance);
                    Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 1, distance,translation);
                    return transmittedMovement.Activate();
                }
                else if(move.Obj.GetType() == typeof(Rack) && base.TheOtherEntity(move.Obj).GetType() == typeof(Gear))
                {
                    Gear g = (Gear)base.TheOtherEntity(move.Obj);
                    Rack r = (Rack)move.Obj;
                    double value = -move.MovementValue / ((g.RootRadius + g.TipRadius) / 2);
                    Transform rotation = Transform.Rotation(value, g.Direction, g.CenterPoint);
                    Movement transmittedMovement = new Movement(base.TheOtherEntity(move.Obj), 2, value, rotation);
                    return transmittedMovement.Activate();
                }
                else if(move.Obj.GetType() == typeof(DrivingWheel)&& base.TheOtherEntity(move.Obj).GetType() == typeof(Lever))
                {
                    Lever l = (Lever)base.TheOtherEntity(move.Obj);
                    Movement transmittedMovement = l.solveRotation(move);
                    return transmittedMovement.Activate();
                }
                else if (move.Obj.GetType() == typeof(DrivingWheel) && base.TheOtherEntity(move.Obj).GetType() == typeof(YokeSlider))
                {
                    YokeSlider y = (YokeSlider)base.TheOtherEntity(move.Obj);
                    Movement transmittedMovement = y.SolveSliding(move);
                    return transmittedMovement.Activate();
                }
                else if (move.Obj.GetType() == typeof(DrivingWheel) && base.TheOtherEntity(move.Obj).GetType() == typeof(GenevaDrivenWheel))
                {
                    GenevaDrivenWheel g = (GenevaDrivenWheel)base.TheOtherEntity(move.Obj);
                    Movement transmittedMovement = g.SolveRotation(move);
                    return transmittedMovement.Activate();
                }
                return false;
            }
        }
    }
}
