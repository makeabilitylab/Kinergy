using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
namespace Kinergy.Geom
{
    class YokeSlider:Component
    {
        Vector3d _mainAxis=Vector3d.Unset;
        Point3d _currentPosition = Point3d.Unset;
        public YokeSlider(Brep model,Vector3d mainAxis,Point3d initialPosition)
        {
            base.model = model;
            _mainAxis = mainAxis;
            _mainAxis.Unitize();
            _currentPosition = initialPosition;
        }
        public Movement SolveSliding(Movement m)
        {
            if(m.Type == 2 && IsEngaged((Component)m.Obj) && m.Obj.GetType() == typeof(DrivingWheel))
            {
                DrivingWheel wheel = (DrivingWheel)m.Obj;
                Point3d pinPos = wheel.getPinPos(m.MovementValue);
                //Calculate the new pos of yoke slider
                Vector3d v = pinPos - _currentPosition;
                v = v * _mainAxis * _mainAxis;
                return new Movement(this, 1,v.Length ,Transform.Translation(v));
            }
            throw new Exception("Unexpected movement passed to YokeSlider Solver");
        }
        public override bool IsEngaged(Component obj)
        {
            bool engaged = false;
            foreach (Relationship.Relationship r in this.constraints)
            {
                if (r.Obj1 == this && r.Obj2 == obj || r.Obj2 == this && r.Obj1 == obj)
                {
                    engaged = true;
                    break;
                }

            }
            return engaged;
        }
        public override bool Move(Movement move)
        {
            _currentPosition.Transform(move.Trans);
            return base.Move(move);
        }
    }
}
