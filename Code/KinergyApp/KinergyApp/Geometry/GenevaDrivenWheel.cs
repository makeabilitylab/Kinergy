using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
namespace Kinergy.Geom
{
    class GenevaDrivenWheel:Component
    {
        Point3d _axisCenter = Point3d.Unset;
        Vector3d _axisDir = Vector3d.Unset;
        int _slotCount = 0;
        double currentAngle = 0;
        public GenevaDrivenWheel(Brep model,Point3d axisCenter,Vector3d axisDir,int slotCount,double initialAngle=0)
        {
            base.model = model;
            _axisCenter = axisCenter;
            _axisDir = axisDir;
            _slotCount = slotCount;
            currentAngle = initialAngle;
        }
        public Movement SolveRotation(Movement m)
        {
            if (m.Type == 2 && IsEngaged((Component)m.Obj) && m.Obj.GetType() == typeof(DrivingWheel))
            {
                //calculate the new angle
                DrivingWheel wheel = (DrivingWheel)m.Obj;
                double priorAngle = wheel.CurrentAngle;
                double posAngle = priorAngle + m.MovementValue;
                double rotationValue = 0;
                if(m.MovementValue>Math.PI*2)
                {
                    rotationValue += Math.PI * 2 / _slotCount * Math.Floor(m.MovementValue / (Math.PI * 2));
                    posAngle = posAngle - Math.Floor(m.MovementValue / (Math.PI * 2)) 
                        * Math.PI * 2;
                }
                if (priorAngle < Math.PI * 2 / _slotCount && posAngle < Math.PI * 2 / _slotCount)
                    rotationValue += m.MovementValue;
                else if(priorAngle < Math.PI * 2 / _slotCount && posAngle > Math.PI * 2 / _slotCount&& posAngle < Math.PI * 2)
                {
                    rotationValue += Math.PI * 2 / _slotCount-priorAngle;
                }
                else if(priorAngle < Math.PI * 2 / _slotCount && posAngle > Math.PI * 2 / _slotCount && posAngle > Math.PI * 2)
                {
                    rotationValue += Math.PI * 2 / _slotCount - priorAngle;
                    rotationValue += Math.Min(Math.PI * 2 / _slotCount, posAngle - Math.PI * 2);
                }
                else if(priorAngle > Math.PI * 2 / _slotCount && posAngle < Math.PI * 2)
                {
                    rotationValue += 0;
                }
                else if(priorAngle > Math.PI * 2 / _slotCount && posAngle > Math.PI * 2)
                {
                    rotationValue += Math.Min(Math.PI * 2 / _slotCount, posAngle - Math.PI * 2);
                }
                return new Movement(this, 2, rotationValue, Transform.Rotation(rotationValue, _axisDir, _axisCenter));
            }
            else
                throw new Exception("Unexpected input given to rotation solver");
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
    }
}
