using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
namespace Kinergy.Geom
{
    class Lever:Component
    {
        Point3d _axisCenter = Point3d.Unset;
        Vector3d _axisDir = Vector3d.Unset;
        Vector3d _mainAxis = Vector3d.Unset;
        double currentAngle = 0;
        public Lever(Brep model,Point3d axisCenter,Vector3d axisDir,Vector3d mainAxis,double initialAngle=0)
        {
            base.model = model;
            _axisCenter = axisCenter;
            _axisDir = axisDir;
            _axisDir.Unitize();
            _mainAxis = mainAxis;
            currentAngle = initialAngle;
        }
        public Movement solveRotation(Movement m)
        {
            if (m.Type == 2 && IsEngaged((Component)m.Obj) && m.Obj.GetType() == typeof(DrivingWheel))
            {
                //calculate the new angle
                DrivingWheel wheel = (DrivingWheel)m.Obj;
                Point3d pinPos = wheel.getPinPos(m.MovementValue);
                Vector3d leverVec = pinPos - _axisCenter;
                leverVec = leverVec - leverVec * _axisDir * _axisDir;
                double targetAngle = Vector3d.VectorAngle(_mainAxis, leverVec);
                //Determine this target angle as positive or negative
                Vector3d tryV = new Vector3d(_mainAxis);
                tryV.Rotate(targetAngle, _axisDir);
                if (Vector3d.VectorAngle(tryV, leverVec) > 0.01 && Math.Abs(targetAngle) > 0.005)
                {
                    targetAngle = -targetAngle;
                }
                return new Movement(this, 2, targetAngle - currentAngle, Transform.Rotation(targetAngle - currentAngle, _axisDir, _axisCenter));
            }
            else
                throw new Exception("Unexpected input given to rotation solver");
        }
        public override bool IsEngaged(Component obj)
        {
            bool engaged = false;
            foreach(Relationship.Relationship r in this.constraints)
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
