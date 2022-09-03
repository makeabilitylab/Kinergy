using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
namespace Kinergy.Geom
{
    class DrivingWheel:Component
    {
        Point3d _axisCenter = Point3d.Unset;
        Vector3d _axisDir = Vector3d.Unset;
        Vector3d _mainAxis = Vector3d.Unset;
        double _pinRadius = 0;
        double currentAngle = 0;//In radians

        public double CurrentAngle { get => currentAngle;  }

        public DrivingWheel(Brep model, Point3d axisCenter, Vector3d axisDir, Vector3d mainAxis,double pinRadius ,double initialAngle = 0)
        {
            base.model = model;
            _axisCenter = axisCenter;
            _axisDir = axisDir;
            _axisDir.Unitize();
            _mainAxis = mainAxis;
            _pinRadius = pinRadius;
            currentAngle = initialAngle;
        }
        
        /// <summary>
        /// Get the current pin position of the driving wheel.
        /// </summary>
        /// <param name="incrementAngle">In radians</param>
        /// <returns></returns>
        public Point3d getPinPos(double incrementAngle=0)
        {
            Vector3d v = _mainAxis * _pinRadius;
            Transform r=Transform.Rotation(currentAngle + incrementAngle, _axisDir, _axisCenter);
            v.Transform(r);
            return _axisCenter + v;
        }
        protected override void ConductMoveAndUpdateParam(Movement move)
        {
            //This method would be overridden by different kinds of entity.Here constraints dosen't matter, just move the model by changing offset and rotate!
            bool valid = true;
            if (!offset.IsValid)
            {
                Rhino.RhinoApp.WriteLine("Invalid trans is passed");
                valid = false;
            }
            if (!offset.IsValid && valid)
            {
                Rhino.RhinoApp.WriteLine("Invalid trans is caused");
                throw new Exception("Invalid trans is caused");
            }
            if(move.Type!=2)
            {
                Rhino.RhinoApp.WriteLine("Invalid move type passed to driving wheel");
                throw new Exception("Invalid move type passed to driving wheel");
            }
            currentAngle += move.MovementValue;
            while (currentAngle < 0)
                currentAngle += (Math.PI * 2);
            currentAngle = currentAngle % (Math.PI * 2);
            Transform tr = new Transform(offset);
            offset = Transform.Multiply(tr, move.Trans);
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
