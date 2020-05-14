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
using Grasshopper.Kernel.Types.Transforms;
using Kinergy.Geom;


namespace Kinergy
{
    public class Movement
    {
        //Define a physical movement, including linear and rotating movements.
        private Entity obj;
        private int type;//1 means linear movement, 2 means self-rotate . 3 means spring squeezing and 4 means spiral rotating
        private double movementValue;
        //private Point3d rotateCenter = Point3d.Unset;
        private Transform trans = Transform.Unset;
        private bool converge=false;
        public Entity Obj { get => obj; private set => obj = value; }
        /// <summary>
        /// 1 means linear movement, 2 means self-rotate . 3 means spring squeezing and 4 means spiral rotating
        /// </summary>
        public int Type { get => type;private set => type = value; }
        
        //public Point3d RotateCenter { get => rotateCenter;private set => rotateCenter = value; }
        public Transform Trans { get => trans;private set => trans = value; }
        public double MovementValue { get => movementValue; private set => movementValue = value; }
        public bool Converge { get => converge;protected set => converge = value; }

        /// <summary>
        /// Constructor for linear movement.
        /// </summary>
        /// <param name="Object">Object to be transformed</param>
        /// <param name="Tp">Type of movement, here should be 1 for linear movement</param>
        /// <param name="m">Transform</param>
        public Movement(Entity Object, int Tp,Transform m)
        {
            if(Tp==2)
            { throw new Exception("Please use the other constructor for self-rotation"); }
            if(Tp!=1)
            { throw new Exception("Movement of this type hasn't been implemented."); }
            obj = Object;
            Type = Tp;
            Trans = m;
        }
        /// <summary>
        /// Constructor for self-rotation movement
        /// </summary>
        /// <param name="Object">Object to be transformed,here should be a gear</param>
        /// <param name="Tp">Type of movement, here should be 2 for self rotation</param>
        /// <param name="deg">Degree of rotation</param>
        public Movement(Entity Object,int Tp,double value)
        {
            if (Tp == 1)
            { throw new Exception("Please use the other constructor for linear movement"); }
            if (Tp == 2)
            {
                if (Object.GetType() != typeof(Gear))
                {
                    obj = Object;
                    Type = Tp;
                    movementValue = value;
                }
                else
                {
                    obj = Object;
                    Gear g = (Gear)Object;
                    Type = Tp;
                    Trans = Transform.Rotation(movementValue / 180 * Math.PI, g.Direction, g.CenterPoint);
                    movementValue = value;
                }
            }
            else if(Tp==3)
            {
                if (Object.GetType() != typeof(Spring))
                { throw new Exception("Movement of type3 only support springs."); }
                obj = Object;
                Type = Tp;
                movementValue = value;
            }
            else if(Tp==4)
            {
                if (Object.GetType() != typeof(Spiral))
                { throw new Exception("Movement of type4 only support spirals."); }
                obj = Object;
                Type = Tp;
                movementValue = value;
            }
            else
            {throw new Exception("Movement of this type hasn't been implemented."); }
        }
        
        public bool Activate()
        {
            return obj.Move(this);
        }
        public void SetConverge()
        {
            converge = true;
        }
    }
}
