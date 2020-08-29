using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinergy.Geom
{
    public class GenevaDrivingWheel:Component
    {
        private GenevaDrivenWheel drivenWheel = null;
        private double wheelRadius,rollerRadius;
        private double rollerThickness, wheelThickness;
        private Point3d centerPoint;
        private Vector3d normal;
        private double angle = 0;//Between 0 and 2pi

        public double WheelRadius { get => wheelRadius;private  set => wheelRadius = value; }
        public double RollerRadius { get => rollerRadius;private  set => rollerRadius = value; }
        public Vector3d Normal { get => normal;private  set => normal = value; }
        public Point3d CenterPoint { get => centerPoint;private  set => centerPoint = value; }
        public double Angle { get => angle;private  set => angle = value; }

        public GenevaDrivingWheel(Point3d ct,Vector3d n,double wheelR,double rollerR,double rollerT=3,double wheelT=3)
        {
            centerPoint = ct;
            normal = n;
            normal.Unitize();
            wheelRadius = wheelR;
            rollerRadius = rollerR;
            rollerThickness = rollerT;
            wheelThickness = wheelT;
            Generate();
        }
        private void FixParameter()
        {
            //TODO check if any parameter mapping is needed
        }
        public override void Generate()
        {
            FixParameter();
            Plane pl1 = new Plane(centerPoint, normal);
            Circle c1 = new Circle(pl1, wheelRadius+rollerRadius);
            Cylinder cy1 = new Cylinder(c1);
            cy1.Height1 = -wheelThickness;
            cy1.Height2 = 0;
            Brep m = new Brep();
            m.Append(cy1.ToBrep(true, true));
            Plane pl2 = new Plane(centerPoint + pl1.XAxis * wheelRadius, normal);
            Circle c2= new Circle(pl2, rollerRadius);
            Cylinder cy2 = new Cylinder(c2);
            cy2.Height1 = 0;
            cy2.Height2 = rollerThickness;
            m.Append(cy2.ToBrep(true, true));
            model = m;
        }
        public GenevaDrivenWheel ConstructDrivenWheel(Vector3d CenterLinkDirection,int num,double thickness=3)
        {
            //TODO
            double centerDis = WheelRadius / Math.Sin(Math.PI / num);
            CenterLinkDirection -= CenterLinkDirection * normal * normal;
            CenterLinkDirection.Unitize();
            Point3d cp = centerPoint + CenterLinkDirection * centerDis+normal*wheelThickness;
            drivenWheel = new GenevaDrivenWheel(this, cp, num, thickness);
            SetAngleZeroPosition();
            return drivenWheel;
        }
        /// <summary>
        /// This funtion rotates this wheel to make the roller nearest to driven wheel and set this pos as angle = 0.
        /// Since the position of roller cannot be directly detected, this function should be called only when the wheel is constructed and never rotated.
        /// </summary>
        private void SetAngleZeroPosition()
        {
            if (drivenWheel == null)
                return;
            Vector3d centerVector = new Vector3d( drivenWheel.CenterPoint)-new Vector3d(centerPoint);
            //Make sure the center Vector is perpendicullar to normal
            centerVector -= centerVector * normal*normal;
            //Then rotate
            Plane pl1 = new Plane(centerPoint, normal);
            Transform r = Transform.Rotation(pl1.XAxis, centerVector, centerPoint);
            model.Transform(r);
            angle = 0;

        }
        public void SetAngle(double a)
        {
            a = a % (Math.PI * 2);
            Movement m = new Movement(this, 2, a - angle);
            ConductMoveAndUpdateParam(m);
            drivenWheel.Follow();
        }
        public void Follow()
        {
            int n = drivenWheel.N;
            //Tell if the driven wheel makes this wheel rotate. If so, follow it.
            if(Math.Cos(angle)>Math.Sin(Math.PI/ n))
            {
                double beta = (drivenWheel.Angle + Math.PI / n ) % (Math.PI * 2 / n);
                double a= Math.Cosh(Math.Sin(Math.PI / n));
                double b = Math.PI / n;
                bool reverse = false;
                if (beta > Math.PI / n)
                {
                    beta = Math.PI * 2 / n - beta;
                    reverse = true;
                }
                
                double targetAngle = beta * a / b;
                if(!reverse)
                {
                    targetAngle = -targetAngle+Math.PI*2;
                }
                SetAngle(targetAngle);
            }
        }
        public override bool Move(Movement move)
        {
            if (move.Type == 2)
                return base.Move(move);
            else
                return false;
        }
        protected override void ConductMoveAndUpdateParam(Movement move)
        {
            base.offset = Transform.Multiply(base.offset, Transform.Rotation(move.MovementValue, normal, centerPoint));
            angle += move.MovementValue;
        }
    }
}
