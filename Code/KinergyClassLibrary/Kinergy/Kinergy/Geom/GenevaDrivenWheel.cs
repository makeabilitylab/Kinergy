using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace Kinergy.Geom
{
    public class GenevaDrivenWheel:Component
    {
        private GenevaDrivingWheel driver = null;
        private double wheelRadius;
        private double wheelThickness;
        private Point3d centerPoint;
        private Vector3d normal;
        private int n;
        private double angle = 0;//Between 0 and 2pi

        public Point3d CenterPoint { get => centerPoint;private set => centerPoint = value; }
        public int N { get => n;private  set => n = value; }
        public double Angle { get => angle;private  set => angle = value; }

        public GenevaDrivenWheel(GenevaDrivingWheel source, Point3d cp,int num, double thickness)
        {
            driver = source;
            double d = source.RollerRadius, a = source.WheelRadius;
            wheelRadius = Math.Sqrt(d * d + a * a * Math.Pow((Math.Cos(Math.PI / num) / Math.Sin(Math.PI / num)), 2));
            wheelThickness = thickness;
            centerPoint = cp;
            normal = source.Normal;
            n = num;
            Generate();
        }
        public void FixParameter()
        {
            //TODO adjust centerPoint pos and other parameters with given info
            double centerDis = driver.WheelRadius / Math.Sin(Math.PI / n);
        }
        public override void Generate()
        {
            FixParameter();
            Plane pl1 = new Plane(centerPoint, normal);
            Vector3d centerVector = new Vector3d(driver.CenterPoint)- new Vector3d(centerPoint);
            //Make sure the center Vector is perpendicullar to normal
            centerVector -= centerVector * normal * normal;
            Circle c1 = new Circle(pl1, wheelRadius);
            Cylinder cy1 = new Cylinder(c1);
            cy1.Height1 = 0;
            cy1.Height2 = wheelThickness;
            Brep m = cy1.ToBrep(true, true);
            //Cut the slots
            //First construct one box
            double centerDis= driver.WheelRadius / Math.Sin(Math.PI / n);
            Box b = new Box(pl1, new Interval(centerDis - driver.WheelRadius - driver.RollerRadius, wheelRadius),
                new Interval(-driver.RollerRadius * 1.05, driver.RollerRadius * 1.05), new Interval(-wheelThickness, wheelThickness));
            for(int i=0;i<n;i++)
            {
                Brep box = b.ToBrep();
                box.Transform(Transform.Rotation(Math.PI * 2*i / n, normal, centerPoint));
                m = Brep.CreateBooleanDifference(m, box, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            }
            model = m;
            SetAngleZeroPosition();
        }
        private void SetAngleZeroPosition()
        {
            if (driver == null)
                return;
            Vector3d centerVector = new Vector3d(driver.CenterPoint)-new Vector3d(centerPoint);
            //Make sure the center Vector is perpendicullar to normal
            centerVector -= centerVector * normal * normal;
            //Then rotate
            Plane pl1 = new Plane(centerPoint, normal);
            Transform r = Transform.Rotation(pl1.XAxis, centerVector, centerPoint);
            model.Transform(r);
            angle = 0;

        }
        public void SetAngle(double a)
        {
            //TODO
            a = a % (Math.PI * 2);
            Movement m = new Movement(this, 2, a - angle);
            ConductMoveAndUpdateParam(m);
        }
        public void Follow(bool clockwise=true)
        {
            //Tell if the other part makes this wheel rotate. If so, follow it by movement, so other parts would follow.
            if(Math.Cos(driver.Angle)>Math.Sin(Math.PI/n))
            {
                double a = Math.Cosh(Math.Sin(Math.PI / n));
                double alpha = driver.Angle + a;
                double b = Math.PI / n;
                bool reverse = false;
                if (alpha > a)
                {
                    alpha = a*2 - alpha;
                    reverse = true;
                }
                double targetAngle = alpha * b / a;
                if (!reverse)
                {
                    targetAngle = -targetAngle;
                }
                //Then rotate to meet the target angle;
                double currA = (angle+b) % (b * 2);
                double diff = 0;
                if(clockwise)
                {
                    diff = targetAngle - currA;
                    if (diff < 0)
                        diff += b * 2;
                }
                else
                {
                    diff = targetAngle - currA;
                    if (diff > 0)
                        diff -= b * 2;
                }
                Movement m=new Movement(this,2,diff);
                m.Activate();
            }
        }
        public override bool Move(Movement move)
        {
            if (move.Type == 2)//The wheel only do rotation. Cancel this if exception occur
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
