using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino;
using Rhino.Geometry;
using Kinergy.KineticUnit;
namespace Kinergy.Geom
{
    public class Wheel:Component
    {
        Point3d centerPoint = Point3d.Unset;
        Vector3d normal = Vector3d.Unset;
        double radius = 0, thickness = 0;
        KineticUnit.KineticUnit KU = null;
        Line direction;

        public Line Direction { get => direction; set => direction = value; }

        public Wheel(Point3d cp,Vector3d n,double r,double t,KineticUnit.KineticUnit parent)
        {
            centerPoint = cp;
            normal = n;
            radius = r;
            thickness = t;
            KU = parent;
            Generate();
        }
        public Wheel(Brep inputWheelBrep, Line dir, KineticUnit.KineticUnit parent)
        {
            direction = dir;
            radius = 22.5;
            Model = inputWheelBrep;
            KU = parent;
        }
        public override void Generate()
        {
            Circle c = new Circle(new Plane(centerPoint, normal), radius);
            Cylinder cy = new Cylinder(c);
            cy.Height1 = -thickness / 2;
            cy.Height2 = thickness / 2;
            model = cy.ToBrep(true, true);
        }
        public override bool Move(Movement move)
        {
            if (move.Type == 2)
            {
                //return base.Move(move);
                ConductMoveAndUpdateParam(move);
                return true;
            }
            else
                return false;

        }
        protected override void ConductMoveAndUpdateParam(Movement move)
        {
            KU.Translate(-move.MovementValue * radius*5);
        }

    }
}
