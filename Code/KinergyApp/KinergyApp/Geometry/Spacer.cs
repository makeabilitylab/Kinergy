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

namespace Kinergy
{
    namespace Geom
    {
        class Spacer: Component
        {
            private Point3d _startPt;
            private Vector3d _axisDir;
            private double _thickness;
            private double _innerRadius;
            private double _outerRadius;
            private double _barWidth;
            private RhinoDoc _myDoc;

            public Point3d StartPt { get => _startPt; set => _startPt = value; }
            public Vector3d AxisDir { get => _axisDir; set => _axisDir = value; }
            public double Thickness { get => _thickness; set => _thickness = value; }
            public double InnerRadius { get => _innerRadius; set => _innerRadius = value; }
            public double OuterRadius { get => _outerRadius; set => _outerRadius = value; }
            public double BarWidth { get => _barWidth; set => _barWidth = value; }

            public Spacer(Point3d staPt, double length, double ir, double or, Vector3d dir)
            {
                _startPt = staPt;
                _axisDir = dir;
                _thickness = length;
                _innerRadius = ir;
                _outerRadius = or;
                _myDoc = RhinoDoc.ActiveDoc;
                _barWidth = 0.8;

                GenerateSpacer();

            }

            public void GenerateSpacer()
            {
                Point3d startPt = _startPt;
                Point3d endPt = _startPt + _axisDir / _axisDir.Length * _thickness;
                Curve shaftCrv = new Line(startPt, endPt).ToNurbsCurve();

                Brep spacerInnerBrep = Brep.CreatePipe(shaftCrv, _innerRadius, false, PipeCapMode.Flat, true, _myDoc.ModelAbsoluteTolerance, _myDoc.ModelAngleToleranceRadians)[0];
                spacerInnerBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == spacerInnerBrep.SolidOrientation)
                    spacerInnerBrep.Flip();

                Brep spacerOuterBrep = Brep.CreatePipe(shaftCrv, _outerRadius, false, PipeCapMode.Flat, true, _myDoc.ModelAbsoluteTolerance, _myDoc.ModelAngleToleranceRadians)[0];
                spacerOuterBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == spacerOuterBrep.SolidOrientation)
                    spacerOuterBrep.Flip();

                Brep ringBrep = Brep.CreateBooleanDifference(spacerOuterBrep, spacerInnerBrep, _myDoc.ModelAbsoluteTolerance)[0];
                ringBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == ringBrep.SolidOrientation)
                    ringBrep.Flip();

                Plane basePln = new Plane(_startPt, _axisDir);
                Rectangle3d outline = new Rectangle3d(basePln, new Interval(-_barWidth/2, _barWidth/2), new Interval(-(_outerRadius+_innerRadius)/2, (_outerRadius+_innerRadius)/2));
                Brep b = Brep.CreateFromSweep(shaftCrv, outline.ToNurbsCurve(), true, _myDoc.ModelAbsoluteTolerance)[0];
                b = b.CapPlanarHoles(_myDoc.ModelAbsoluteTolerance);

                b.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == b.SolidOrientation)
                    b.Flip();

                //Brep spacerBrep = Brep.CreateBooleanUnion(new List<Brep> { ringBrep, b }, _myDoc.ModelAbsoluteTolerance)[0];
                Brep spacerBrep = spacerOuterBrep;
                base.Model = spacerBrep;
            }
        }

    }

}
