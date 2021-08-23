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
        class Shaft: Component
        {
            private Point3d _startPt;
            private Vector3d _axisDir;
            private double _len;
            private double _radius;
            private RhinoDoc _myDoc;

            public Point3d StartPt { get => _startPt; set => _startPt = value; }
            public double Len { get => _len; set => _len = value; }
            public double Radius { get => _radius; set => _radius = value; }
            public Vector3d AxisDir { get => _axisDir; set => _axisDir = value; }

            public Shaft(Point3d staPt, double length, double r, Vector3d dir)
            {
                _startPt = staPt;
                _axisDir = dir;
                _len = length;
                _radius = r;
                _myDoc = RhinoDoc.ActiveDoc;

                GenerateShaft();

            }

            public void GenerateShaft()
            {
                Point3d startPt = _startPt;
                Point3d endPt = _startPt + _axisDir / _axisDir.Length * _len;
                Curve shaftCrv = new Line(startPt, endPt).ToNurbsCurve();

                Brep shaftBrep = Brep.CreatePipe(shaftCrv, _radius, false, PipeCapMode.Flat, true, _myDoc.ModelAbsoluteTolerance, _myDoc.ModelAngleToleranceRadians)[0];

                shaftBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == shaftBrep.SolidOrientation)
                    shaftBrep.Flip();

                base.Model = shaftBrep;
            }
        }
    }
    
}
