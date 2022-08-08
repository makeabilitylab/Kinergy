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
        class GenevaDrive: Component
        {
            // More info about Geneva drive: http://fabacademy.org/2020/labs/irbid/students/willa-alzubi/Computer%20Aided%20Design%20CAD.html
            // b = Geneva wheel radius
            // n = driven slot quantity
            // p = drive pin diameter
            // t = allowed clearance
            // c = center distance 
            // a = drive crank radius
            // s = slot center length = (a+b) - c
            // w = slot width
            // y = stop arc radius 
            // z = stop disc radius 
            // v = clearance arc 

            private double _b;
            private int _n;
            private double _p;
            private double _t;
            private double _c;
            private double _a;
            private double _s;
            private double _w;
            private double _y;
            private double _z;
            private double _v;
            private Point3d _drivenCenPos;
            private Vector3d _drivenAxisDir;
            private double _thickness;
            private RhinoDoc _myDoc;
            private Curve _GenevaWheelCurve;
            private Curve _drivePinCurve;
            private Curve _driveWheelCurve;
            private Curve _spacerHalveCurve;
            private Curve _spacerHalveInnerCurve;
            private Vector3d _trajDir;

            private List<Brep> _genevaModels;
            public DrivingWheel drivingWheel = null;
            public GenevaDrivenWheel drivenWheel = null;

            public double B { get => _b; set => _b = value; }
            public int N { get => _n; set => _n = value; }
            public double P { get => _p; set => _p = value; }
            public double T { get => _t; set => _t = value; }
            public double C { get => _c; set => _c = value; }
            public double A { get => _a; set => _a = value; }
            public double S { get => _s; set => _s = value; }
            public double W { get => _w; set => _w = value; }
            public Point3d DrivenCenPos { get => _drivenCenPos; set => _drivenCenPos = value; }
            public Vector3d DrivenAxisDir { get => _drivenAxisDir; set => _drivenAxisDir = value; }
            public double Thickness { get => _thickness; set => _thickness = value; }
            public double Y { get => _y; set => _y = value; }
            public double Z { get => _z; set => _z = value; }
            public double V { get => _v; set => _v = value; }
            public List<Brep> GenevaModels { get => _genevaModels; set => _genevaModels = value; }
            public Vector3d TrajDir { get => _trajDir; set => _trajDir = value; }
            public Curve GenevaWheelCurve { get => _GenevaWheelCurve; set => _GenevaWheelCurve = value; }
            public Curve DrivePinCurve { get => _drivePinCurve; set => _drivePinCurve = value; }
            public Curve DriveWheelCurve { get => _driveWheelCurve; set => _driveWheelCurve = value; }
            public Curve SpacerHalveCurve { get => _spacerHalveCurve; set => _spacerHalveCurve = value; }
            public Curve SpacerHalveInnerCurve { get => _spacerHalveInnerCurve; set => _spacerHalveInnerCurve = value; }

            public GenevaDrive(Point3d drivenCen, int numSlot, Vector3d axisDir, double thickness, Vector3d motionDir)
            {
                _n = numSlot;
                _drivenCenPos = drivenCen;
                _drivenAxisDir = axisDir;
                _thickness = thickness;
                _trajDir = motionDir;

                // constant
                _b = 15.5;
                _p = 2;
                _t = 0.4;

                // calculation
                _c = _b / Math.Cos(180.0 / _n * Math.PI / 180);
                _a = Math.Sqrt(Math.Pow(_c, 2) - Math.Pow(_b, 2));
                _s = (_a + _b) - _c;
                _w = _p + 2 * _t;
                _y = _a - 1.5 * _p;
                _z = _y - _t;
                _v = _b * _z / _a;

                _myDoc = RhinoDoc.ActiveDoc;
                _genevaModels = new List<Brep>();

                GenerateGenevaDrive();
            }
            protected override void GenerateBaseCurve()
            {
                #region generate the driven wheel
                double angle = 360 / _n * Math.PI / 180;
                angle = angle / 2;

                // generate the circle
                Circle circle = new Circle(new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1)), new Point3d(0, 0, 0), _b);
                Curve circleCrv = circle.ToNurbsCurve();
                
                List<Curve> slotCrvs = new List<Curve>();
                List<Curve> stopCrvs = new List<Curve>();

                for (int i = 0; i<_n; i++)
                {
                    // generate the slot curves
                    Curve sideLn = new Line(new Point3d(-_w / 2, _b, 0), new Point3d(-_w / 2, _b - _s, 0)).ToNurbsCurve();
                    Curve btmCrv = new Arc(new Point3d(-_w / 2, _b - _s, 0), new Point3d(0, _b - _s - _w / 2, 0), new Point3d(_w / 2, _b - _s, 0)).ToNurbsCurve();
                    Curve sideLn1 = new Line(new Point3d(_w / 2, _b - _s, 0), new Point3d(_w / 2, _b, 0)).ToNurbsCurve();

                    Curve singleSlotCrv = Curve.JoinCurves(new List<Curve> { sideLn, btmCrv, sideLn1 }, _myDoc.ModelAbsoluteTolerance, false)[0];

                    Transform rot = Transform.Rotation(angle * 2 * i, new Vector3d(0, 0, 1), new Point3d(0, 0, 0));
                    singleSlotCrv.Transform(rot);

                    slotCrvs.Add(singleSlotCrv);

                    // generate the stop arc curves
                    Point3d cenOuter = new Point3d(_c * Math.Sin(angle), _c * Math.Cos(angle), 0);
                    cenOuter.Transform(rot);

                    Curve outerCircleCrv = new Circle(new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1)), cenOuter, _y).ToNurbsCurve();
                    stopCrvs.Add(outerCircleCrv);
                }

                // intersect the circle with the two lists and find those curve segments to construct the profile of the driven wheel
                List<double> intersectionParams = new List<double>();
                List<Curve> outArcs = new List<Curve>();
                List<Curve> slotArcs = new List<Curve>();

                List<Curve> slotGapCrvs = new List<Curve>();
                List<Curve> circleGapCrvs = new List<Curve>();

                foreach(Curve slotCrv in slotCrvs)
                {
                    var intersectionPts = Rhino.Geometry.Intersect.Intersection.CurveCurve(circleCrv, slotCrv, _myDoc.ModelAbsoluteTolerance, _myDoc.ModelAbsoluteTolerance);

                    List<double> tempCirclePramas = new List<double>();
                    List<double> slotArcParams = new List<double>();
                    foreach(var seg in intersectionPts)
                    {
                        intersectionParams.Add(seg.ParameterA);
                        tempCirclePramas.Add(seg.ParameterA);
                        slotArcParams.Add(seg.ParameterB);
                    }

                    var cirCandidates = circleCrv.Split(tempCirclePramas);
                    if (cirCandidates[0].GetLength() <= cirCandidates[1].GetLength())
                        slotGapCrvs.Add(cirCandidates[0]);
                    else
                        slotGapCrvs.Add(cirCandidates[1]);

                    var slotCandidates = slotCrv.Split(slotArcParams);
                    if (slotCandidates[0].GetLength() >= slotCandidates[1].GetLength() && slotCandidates[0].GetLength() >= slotCandidates[2].GetLength())
                        slotArcs.Add(slotCandidates[0]);
                    else if (slotCandidates[1].GetLength() >= slotCandidates[0].GetLength() && slotCandidates[1].GetLength() >= slotCandidates[2].GetLength())
                        slotArcs.Add(slotCandidates[1]);
                    else
                        slotArcs.Add(slotCandidates[2]);
                }

                foreach(Curve cirCrv in stopCrvs)
                {
                    var intersectionPts = Rhino.Geometry.Intersect.Intersection.CurveCurve(circleCrv, cirCrv, _myDoc.ModelAbsoluteTolerance, _myDoc.ModelAbsoluteTolerance);

                    List<double> outCircleParams = new List<double>();
                    List<double> tempCirclePramas = new List<double>();
                    foreach (var seg in intersectionPts)
                    {
                        intersectionParams.Add(seg.ParameterA);
                        outCircleParams.Add(seg.ParameterB);
                        tempCirclePramas.Add(seg.ParameterA);
                    }

                    var outArcCandidates = cirCrv.Split(outCircleParams);
                    if (outArcCandidates[0].GetLength() <= outArcCandidates[1].GetLength())
                        outArcs.Add(outArcCandidates[0]);
                    else
                        outArcs.Add(outArcCandidates[1]);

                    var cirCandidates = circleCrv.Split(tempCirclePramas);
                    if (cirCandidates[0].GetLength() <= cirCandidates[1].GetLength())
                        circleGapCrvs.Add(cirCandidates[0]);
                    else
                        circleGapCrvs.Add(cirCandidates[1]);
                }

                intersectionParams.Sort((a, b) => a.CompareTo(b)); // ascending order

                var circleSeg = circleCrv.Split(intersectionParams);

                List<Curve> circleLeftoverCrvs = new List<Curve>();

                foreach(Curve crv in circleSeg)
                {
                    Point3d crvPt1 = crv.PointAt(0);
                    Point3d crvPt2 = crv.PointAt(1);

                    bool isFind = false;

                    foreach(Curve crv1 in slotGapCrvs)
                    {
                        Point3d crvTestPt1 = crv1.PointAt(0);
                        Point3d crvTestPt2 = crv1.PointAt(1);

                        if (crvTestPt1.Equals(crvPt1) && crvTestPt2.Equals(crvPt2) ||
                            crvTestPt1.Equals(crvPt2) && crvTestPt2.Equals(crvPt1))
                        {
                            isFind = true;
                            break;
                        }
                    }

                    foreach (Curve crv1 in circleGapCrvs)
                    {
                        Point3d crvTestPt1 = crv1.PointAt(0);
                        Point3d crvTestPt2 = crv1.PointAt(1);

                        if (crvTestPt1.Equals(crvPt1) && crvTestPt2.Equals(crvPt2) ||
                            crvTestPt1.Equals(crvPt2) && crvTestPt2.Equals(crvPt1))
                        {
                            isFind = true;
                            break;
                        }
                    }

                    if (!isFind)
                        circleLeftoverCrvs.Add(crv);
                }

                List<Curve> dw_crvs = new List<Curve>();

                foreach (Curve c in slotArcs)
                {
                    dw_crvs.Add(c);
                }

                foreach (Curve c in outArcs)
                {
                    dw_crvs.Add(c);
                }
                foreach (Curve c in circleLeftoverCrvs)
                {
                    dw_crvs.Add(c);
                }

                _GenevaWheelCurve = Curve.JoinCurves(dw_crvs, _myDoc.ModelAbsoluteTolerance, false)[0];
                //_myDoc.Objects.AddCurve(drivenWheelCrv);
                //_myDoc.Views.Redraw();
                #endregion

                #region Generate the driving crank

                Point3d driveCen = new Point3d(-_c, 0, -_t - _thickness + 0.1);
                _driveWheelCurve = new Circle(new Plane(driveCen, new Vector3d(0, 0, 1)), driveCen, _a + _p).ToNurbsCurve();
                //Make sure pin is in slot

                Point3d pinCen = new Point3d(-_c, _a, -_t - _thickness + 0.1);
                _drivePinCurve = new Circle(new Plane(pinCen, new Vector3d(0, 0, 1)), pinCen, _p/2).ToNurbsCurve();
                Transform pinRot = Transform.Rotation(Math.PI / 2 - Math.Atan(_b / _a), new Vector3d(0, 0, -1), driveCen);
                _drivePinCurve.Transform(pinRot);

                #endregion

                #region generate the halve of spacer

                Curve spacerBridgeCrv = new Line(new Point3d(-_c, 3, 0), new Point3d(-_c, -3, 0)).ToNurbsCurve();
                Curve spacerArcCrv = new Arc(new Point3d(-_c, -3, 0), new Point3d(-_c - 3, 0, 0), new Point3d(-_c, 3, 0)).ToNurbsCurve();
                _spacerHalveCurve = Curve.JoinCurves(new List<Curve> { spacerBridgeCrv, spacerArcCrv }, _myDoc.ModelAbsoluteTolerance, false)[0];

                Curve spacerInnerBridgeCrv = new Line(new Point3d(-_c - 0.6, 2.12, 0), new Point3d(-_c - 0.6, -2.12, 0)).ToNurbsCurve();
                Curve spacerInnerArcCrv = new Arc(new Point3d(-_c - 0.6, -2.12, 0), new Point3d(-_c - 2.2, 0, 0), new Point3d(-_c - 0.6, 2.12, 0)).ToNurbsCurve();
                _spacerHalveInnerCurve = Curve.JoinCurves(new List<Curve> { spacerInnerBridgeCrv, spacerInnerArcCrv }, _myDoc.ModelAbsoluteTolerance, false)[0];

                #endregion
            }
            public void GenerateGenevaDrive()
            {
                GenerateBaseCurve();

                #region extrude all three parts separately

                var sweep = new SweepOneRail();
                sweep.AngleToleranceRadians = _myDoc.ModelAngleToleranceRadians;
                sweep.ClosedSweep = false;
                sweep.SweepTolerance = _myDoc.ModelAbsoluteTolerance;

                Point3d driveCen = new Point3d(-_c, 0, -_t - _thickness + 0.1);
                Point3d pinCen = new Point3d(-_c, _a, -_t - _thickness + 0.1);

                Curve genevaWheelPathCrv = new Line(new Point3d(0, 0, 0), new Point3d(0, 0, _thickness)).ToNurbsCurve();
                Curve driveWheelPathCrv = new Line(driveCen, new Point3d(driveCen.X, driveCen.Y, driveCen.Z + _thickness)).ToNurbsCurve();
                Curve pinPathCrv = new Line(pinCen, new Point3d(pinCen.X, pinCen.Y, pinCen.Z + 1.5 * _thickness + _t - 0.1)).ToNurbsCurve();
                Curve spacerPathCrv = new Line(new Point3d(0, 0, 0), new Point3d(0, 0, 1)).ToNurbsCurve();

                Brep[] spacerOuterBreps = sweep.PerformSweep(spacerPathCrv, _spacerHalveCurve);
                Brep spacerOuterBrep = spacerOuterBreps[0];
                Brep spacerOuterSolid = spacerOuterBrep.CapPlanarHoles(_myDoc.ModelAbsoluteTolerance);

                spacerOuterSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == spacerOuterSolid.SolidOrientation)
                    spacerOuterSolid.Flip();

                Brep[] spacerInnerBreps = sweep.PerformSweep(spacerPathCrv, _spacerHalveInnerCurve);
                Brep spacerInnerBrep = spacerInnerBreps[0];
                Brep spacerInnerSolid = spacerInnerBrep.CapPlanarHoles(_myDoc.ModelAbsoluteTolerance);

                spacerInnerSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == spacerInnerSolid.SolidOrientation)
                    spacerInnerSolid.Flip();

                Brep spacerSolid = Brep.CreateBooleanDifference(spacerOuterSolid, spacerInnerSolid, _myDoc.ModelAbsoluteTolerance)[0];
                spacerSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == spacerSolid.SolidOrientation)
                    spacerSolid.Flip();

                Brep[] genevaWheelBreps = sweep.PerformSweep(genevaWheelPathCrv, _GenevaWheelCurve);
                Brep genevaWheelBrep = genevaWheelBreps[0];
                Brep genevaWheelSolid = genevaWheelBrep.CapPlanarHoles(_myDoc.ModelAbsoluteTolerance);

                genevaWheelSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == genevaWheelSolid.SolidOrientation)
                    genevaWheelSolid.Flip();

                Brep[] driveWheelBreps = sweep.PerformSweep(driveWheelPathCrv, _driveWheelCurve);
                Brep driveWheelBrep = driveWheelBreps[0];
                Brep driveWheelSolid = driveWheelBrep.CapPlanarHoles(_myDoc.ModelAbsoluteTolerance);

                driveWheelSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == driveWheelSolid.SolidOrientation)
                    driveWheelSolid.Flip();

                Brep[] pinBreps = sweep.PerformSweep(pinPathCrv, _drivePinCurve);
                Brep pinBrep = pinBreps[0];
                Brep pinSolid = pinBrep.CapPlanarHoles(_myDoc.ModelAbsoluteTolerance);

                pinSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == pinSolid.SolidOrientation)
                    pinSolid.Flip();

                double r = 2;
           
                Brep holeBrep = Brep.CreatePipe(driveWheelPathCrv, r + _t, false, PipeCapMode.Round, true, _myDoc.ModelAbsoluteTolerance, _myDoc.ModelAngleToleranceRadians)[0];

                holeBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == holeBrep.SolidOrientation)
                    holeBrep.Flip();

                driveWheelSolid = Brep.CreateBooleanDifference(driveWheelSolid, holeBrep, _myDoc.ModelAbsoluteTolerance)[0];
                driveWheelSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == driveWheelSolid.SolidOrientation)
                    driveWheelSolid.Flip();

                #endregion

                #region transform (tranlsation, rotation, self-rotation)

                //First rotate driven wheel for a better starting position
                Transform drivenWheelSelfRotate = Transform.Rotation(Math.PI*(0.5+1.0/_n), new Vector3d(0, 0, 1), Point3d.Origin);
                genevaWheelSolid.Transform(drivenWheelSelfRotate);

                Transform move = Transform.Translation(new Vector3d(_drivenCenPos));
                Transform rotate = Transform.Rotation(new Vector3d(0, 0, 1), _drivenAxisDir, _drivenCenPos);
                Vector3d originalTrajDir = new Vector3d(1, 0, 0);
                originalTrajDir.Transform(move);
                originalTrajDir.Transform(rotate);
                //Xia's note: the previous self Rotate is not along a specific rotation axis, it might cause error.
                double angle = Vector3d.VectorAngle(originalTrajDir, _trajDir);
                Transform selfRotate = Transform.Rotation(angle,DrivenAxisDir, _drivenCenPos);
                //Transform selfRotate = Transform.Rotation(originalTrajDir, _trajDir,_drivenCenPos);

                genevaWheelSolid.Transform(move);
                genevaWheelSolid.Transform(rotate);
                genevaWheelSolid.Transform(selfRotate);

                driveWheelSolid.Transform(move);
                driveWheelSolid.Transform(rotate);
                driveWheelSolid.Transform(selfRotate);

                pinSolid.Transform(move);
                pinSolid.Transform(rotate);
                pinSolid.Transform(selfRotate);

                spacerSolid.Transform(move);
                spacerSolid.Transform(rotate);
                spacerSolid.Transform(selfRotate);

                driveCen.Transform(move);
                driveCen.Transform(rotate);
                driveCen.Transform(selfRotate);

                #endregion

                _genevaModels.Add(genevaWheelSolid);
                _genevaModels.Add(driveWheelSolid);
                _genevaModels.Add(pinSolid);
                _genevaModels.Add(spacerSolid);

                drivenWheel = new GenevaDrivenWheel(genevaWheelSolid, _drivenCenPos, _drivenAxisDir, _n, 0);
                drivingWheel = new DrivingWheel(pinSolid, driveCen, _drivenAxisDir, _trajDir, _a,0);
            }
        }
    }
    
}
