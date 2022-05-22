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
        class CrankSlottedLever : Component
        {
            private RhinoDoc _mydoc;

            private Point3d _wheelCen;
            private Vector3d _wheelAxisDir;
            private Vector3d _oscillationAxisCen;
            private double _thickness; // the thickness of the wheel
            private double _p; // pin diameter
            private double _leverThickness;
            private double _leverLen;
            private double _leverWidth;
            private double _t; // clearance
            private double _slotLen;
            private double _wheelRadius;
            private double _crankRadius;
            private double _anchorDis; // _anchorDis = 1.5 * _wheelRadius
            private double _anchorRadius;
            private double _anchorLeverClearance;

            private Curve _wheelCrv;
            private Curve _pinCrv;
            private Curve _anchorCrv;
            private Curve _slottedBarCrv;
            private Curve _slotCrv;
            private Curve _stopWallCrv;

            private Point3d anchorCenter;
            private List<Brep> _crankSlottedLeverModels;
            public Brep leverSweepSpace=null;
            public Brep eeSweepSpace=null;
            public CrankSlottedLever(Point3d pos, Vector3d axisDir, Vector3d dir, double wheelRadius, double connectorLen)
            {
                _wheelCen = pos;
                _wheelAxisDir = axisDir;
                _oscillationAxisCen = dir;
                _wheelRadius = wheelRadius;

                // constant
                _mydoc = RhinoDoc.ActiveDoc;
                _thickness = 3.6;
                _p = 2;
                _leverThickness = 2;
                _t = 0.5;
                _anchorRadius = 2.0;
                _anchorLeverClearance = 1.2;

                // calculated
                _leverLen = connectorLen + 1.5 * _wheelRadius + _anchorRadius + _anchorLeverClearance;
                _anchorDis = 1.5 * _wheelRadius;
                _crankRadius = _wheelRadius - _p;
                _leverWidth = _anchorRadius * 2 + 2 * _anchorLeverClearance;
                _slotLen = 2 * _crankRadius;
                _crankSlottedLeverModels = new List<Brep>();

                GenerateCrankSlottedLever();
            }

            protected override void GenerateBaseCurve()
            {
                #region generate the wheel curve

                _wheelCrv = new Circle(new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1)), new Point3d(0, 0, 0), _wheelRadius).ToNurbsCurve();

                #endregion

                #region generate the pin curve

                _pinCrv = new Circle(new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1)), new Point3d(_crankRadius, 0, 0), _p / 2).ToNurbsCurve();

                #endregion

                #region generate the anchor curve

                _anchorCrv = new Circle(new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1)), new Point3d(-_anchorDis, 0, 0), _anchorRadius).ToNurbsCurve();
                anchorCenter = new Circle(new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1)), new Point3d(-_anchorDis, 0, 0), _anchorRadius).Center;
                Transform trans = Transform.Translation(new Vector3d(0, 0, _thickness));
                Transform trans1 = Transform.Translation(new Vector3d(0, 0, _thickness + 2.2));
                _anchorCrv.Transform(trans);
                anchorCenter.Transform(trans);

                #endregion

                #region generate the slotted bar curve
                List<Curve> slottedLeverProfileCrvs = new List<Curve>();
                Curve leverBtmLn = new Line(new Point3d(-1.5 * _wheelRadius, -_anchorRadius - _anchorLeverClearance, 0), new Point3d(-1.5 * _wheelRadius + _leverLen - _anchorRadius - _anchorLeverClearance, -_anchorRadius - _anchorLeverClearance, 0)).ToNurbsCurve();
                Curve leverRightCrv = new Arc(new Point3d(-1.5 * _wheelRadius + _leverLen - _anchorRadius - _anchorLeverClearance, -_anchorRadius - _anchorLeverClearance, 0), new Point3d(-1.5 * _wheelRadius + _leverLen, 0, 0), new Point3d(-1.5 * _wheelRadius + _leverLen - _anchorRadius - _anchorLeverClearance, _anchorRadius + _anchorLeverClearance, 0)).ToNurbsCurve();
                Curve leverTopLn = new Line(new Point3d(-1.5 * _wheelRadius + _leverLen - _anchorRadius - _anchorLeverClearance, _anchorRadius + _anchorLeverClearance, 0), new Point3d(-1.5 * _wheelRadius, _anchorRadius + _anchorLeverClearance, 0)).ToNurbsCurve();
                Curve leverLeftCrv = new Arc(new Point3d(-1.5 * _wheelRadius, _anchorRadius + _anchorLeverClearance, 0), new Point3d(-1.5 * _wheelRadius - _anchorRadius - _anchorLeverClearance, 0, 0), new Point3d(-1.5 * _wheelRadius, -_anchorRadius - _anchorLeverClearance, 0)).ToNurbsCurve();
                slottedLeverProfileCrvs.Add(leverBtmLn);
                slottedLeverProfileCrvs.Add(leverRightCrv);
                slottedLeverProfileCrvs.Add(leverTopLn);
                slottedLeverProfileCrvs.Add(leverLeftCrv);

                _slottedBarCrv = Curve.JoinCurves(slottedLeverProfileCrvs, _mydoc.ModelAbsoluteTolerance, false)[0];
                _slottedBarCrv.Transform(trans1);

                #endregion

                #region generate the slot curve

                List<Curve> slotProfileCrvs = new List<Curve>();
                Curve slotBtmLn = new Line(new Point3d(-_crankRadius, -_p / 2 - _t, 0), new Point3d(_crankRadius, -_p / 2 - _t, 0)).ToNurbsCurve();
                Curve slotRightCrv = new Arc(new Point3d(_crankRadius, -_p / 2 - _t, 0), new Point3d(_crankRadius + _p / 2 + _t, 0, 0), new Point3d(_crankRadius, _p / 2 + _t, 0)).ToNurbsCurve();
                Curve slotTopLn = new Line(new Point3d(_crankRadius, _p / 2 + _t, 0), new Point3d(-_crankRadius, _p / 2 + _t, 0)).ToNurbsCurve();
                Curve slotLeftCrv = new Arc(new Point3d(-_crankRadius, _p / 2 + _t, 0), new Point3d(-_crankRadius - _p / 2 - _t, 0, 0), new Point3d(-_crankRadius, -_p / 2 - _t, 0)).ToNurbsCurve();
                slotProfileCrvs.Add(slotBtmLn);
                slotProfileCrvs.Add(slotRightCrv);
                slotProfileCrvs.Add(slotTopLn);
                slotProfileCrvs.Add(slotLeftCrv);

                _slotCrv = Curve.JoinCurves(slotProfileCrvs, _mydoc.ModelAbsoluteTolerance, false)[0];
                _slotCrv.Transform(trans);

                #endregion

            }

            public void GenerateCrankSlottedLever()
            {
                GenerateBaseCurve();

                // generate breps

                var sweep = new SweepOneRail();
                sweep.AngleToleranceRadians = _mydoc.ModelAngleToleranceRadians;
                sweep.ClosedSweep = false;
                sweep.SweepTolerance = _mydoc.ModelAbsoluteTolerance;

                Curve wheelPathCrv = new Line(new Point3d(0, 0, 0), new Point3d(0, 0, _thickness)).ToNurbsCurve();
                Curve pinPathCrv = new Line(new Point3d(0, 0, 0), new Point3d(0, 0, _thickness + 2.2 + _leverThickness + _t - 0.2 + _t / 2)).ToNurbsCurve();
                Curve anchorPathCrv = new Line(new Point3d(-_anchorDis, 0, 0), new Point3d(-_anchorDis, 0, _thickness + 2.2)).ToNurbsCurve();
                Transform trans = Transform.Translation(new Vector3d(0, 0, _thickness));
                anchorPathCrv.Transform(trans);
                Curve slottedLeverPathCrv = new Line(new Point3d(0, 0, _thickness + 2.2), new Point3d(0, 0, _thickness + 2.2 + _leverThickness)).ToNurbsCurve();
                Curve slotPathCrv = new Line(new Point3d(0, 0, _thickness), new Point3d(0, 0, _thickness + 2.2 + _leverThickness + _t)).ToNurbsCurve();
                _stopWallCrv = new Line(new Point3d(0, 0, 2.2 + _thickness + _leverThickness + _t - 0.2), new Point3d(0, 0, 2.2 + _thickness + _leverThickness + _t - 0.2 + 3 * _t)).ToNurbsCurve();
                Curve stopWallGrooveCrv = new Line(new Point3d(0, 0, 2.2 + _thickness + _leverThickness + _t - 0.2), new Point3d(0, 0, 2.2 + _thickness + _leverThickness + _t - 0.2 + 1.5 * _t)).ToNurbsCurve();

                // generate the stop wall
                double wallOuterRadius = _wheelRadius;
                double wallGrooveRadius = _crankRadius + _p / 2 + _t;
                double wallInnerRadius = _crankRadius - _p / 2 - _t;

                Brep wallOuterSolid = Brep.CreatePipe(_stopWallCrv, wallOuterRadius, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];
                wallOuterSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == wallOuterSolid.SolidOrientation)
                    wallOuterSolid.Flip();

                Brep wallGrooveSolid = Brep.CreatePipe(stopWallGrooveCrv, wallGrooveRadius, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];
                wallGrooveSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == wallGrooveSolid.SolidOrientation)
                    wallGrooveSolid.Flip();

                Brep wallInnerSolid = Brep.CreatePipe(_stopWallCrv, wallInnerRadius, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];
                wallInnerSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == wallInnerSolid.SolidOrientation)
                    wallInnerSolid.Flip();

                Brep wallHalfSolid1 = Brep.CreateBooleanDifference(wallOuterSolid, wallGrooveSolid, _mydoc.ModelAbsoluteTolerance)[0];
                wallHalfSolid1.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == wallHalfSolid1.SolidOrientation)
                    wallHalfSolid1.Flip();

                Brep stopWallSolid = Brep.CreateBooleanUnion(new List<Brep> { wallHalfSolid1, wallInnerSolid }, _mydoc.ModelAbsoluteTolerance)[0];
                stopWallSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == stopWallSolid.SolidOrientation)
                    stopWallSolid.Flip();

                Brep[] wheelBreps = sweep.PerformSweep(wheelPathCrv, _wheelCrv);
                Brep wheelBrep = wheelBreps[0];
                Brep wheelSolid = wheelBrep.CapPlanarHoles(_mydoc.ModelAbsoluteTolerance);
                wheelSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == wheelSolid.SolidOrientation)
                    wheelSolid.Flip();

                double shaftRadius = 2.6;
                Brep shaft = Brep.CreatePipe(wheelPathCrv, shaftRadius, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];
                shaft.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == shaft.SolidOrientation)
                    shaft.Flip();

                Brep wheelCrankSolid = Brep.CreateBooleanDifference(wheelSolid, shaft, _mydoc.ModelAbsoluteTolerance)[0];
                wheelCrankSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == wheelCrankSolid.SolidOrientation)
                    wheelCrankSolid.Flip();

                Brep[] pinBreps = sweep.PerformSweep(pinPathCrv, _pinCrv);
                Brep pinBrep = pinBreps[0];
                Brep pinSolid = pinBrep.CapPlanarHoles(_mydoc.ModelAbsoluteTolerance);
                pinSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == pinSolid.SolidOrientation)
                    pinSolid.Flip();

                Brep[] anchorBreps = sweep.PerformSweep(anchorPathCrv, _anchorCrv);
                Brep anchorBrep = anchorBreps[0];
                Brep anchorSolid = anchorBrep.CapPlanarHoles(_mydoc.ModelAbsoluteTolerance);
                anchorSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == anchorSolid.SolidOrientation)
                    anchorSolid.Flip();

                Brep[] slottedLeverOuterBreps = sweep.PerformSweep(slottedLeverPathCrv, _slottedBarCrv);
                Brep slottedLeverOuterBrep = slottedLeverOuterBreps[0];
                Brep slottedLeverOuterSoild = slottedLeverOuterBrep.CapPlanarHoles(_mydoc.ModelAbsoluteTolerance);
                slottedLeverOuterSoild.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == slottedLeverOuterSoild.SolidOrientation)
                    slottedLeverOuterSoild.Flip();

                Curve leftCircleAxis = new Line(new Point3d(-_anchorDis, 0, _thickness + 2.2), new Point3d(-_anchorDis, 0, _thickness + 2.2 + _leverThickness)).ToNurbsCurve();
                Curve rightCircleAxis = new Line(new Point3d(_leverLen - 1.5 * _wheelRadius - _anchorRadius - _anchorLeverClearance, 0, _thickness + 2.2), new Point3d(_leverLen - 1.5 * _wheelRadius - _anchorRadius - _anchorLeverClearance, 0, _thickness + 2.2 + _leverThickness)).ToNurbsCurve();
                Brep leftCircleSolid = Brep.CreatePipe(leftCircleAxis, _anchorRadius + _anchorLeverClearance, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];
                leftCircleSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == leftCircleSolid.SolidOrientation)
                    leftCircleSolid.Flip();
                Brep rightCircleSolid = Brep.CreatePipe(rightCircleAxis, _anchorRadius + _anchorLeverClearance, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];
                rightCircleSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == rightCircleSolid.SolidOrientation)
                    rightCircleSolid.Flip();

                slottedLeverOuterSoild = Brep.CreateBooleanUnion(new List<Brep>{ slottedLeverOuterSoild, leftCircleSolid, rightCircleSolid}, _mydoc.ModelAbsoluteTolerance)[0];
                slottedLeverOuterSoild.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == slottedLeverOuterSoild.SolidOrientation)
                    slottedLeverOuterSoild.Flip();


                Brep[] slotBreps = sweep.PerformSweep(slotPathCrv, _slotCrv);
                Brep slotBrep = slotBreps[0];
                Brep slotSolid = slotBrep.CapPlanarHoles(_mydoc.ModelAbsoluteTolerance);
                slotSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == slotSolid.SolidOrientation)
                    slotSolid.Flip();

                Brep slotIntermediate = Brep.CreateBooleanDifference(slottedLeverOuterSoild, slotSolid, _mydoc.ModelAbsoluteTolerance)[0];
                slotIntermediate.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == slotIntermediate.SolidOrientation)
                    slotIntermediate.Flip();
                Brep slottedLeverSolid = Brep.CreateBooleanDifference(slotIntermediate, anchorSolid, _mydoc.ModelAbsoluteTolerance)[0];

                //Brep slottedLeverSolid = Brep.CreateBooleanDifference(new List<Brep> { slottedLeverOuterSoild }, new List<Brep> { anchorSolid, slotSolid }, _mydoc.ModelAbsoluteTolerance)[0];
                slottedLeverSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == slottedLeverSolid.SolidOrientation)
                    slottedLeverSolid.Flip();


                // translate, rotate and self-rotate
                Transform move = Transform.Translation(new Vector3d(_wheelCen));
                Transform rotate = Transform.Rotation(new Vector3d(0, 0, 1), _wheelAxisDir, _wheelCen);
                Vector3d originalMotionDir = new Vector3d(1, 0, 0);
                originalMotionDir.Transform(move);
                originalMotionDir.Transform(rotate);
                Transform selfRotate = Transform.Rotation(originalMotionDir, _oscillationAxisCen, _wheelCen);

                wheelCrankSolid.Transform(move);
                wheelCrankSolid.Transform(rotate);
                wheelCrankSolid.Transform(selfRotate);

                pinSolid.Transform(move);
                pinSolid.Transform(rotate);
                pinSolid.Transform(selfRotate);

                slottedLeverSolid.Transform(move);
                slottedLeverSolid.Transform(rotate);
                slottedLeverSolid.Transform(selfRotate);

                stopWallSolid.Transform(move);
                stopWallSolid.Transform(rotate);
                stopWallSolid.Transform(selfRotate);

                anchorCenter.Transform(move);
                anchorCenter.Transform(rotate);
                anchorCenter.Transform(selfRotate);

                _crankSlottedLeverModels.Add(wheelCrankSolid);
                _crankSlottedLeverModels.Add(pinSolid);
                _crankSlottedLeverModels.Add(slottedLeverSolid);
                _crankSlottedLeverModels.Add(stopWallSolid);
            }
            public void AddEndEffector(Brep ee)
            {
                //First create lever sweep trajectory, which is a triangle
                Point3d anchorDirPoint1 = anchorCenter + _oscillationAxisCen * LeverLen;
                Point3d anchorDirPoint2 = anchorCenter + _oscillationAxisCen * LeverLen;
                double angle =Math.Asin(_crankRadius/(WheelRadius*1.5));
                anchorDirPoint1.Transform(Transform.Rotation(angle+0.1, WheelAxisDir, anchorCenter));
                anchorDirPoint2.Transform(Transform.Rotation(-angle-0.1, WheelAxisDir, anchorCenter));
                List<Point3d> sectionLinePts = new List<Point3d> { anchorCenter,anchorDirPoint1,anchorDirPoint2, anchorCenter, };
                Polyline section = new Polyline(sectionLinePts);
                Curve sectionCurve = section.ToNurbsCurve();
                //RhinoDoc.ActiveDoc.Objects.AddCurve(sectionCurve);
                //RhinoDoc.ActiveDoc.Views.Redraw();
                var sweep = new SweepOneRail();
                sweep.AngleToleranceRadians = RhinoDoc.ActiveDoc.ModelAngleToleranceRadians;
                sweep.ClosedSweep = false;
                sweep.SweepTolerance = RhinoDoc.ActiveDoc.ModelAbsoluteTolerance;
                Curve rail1 = new Line(anchorCenter, anchorCenter+WheelAxisDir*(2+0.6*2)).ToNurbsCurve();
                Brep[] leverSweepList = sweep.PerformSweep(rail1, sectionCurve);
                //RhinoDoc.ActiveDoc.Objects.AddBrep(leverSweepList[0]);
                //RhinoDoc.ActiveDoc.Views.Redraw();
                Brep leverSweep = leverSweepList[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                leverSweep.Transform(Transform.Translation(WheelAxisDir * (1.6)));
                leverSweepSpace = leverSweep;
                leverSweepSpace.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == leverSweepSpace.SolidOrientation)
                    leverSweepSpace.Flip();
                //Sweep ee model by rotating 5 degree once
                Brep eeSweepResult = ee.DuplicateBrep();
                eeSweepResult.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == eeSweepResult.SolidOrientation)
                    eeSweepResult.Flip();
                for (int i=1;i<=10;i++)
                {
                    Brep ee1= ee.DuplicateBrep();
                    ee1.Transform(Transform.Rotation(i*angle / 10, WheelAxisDir, anchorCenter));
                    Brep ee2 = ee.DuplicateBrep();
                    ee2.Transform(Transform.Rotation(-i * angle / 10, WheelAxisDir, anchorCenter));
                    try
                    {
                        var result = Brep.CreateBooleanUnion(new List<Brep> { eeSweepResult, ee1, ee2 }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                        eeSweepResult = result[0];
                        eeSweepResult.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                        if (BrepSolidOrientation.Inward == eeSweepResult.SolidOrientation)
                            eeSweepResult.Flip();
                    }
                    catch { }
                    //RhinoDoc.ActiveDoc.Objects.AddBrep(eeSweepResult);
                    //RhinoDoc.ActiveDoc.Views.Redraw();
                }
                eeSweepSpace = eeSweepResult;
                //eeSweepSpace = Brep.CreateOffsetBrep(eeSweepResult, 0.6, false, false, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out _)[0];
                //eeSweepSpace.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                //if (BrepSolidOrientation.Inward == eeSweepSpace.SolidOrientation)
                //    eeSweepSpace.Flip();
            }
            public Point3d WheelCen { get => _wheelCen; set => _wheelCen = value; }
            public Vector3d WheelAxisDir { get => _wheelAxisDir; set => _wheelAxisDir = value; }
            public Vector3d OscillationAxisCen { get => _oscillationAxisCen; set => _oscillationAxisCen = value; }
            public double Thickness { get => _thickness; set => _thickness = value; }
            public double P { get => _p; set => _p = value; }
            public double LeverThickness { get => _leverThickness; set => _leverThickness = value; }
            public double LeverLen { get => _leverLen; set => _leverLen = value; }
            public double LeverLen1 { get => _leverLen; set => _leverLen = value; }
            public double T { get => _t; set => _t = value; }
            public double SlotLen { get => _slotLen; set => _slotLen = value; }
            public double WheelRadius { get => _wheelRadius; set => _wheelRadius = value; }
            public double CrankRadius { get => _crankRadius; set => _crankRadius = value; }
            public double AnchorDis { get => _anchorDis; set => _anchorDis = value; }
            public Curve WheelCrv { get => _wheelCrv; set => _wheelCrv = value; }
            public Curve PinCrv { get => _pinCrv; set => _pinCrv = value; }
            public Curve AnchorCrv { get => _anchorCrv; set => _anchorCrv = value; }
            public Curve SlottedBarCrv { get => _slottedBarCrv; set => _slottedBarCrv = value; }
            public Curve SlotCrv { get => _slotCrv; set => _slotCrv = value; }
            public double LeverWidth { get => _leverWidth; set => _leverWidth = value; }
            public List<Brep> CrankSlottedLeverModels { get => _crankSlottedLeverModels; set => _crankSlottedLeverModels = value; }
            public double AnchorRadius { get => _anchorRadius; set => _anchorRadius = value; }
            public double AnchorLeverClearance { get => _anchorLeverClearance; set => _anchorLeverClearance = value; }
            public Curve StopWallCrv { get => _stopWallCrv; set => _stopWallCrv = value; }
            public Point3d AnchorCenter { get => anchorCenter;}
        }
    }
}