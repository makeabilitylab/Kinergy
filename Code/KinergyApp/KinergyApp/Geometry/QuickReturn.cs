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
        class QuickReturn: Component
        {
            // More simplified quick-return mechanism: https://qxf2.com/blog/wp-content/uploads/2017/05/Skotch_yoke.png

            private double _amplitude; // 2 * the rotation radius (the distance from the pin center to the crank center) 
            private double _crankRadius; // the radius of the crank (the wheel)
            private double _p; // pin diameter
            private double _t; // clearance
            private Point3d _startPt; // the wheeling center
            private Vector3d _axisDir; // rotary axis of the wheel
            private Vector3d _trajDir; // reciprocationg direction
            private double _sliderRadius; // slider radius
            private double _crankLen; // the distance between the yoke and the bearing block
            private double _blockLen; // the length of the bearing block
            private double _connectorLen; // the length between the block and the end-effector
            private double _thickness; // the thickness of everything
            private double _yokeWidth; // the width of the yoke bar

            private RhinoDoc _mydoc;
            private List<Brep> _quickReturnModels;
            private Curve _bearingBlockCrv;
            private Curve _yokeCrv;
            private Curve _yokeOuterCrv;
            private Curve _pinCrv;
            private Curve _crankCrv;
            private Curve _sliderCrv;
            private Curve _stopWallCrv;

            Brep crankWheelSolid;
            Brep pinSolid;
            Brep yokeSolid;
            Brep sliderSolid;
            Brep bearingBlockSolid;
            Brep stopWallSolid;

            public double Amplitude { get => _amplitude; set => _amplitude = value; }
            public double CrankRadius { get => _crankRadius; set => _crankRadius = value; }
            public double P { get => _p; set => _p = value; }
            public double T { get => _t; set => _t = value; }
            public Point3d StartPt { get => _startPt; set => _startPt = value; }
            public Vector3d AxisDir { get => _axisDir; set => _axisDir = value; }
            public Vector3d TrajDir { get => _trajDir; set => _trajDir = value; }
            public double SliderRadius { get => _sliderRadius; set => _sliderRadius = value; }
            public double CrankLen { get => _crankLen; set => _crankLen = value; }
            public double BlockLen { get => _blockLen; set => _blockLen = value; }
            public double ConnectorLen { get => _connectorLen; set => _connectorLen = value; }
            public List<Brep> QuickReturnModels { get => _quickReturnModels; set => _quickReturnModels = value; }
            public Curve BearingBlockCrv { get => _bearingBlockCrv; set => _bearingBlockCrv = value; }
            public Curve YokeCrv { get => _yokeCrv; set => _yokeCrv = value; }
            public Curve PinCrv { get => _pinCrv; set => _pinCrv = value; }
            public Curve CrankCrv { get => _crankCrv; set => _crankCrv = value; }
            public double Thickness { get => _thickness; set => _thickness = value; }
            public double YokeWidth { get => _yokeWidth; set => _yokeWidth = value; }
            public Curve SliderCrv { get => _sliderCrv; set => _sliderCrv = value; }
            public Curve YokeOuterCrv { get => _yokeOuterCrv; set => _yokeOuterCrv = value; }
            public Curve StopWallCrv { get => _stopWallCrv; set => _stopWallCrv = value; }
            public Brep CrankWheelSolid { get => crankWheelSolid; }
            public Brep PinSolid { get => pinSolid;}
            public Brep YokeSolid { get => yokeSolid;}
            public Brep SliderSolid { get => sliderSolid; }
            public Brep BearingBlockSolid { get => bearingBlockSolid;}
            public Brep StopWallSolid { get => stopWallSolid;}

            public QuickReturn(Point3d startPt, Vector3d axisDir, Vector3d motionDir, double amplitude, double thickness, double sliderLen)
            {
                _startPt = startPt;
                _axisDir = axisDir;
                _trajDir = motionDir;
                _amplitude = amplitude;
                _thickness = thickness;

                // constant
                _p = 2;
                _t = 0.5;
                _sliderRadius = 1.5;
                _yokeWidth = 1;

                // calculate
                _blockLen = 4 * _sliderRadius;
                _crankRadius = _amplitude / 2 + _p / 2 + _t + _yokeWidth + _t;
                _crankLen = _yokeWidth + _t + _p;
                _connectorLen = sliderLen - _blockLen - _amplitude / 2 - _p / 2 - _t - _crankLen; // sliderLen = _connectorLen + _p + _crankRadius + _blockLen

                // others
                _quickReturnModels = new List<Brep>();
                _mydoc = RhinoDoc.ActiveDoc;

                GenerateQuickReturn();

            }

            protected override void GenerateBaseCurve()
            {
                #region generate the wheel crank curve

                _crankCrv = new Circle(new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1)), new Point3d(0, 0, 0), _crankRadius).ToNurbsCurve();

                #endregion

                #region generate the pin curve

                _pinCrv = new Circle(new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1)), new Point3d(-_amplitude/2, 0, 0), _p/2).ToNurbsCurve();

                #endregion

                #region generete the yoke curve

                List<Curve> yokeCrvs = new List<Curve>();
                Curve rightSideLn = new Line(new Point3d(-_amplitude / 2 + _p / 2 + _t, _amplitude / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t, -_amplitude / 2, 0)).ToNurbsCurve();
                Curve bottomHalfCircle = new Arc(new Point3d(-_amplitude / 2 + _p / 2 + _t, -_amplitude / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t) / 2, -_amplitude / 2 - (_p + 2 * _t) / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t), -_amplitude / 2, 0)).ToNurbsCurve();
                Curve leftSideLn = new Line(new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t), -_amplitude / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t), _amplitude / 2, 0)).ToNurbsCurve();
                Curve topHalfCircle = new Arc(new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t), _amplitude / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t) / 2, _amplitude / 2 + (_p + 2 * _t) / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t, _amplitude / 2, 0)).ToNurbsCurve();
                yokeCrvs.Add(rightSideLn);
                yokeCrvs.Add(bottomHalfCircle);
                yokeCrvs.Add(leftSideLn);
                yokeCrvs.Add(topHalfCircle);

                _yokeCrv = Curve.JoinCurves(yokeCrvs, _mydoc.ModelAbsoluteTolerance, false)[0];

                Transform yokeMove = Transform.Translation(new Vector3d(0, 0, 2.2 + _thickness));
                _yokeCrv.Transform(yokeMove);

                List<Curve> yokeOuterCrvs = new List<Curve>();
                Curve rightSideLnOuter = new Line(new Point3d(-_amplitude / 2 + _p / 2 + _t + _yokeWidth, _amplitude / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t + _yokeWidth, -_amplitude / 2, 0)).ToNurbsCurve();
                Curve bottomHalfCircleOuter = new Arc(new Point3d(-_amplitude / 2 + _p / 2 + _t + _yokeWidth, -_amplitude / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t) / 2, -_amplitude / 2 - (_p + 2 * _t) / 2 - _yokeWidth, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t) - _yokeWidth, -_amplitude / 2, 0)).ToNurbsCurve();
                Curve leftSideLnOuter = new Line(new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t) - _yokeWidth, -_amplitude / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t) - _yokeWidth, _amplitude / 2, 0)).ToNurbsCurve();
                Curve topHalfCircleOuter = new Arc(new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t) - _yokeWidth, _amplitude / 2, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t - (_p + 2 * _t) / 2, _amplitude / 2 + (_p + 2 * _t) / 2 + _yokeWidth, 0), new Point3d(-_amplitude / 2 + _p / 2 + _t + _yokeWidth, _amplitude / 2, 0)).ToNurbsCurve();
                yokeOuterCrvs.Add(rightSideLnOuter);
                yokeOuterCrvs.Add(bottomHalfCircleOuter);
                yokeOuterCrvs.Add(leftSideLnOuter);
                yokeOuterCrvs.Add(topHalfCircleOuter);

                _yokeOuterCrv = Curve.JoinCurves(yokeOuterCrvs, _mydoc.ModelAbsoluteTolerance, false)[0];
                _yokeOuterCrv.Transform(yokeMove);

                #endregion

                #region generate the axis of the slider

                double sliderLen = _connectorLen + _p + _crankRadius + _blockLen;
                _sliderCrv = new Line(new Point3d(-_amplitude / 2 + _p / 2 + _t, 0, 0), new Point3d(sliderLen + (-_amplitude / 2 + _p / 2 + _t), 0, 0)).ToNurbsCurve();
                Transform slidingMove = Transform.Translation(new Vector3d(0, 0, 2.2 + _thickness + _sliderRadius));
                _sliderCrv.Transform(slidingMove);

                #endregion

                #region generate the bearing block axis curve

                _bearingBlockCrv = new Line(new Point3d(_crankRadius + _p, 0, 0), new Point3d(_crankRadius + _p + _blockLen, 0, 0)).ToNurbsCurve();
                _bearingBlockCrv.Transform(slidingMove);

                #endregion
            }

            public void GenerateQuickReturn()
            {
                GenerateBaseCurve();

                #region create the breps using those curves

                var sweep = new SweepOneRail();
                sweep.AngleToleranceRadians = _mydoc.ModelAngleToleranceRadians;
                sweep.ClosedSweep = false;
                sweep.SweepTolerance = _mydoc.ModelAbsoluteTolerance;

                Curve crankWheelPathCrv = new Line(new Point3d(0, 0, 0), new Point3d(0, 0, _thickness)).ToNurbsCurve();
                Curve pinPathCrv = new Line(new Point3d(0, 0, 0), new Point3d(0, 0, 2.2 + _thickness +  2 * SliderRadius + _t - 0.2 + _t/2)).ToNurbsCurve();
                Curve yokePathCrv = new Line(new Point3d(0, 0, 2.2 + _thickness), new Point3d(0, 0, 2.2 + _thickness + 2 * SliderRadius)).ToNurbsCurve();
                Curve yokePathInnerCrv = new Line(new Point3d(0, 0, 2.2 + _thickness-_t), new Point3d(0, 0, 2.2 + _thickness + 2 * SliderRadius +_t)).ToNurbsCurve();
                _stopWallCrv = new Line(new Point3d(0, 0, 2.2 + _thickness + 2 * SliderRadius + _t - 0.2), new Point3d(0, 0, 2.2 + _thickness + 2 * SliderRadius + _t - 0.2 + 3 * _t)).ToNurbsCurve();
                Curve stopWallGrooveCrv = new Line(new Point3d(0, 0, 2.2 + _thickness + 2 * SliderRadius + _t - 0.2), new Point3d(0, 0, 2.2 + _thickness + 2 * SliderRadius + _t - 0.2 + 1.5 * _t)).ToNurbsCurve();

                // generate the stop wall

                double wallOuterRadius = _amplitude / 2 + _p / 2 + _t + _p;
                double wallGrooveRadius = _amplitude / 2 + _p / 2 + _t;
                double wallInnerRadius = _amplitude / 2 - _p / 2 - _t;

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

                stopWallSolid = Brep.CreateBooleanUnion(new List<Brep> { wallHalfSolid1, wallInnerSolid }, _mydoc.ModelAbsoluteTolerance)[0];
                stopWallSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == stopWallSolid.SolidOrientation)
                    stopWallSolid.Flip();

                Brep[] crankWheelBreps = sweep.PerformSweep(crankWheelPathCrv, _crankCrv);
                Brep crankWheelBrep = crankWheelBreps[0];
                Brep crankWheel = crankWheelBrep.CapPlanarHoles(_mydoc.ModelAbsoluteTolerance);

                crankWheel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == crankWheel.SolidOrientation)
                    crankWheel.Flip();

                double shaftRadius = 1.5;
                Brep crankHoleSolid = Brep.CreatePipe(crankWheelPathCrv, shaftRadius + _t, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];

                crankHoleSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == crankHoleSolid.SolidOrientation)
                    crankHoleSolid.Flip();

                crankWheelSolid = Brep.CreateBooleanDifference(crankWheel, crankHoleSolid, _mydoc.ModelAbsoluteTolerance)[0];
                crankWheelSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == crankWheelSolid.SolidOrientation)
                    crankWheelSolid.Flip();


                Brep[] pinBreps = sweep.PerformSweep(pinPathCrv, _pinCrv);
                Brep pinBrep = pinBreps[0];
                pinSolid = pinBrep.CapPlanarHoles(_mydoc.ModelAbsoluteTolerance);

                pinSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == pinSolid.SolidOrientation)
                    pinSolid.Flip();

                Brep[] yokeOuterBreps = sweep.PerformSweep(yokePathCrv, _yokeOuterCrv);
                Brep yokeOuterBrep = yokeOuterBreps[0];
                Brep yokeOuterSolid = yokeOuterBrep.CapPlanarHoles(_mydoc.ModelAbsoluteTolerance);
                yokeOuterSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == yokeOuterSolid.SolidOrientation)
                    yokeOuterSolid.Flip();

                Transform innerYokeTrans = Transform.Translation(new Vector3d(0, 0, -_t));
                _yokeCrv.Transform(innerYokeTrans);

                Brep[] yokeInnerBreps = sweep.PerformSweep(yokePathInnerCrv, _yokeCrv);
                Brep yokeInnerBrep = yokeInnerBreps[0];
                Brep yokeInnerSolid = yokeInnerBrep.CapPlanarHoles(_mydoc.ModelAbsoluteTolerance);
                yokeInnerSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == yokeInnerSolid.SolidOrientation)
                    yokeInnerSolid.Flip();

                yokeSolid = Brep.CreateBooleanDifference(yokeOuterSolid, yokeInnerSolid, _mydoc.ModelAbsoluteTolerance)[0];

                sliderSolid = Brep.CreatePipe(_sliderCrv, _sliderRadius, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];

                sliderSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == sliderSolid.SolidOrientation)
                    sliderSolid.Flip();

                Brep blockOuterSolid = Brep.CreatePipe(_bearingBlockCrv, _sliderRadius + _t + _p, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];
                blockOuterSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == blockOuterSolid.SolidOrientation)
                    blockOuterSolid.Flip();

                Brep blockInnerSolid = Brep.CreatePipe(_bearingBlockCrv, _sliderRadius + _t, false, PipeCapMode.Flat, true, _mydoc.ModelAbsoluteTolerance, _mydoc.ModelAngleToleranceRadians)[0];
                blockInnerSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == blockInnerSolid.SolidOrientation)
                    blockInnerSolid.Flip();

                bearingBlockSolid = Brep.CreateBooleanDifference(blockOuterSolid, blockInnerSolid, _mydoc.ModelAbsoluteTolerance)[0];
                bearingBlockSolid.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == bearingBlockSolid.SolidOrientation)
                    bearingBlockSolid.Flip();

                #endregion

                #region transform (translation, rotation, self-rotation)
                Transform move = Transform.Translation(new Vector3d(_startPt));
                Transform rotation = Transform.Rotation(new Vector3d(0, 0, 1), _axisDir, _startPt);
                Vector3d originalMotionDir = new Vector3d(1, 0, 0);
                originalMotionDir.Transform(move);
                originalMotionDir.Transform(rotation);

                Transform selfRot = Transform.Rotation(originalMotionDir, _trajDir, _startPt);

                crankWheelSolid.Transform(move);
                crankWheelSolid.Transform(rotation);
                crankWheelSolid.Transform(selfRot);

                pinSolid.Transform(move);
                pinSolid.Transform(rotation);
                pinSolid.Transform(selfRot);

                yokeSolid.Transform(move);
                yokeSolid.Transform(rotation);
                yokeSolid.Transform(selfRot);

                sliderSolid.Transform(move);
                sliderSolid.Transform(rotation);
                sliderSolid.Transform(selfRot);

                bearingBlockSolid.Transform(move);
                bearingBlockSolid.Transform(rotation);
                bearingBlockSolid.Transform(selfRot);

                stopWallSolid.Transform(move);
                stopWallSolid.Transform(rotation);
                stopWallSolid.Transform(selfRot);

                #endregion

                _quickReturnModels.Add(crankWheelSolid);
                _quickReturnModels.Add(pinSolid);
                _quickReturnModels.Add(yokeSolid);
                _quickReturnModels.Add(sliderSolid);
                _quickReturnModels.Add(bearingBlockSolid);
                _quickReturnModels.Add(stopWallSolid);

            }
        }
    }
   
}
