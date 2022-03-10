﻿using System;
using System.Collections.Generic;
using System.IO;
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
using KinergyUtilities;
using Kinergy.Geom;
using Kinergy.Relationship;
using Kinergy.KineticUnit;
using Kinergy;
using System.Diagnostics;
using Rhino.Geometry.Intersect;

namespace Kinergy.KineticUnit
{
    class ContinuousTranslation : KineticUnit
    {
        //The initial inputs
        private Brep _model;
        private int _selectedAxisIndex=0;
        private int _speed;  // the range on the interface is 0-9
        private int _distance;    // the range on the interface is 0-9
        private int _energy; //  the range on the interface is 0-9
        private Vector3d _direction = Vector3d.Unset;
        private bool _addLock;
        private RhinoDoc _myDoc;
        private int _inputType;
        private Brep _innerCavity;
        private double _skeletonLen;
        private Point3d _motorRefPt;
        private Vector3d _mainAxis;

        private Curve _skeleton = null;
        private List<Shape> _modelCut;
        private List<Lock> _locks;
        private List<Gear> _gears=new List<Gear>();
        private GearTrainParam _gearParam;
        private Gear drivingGear;
        private Gear lastGear;
        private List<Entity> _axelsStoppers = new List<Entity>();
        private Helix _spring;
        private Rack endEffectorRack=null;
        private List<Entity> endEffectorRackConfiningStructure = new List<Entity>();
        private Entity endEffectorConnectingStructure=null;
        Brep constrainingStructureSpaceTaken;
        private int _shaftNum;
        private List<List<int>> _r_shaft_num_List;
        private List<List<double>> _shaft_radius_pool;

        public const double clearance = 0.3;
        public const double gearFaceWidth = 3.6;
        public const double gearModule = 1;
        public const double gearPressureAngle = 20;
        public const double shaftRadius = 1.5;
        public RhinoDoc myDoc = RhinoDoc.ActiveDoc;

        Brep b1 = null, b2 = null, b3 = null;
        Entity p1 = null, p2 = null, p3 = null;
        bool reversed = false;
        double t1 = 0, t2 = 0;

        public int ShaftNum { get => _shaftNum; set => _shaftNum = value; }

        public ContinuousTranslation(Brep Model,int selectedAxisIndex, Vector3d Direction, Brep innerCavity, Point3d motionCtrlPt, int speed, int dis, int eneryg, int InputType)
        {
            _model = Model;
            _selectedAxisIndex = selectedAxisIndex;
            _speed = speed;
            _energy = eneryg;
            _distance = dis;
            _direction = Direction;
            _modelCut = new List<Shape>();
            _myDoc = RhinoDoc.ActiveDoc;
            _locks = new List<Lock>();
            _inputType = InputType;
            _motorRefPt = motionCtrlPt;

            _innerCavity = innerCavity;
            BoxLike currB = new BoxLike(_model, _direction);

            _skeleton = currB.Skeleton;
            _skeleton.Transform(currB.RotateBack);
            _skeletonLen = _skeleton.PointAtNormalizedLength(0).DistanceTo(_skeleton.PointAtNormalizedLength(1));

            _shaftNum = 0;
            _r_shaft_num_List = new List<List<int>>();
        }
        public void AddGears(List<Gear> gears,List<Entity> axelsStoppers,GearTrainParam gearParam)
        {
            _gearParam=gearParam;
            drivingGear = gears[0];
            lastGear = gears.Last();
            //First remove existing gears before adding new
            foreach(Gear g in _gears)
            {
                entityList.Remove(g);
            }
            _gears.Clear();
            foreach(Entity e in _axelsStoppers)
            {
                entityList.Remove(e);
            }
            _axelsStoppers.Clear();
            foreach(Gear g in gears)
            {
                _gears.Add(g);
                entityList.Add(g);
            }
            foreach (Entity e in axelsStoppers)
            {
                _axelsStoppers.Add(e);
                entityList.Add(e);
            }
            //TODO register connecting relations
        }
        public void BuildEndEffectorRack(double eeMovingDis,GearTrainParam selectedGearTrainParam,int eeMovingDirectionSelection,Point3d eeLineDotPt,Vector3d mainAxis,Vector3d perpAxis, Vector3d otherAxis)
        {
            _mainAxis = mainAxis;
            //First remove existing structure
            if (endEffectorRack != null)
                entityList.Remove(endEffectorRack);
            endEffectorRack = null;
            foreach(Entity e in endEffectorRackConfiningStructure)
            {
                entityList.Remove(e);
            }
            endEffectorRackConfiningStructure.Clear();
            if (endEffectorConnectingStructure != null)
                entityList.Remove(endEffectorConnectingStructure);
            endEffectorConnectingStructure = null;

            Rack rack;
            List<Brep> constrainingStructure = new List<Brep>();
            
            Brep connectingStructure = null;//that connects rack and end effector;
            double gapMiddlePart2EE = 0;
            #region Build Rack and Constraining Structure

            //First calculate ee moving distance based on motion params.
            double eeMovingDistance = eeMovingDis;
            double rackExtraLength = 10;//TODO check this const value.
            if (eeMovingDirectionSelection == 2 || eeMovingDirectionSelection == 3)//The selected moving direction is perpendicular to main direction. i.e. same as user selected orientation
            {
                //The rack should be linked to last gear
                //TODO find out the parameter of the last gear and the user selected position (eeLineDotPt)
                double lastGearRadius = selectedGearTrainParam.parameters.Last().radius;
                Point3d contactPoint = eeLineDotPt + mainAxis * lastGearRadius;
                //Calculate rack length based on eeMovingDistance
                double rackLength = eeMovingDistance + rackExtraLength;
                //Create rack. It's composed with 3 parts: rack, backbone and connecting bone with ee
                double rackFaceWidth = 3.6;
                double rackThickness = 5;
                double teethHeight = 2.25;
                double backboneFacewidth = 10, backboneThickness = 4;
                double connectboneFacewidth = 5, connectboneThickness = 5;
                Point3d contactPointRackBackPoint = contactPoint + mainAxis * (teethHeight / 2 + 0.3);
                Point3d rackStartPoint, rackEndPoint;
                if (eeMovingDirectionSelection == 2)
                {
                    rackStartPoint = contactPointRackBackPoint - perpAxis * eeMovingDistance;
                    rackEndPoint = contactPointRackBackPoint + perpAxis * rackExtraLength;
                }
                else
                {
                    rackStartPoint = contactPointRackBackPoint + perpAxis * eeMovingDistance;
                    rackEndPoint = contactPointRackBackPoint - perpAxis * rackExtraLength;
                }
                Vector3d rackVector = new Vector3d(rackEndPoint) - new Vector3d(rackStartPoint);
                Point3d rackMidPoint = rackStartPoint + rackVector / 2;
                //Figure out the rack extrution direction and move half facewidth
                Vector3d extrutionDirection = new Plane(rackMidPoint, rackVector, -mainAxis).Normal;

                rack = new Rack(rackMidPoint - extrutionDirection * rackFaceWidth / 2, rackVector, -mainAxis, rackLength, 1, rackFaceWidth, otherAxis, rackThickness, 20);
                
                Plane rackPlane = new Plane(rackMidPoint, otherAxis, perpAxis);
                Box rackBackboneBox;
                if (rackPlane.Normal * mainAxis > 0)
                    rackBackboneBox = new Box(rackPlane, new Interval(-backboneFacewidth / 2, backboneFacewidth / 2), new Interval(-rackLength / 2, rackLength / 2), new Interval(0, backboneThickness));
                else
                    rackBackboneBox = new Box(rackPlane, new Interval(-backboneFacewidth / 2, backboneFacewidth / 2), new Interval(-rackLength / 2, rackLength / 2), new Interval(-backboneThickness, 0));
                Brep rackBackbone = rackBackboneBox.ToBrep();
                rack.Model = Brep.CreateBooleanUnion(new List<Brep> { rack.Model, rackBackbone }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];//Potential risk is failed boolean operation
                                                                                                                                                //Create holder and connectBone based on input model scale
                                                                                                                                                //Use a line intersection to get the scale -> 2 intersection points
                double maxScale =_model.GetBoundingBox(true).Diagonal.Length * 2;
                Line line = new Line(eeLineDotPt - perpAxis * maxScale, perpAxis * maxScale * 2);
                Point3d[] intersections;
                if (!Rhino.Geometry.Intersect.Intersection.CurveBrep(line.ToNurbsCurve(), _model, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out _, out intersections))
                {
                    //If intersection with model fails, use inner cavity instead
                    line = new Line(eeLineDotPt - mainAxis, perpAxis);
                    Interval interval;
                    //Calculate the midpoint of axis
                    Point3d edgePoint1, edgePoint2;
                    Rhino.Geometry.Intersect.Intersection.LineBox(line, _innerCavity.GetBoundingBox(true), RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out interval);
                    edgePoint1 = line.PointAt(interval.Min) + mainAxis;
                    edgePoint2 = line.PointAt(interval.Max) + mainAxis;
                    intersections = new Point3d[2];
                    intersections[0] = edgePoint1;
                    intersections[1] = edgePoint2;
                }
                //Use the 2 intersection points to create holder. holder has 4 parts, 2 from intersection0 to last gear. 2 from las gear to intersection1
                #region create 4 parts of holder
                //Figure out the ranges
                Point3d gearSidePoint1 = eeLineDotPt + perpAxis * (lastGearRadius+teethHeight);
                Point3d gearSidePoint2 = eeLineDotPt - perpAxis * (lastGearRadius+teethHeight);
                Point3d[] segmentingPoints = new Point3d[4];
                segmentingPoints[0] = intersections[0];
                segmentingPoints[3] = intersections[1];
                if (intersections[0].DistanceTo(gearSidePoint1) < intersections[0].DistanceTo(gearSidePoint2))
                {
                    segmentingPoints[1] = gearSidePoint1;
                    segmentingPoints[2] = gearSidePoint2;
                }
                else
                {
                    segmentingPoints[1] = gearSidePoint2;
                    segmentingPoints[2] = gearSidePoint1;
                }
                //Each part of holder is formed with 3 boxes.Use a same section line to generate them
                //Plane holderPlane = new Plane(segmentingPoints[0], otherAxis, perpAxis);
                List<Point3d> sectionLinePts = new List<Point3d>();
                sectionLinePts.Add(segmentingPoints[0]);
                Point3d pt2 = segmentingPoints[0] + otherAxis * (2 + 0.3 + backboneFacewidth / 2);
                sectionLinePts.Add(pt2);
                Point3d pt3 = pt2 + mainAxis * (lastGearRadius + 0.3 + teethHeight / 2   + backboneThickness + 0.3 + 2);
                sectionLinePts.Add(pt3);
                Point3d pt4 = segmentingPoints[0] + mainAxis * (lastGearRadius + 0.3 + teethHeight / 2  + backboneThickness + 0.3 + 2) + otherAxis * (connectboneFacewidth / 2 + 0.3);
                sectionLinePts.Add(pt4);
                Point3d pt5 = pt4 - mainAxis * 2;
                sectionLinePts.Add(pt5);
                Point3d pt6 = pt5 + otherAxis * (backboneFacewidth - connectboneFacewidth) / 2;
                Point3d pt8 = segmentingPoints[0] + mainAxis * (lastGearRadius - 0.3);//Here minus 0.3 is to make sure rack dosen't overlap with holder
                Point3d pt7 = pt8 + otherAxis * (0.3 + backboneFacewidth / 2);
                sectionLinePts.Add(pt6);
                sectionLinePts.Add(pt7);
                sectionLinePts.Add(pt8);
                sectionLinePts.Add(segmentingPoints[0]);
                Polyline section = new Polyline(sectionLinePts);
                Curve sectionCurve = section.ToNurbsCurve();

                var sweep = new SweepOneRail();
                sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
                sweep.ClosedSweep = false;
                sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;
                //The first holder from seg0 to seg1
                Curve rail1 = new Line(segmentingPoints[0], segmentingPoints[1]).ToNurbsCurve();
                Brep[] holder1list = sweep.PerformSweep(rail1, sectionCurve);
                Brep holder1 = holder1list[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                //Second is a mirror of first
                Brep holder2 = holder1list[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                holder2.Transform(Transform.Mirror(segmentingPoints[0], otherAxis));
                //Third from seg2 to seg3
                sectionCurve.Transform(Transform.Translation(segmentingPoints[2] - segmentingPoints[0]));
                Curve rail3 = new Line(segmentingPoints[2], segmentingPoints[3]).ToNurbsCurve();
                Brep[] holder3list = sweep.PerformSweep(rail3, sectionCurve);
                Brep holder3 = holder3list[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                //Fourth is mirrored from 3rd
                Brep holder4 = holder3list[0].CapPlanarHoles(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                holder4.Transform(Transform.Mirror(segmentingPoints[0], otherAxis));
                constrainingStructure = new List<Brep> { holder1, holder2, holder3,holder4 };
                #endregion

                //Use the 2 inersection points to create connector
                Vector3d intersectionVector = (new Vector3d(intersections[1]) - new Vector3d(intersections[0]));
                Point3d intersectionMidPoint = intersections[0] + intersectionVector / 2;
                Box connectboneBox;
                Plane connectbonePlane = new Plane(intersectionMidPoint + mainAxis * (lastGearRadius + rackThickness + teethHeight / 2 + 0.3 + backboneThickness), otherAxis, perpAxis);
                if (connectbonePlane.Normal * mainAxis > 0)
                    connectboneBox = new Box(rackPlane, new Interval(-connectboneFacewidth / 2, connectboneFacewidth / 2), new Interval(-rackLength / 2, rackLength / 2), new Interval(backboneThickness, backboneThickness+connectboneThickness));
                else
                    connectboneBox = new Box(rackPlane, new Interval(-connectboneFacewidth / 2, connectboneFacewidth / 2), new Interval(-rackLength / 2, rackLength / 2), new Interval(-connectboneThickness-backboneThickness, -backboneThickness));
                Brep connectbone = connectboneBox.ToBrep();
                connectingStructure = connectbone;
                gapMiddlePart2EE = lastGearRadius + 0.3 + teethHeight / 2 + rackThickness + backboneThickness + connectboneThickness;

                Plane constrainPlane = new Plane(eeLineDotPt,mainAxis,otherAxis);
                Box constrainSpaceBox = new Box(constrainPlane, new Interval(0, lastGearRadius + 0.3 + teethHeight / 2 + backboneThickness + 0.3 + 2),
                    new Interval(-2 - 0.3 - backboneFacewidth / 2 - 0.3, 2 + 0.3 + backboneFacewidth / 2 + 0.3),
                    new Interval(-intersections[0].DistanceTo(intersections[1]), intersections[0].DistanceTo(intersections[1])));
                constrainingStructureSpaceTaken = constrainSpaceBox.ToBrep();
            }
            else//The selected moving direction is exactly the main direction, so the rack would get into the model
            {
                GearParameter lgp = selectedGearTrainParam.parameters.Last();
                double lastGearRadius = lgp.radius;
                Point3d contactPoint = eeLineDotPt + perpAxis * lastGearRadius;//TODO select up or down based on gear position!
                                                                               //Calculate rack length based on eeMovingDistance and inner cavity space
                rackExtraLength = Math.Max(rackExtraLength, lgp.radius + 2);//Make sure rack extra length can outgrow last gear
                double rackLength = eeMovingDistance + rackExtraLength;
                double innerCavityMainAxisLength = 0;
                Box innerCavityBox = new Box(_innerCavity.GetBoundingBox(true));
                switch (_selectedAxisIndex)
                {
                    case 1: innerCavityMainAxisLength = innerCavityBox.X.Length; break;
                    case 2: innerCavityMainAxisLength = innerCavityBox.Y.Length; ; break;
                    case 3: innerCavityMainAxisLength = innerCavityBox.Z.Length; break;
                    default: break;
                }
                double rackFaceWidth = 3.6;
                double rackThickness = 5;
                double teethHeight = 2.25;
                double rackHolderWidth = 2;
                double rackCompoundThickness = rackFaceWidth + 0.3 * 2 + rackHolderWidth * 2;
                //First just use the end tip of the last gear for the rack contact
                Point3d lgctAtEnd = lgp.center + lgp.norm * lgp.faceWidth;
                Point3d rackContactPoint = lgctAtEnd - lgp.radius * perpAxis - lgp.norm * rackCompoundThickness / 2;
                Point3d rackContactPointOnRackBack = rackContactPoint - perpAxis * (0.5 + teethHeight / 2 + rackThickness);//Xia's note: probably need clearance more than 0.3
                Point3d rackStart = rackContactPointOnRackBack + mainAxis * rackExtraLength;
                Point3d rackEnd = rackContactPointOnRackBack - mainAxis * eeMovingDistance;
                Vector3d rackVector = new Vector3d(rackEnd) - new Vector3d(rackStart);
                Point3d rackMidPoint = rackStart + rackVector / 2;
                //Since rack is extruded to one direction, we need to find it first to know it's actual position
                Vector3d extrutionDirection = new Plane(rackMidPoint, rackVector, perpAxis).Normal;
                rackMidPoint -= extrutionDirection * rackFaceWidth / 2;
                rackMidPoint = rackMidPoint + perpAxis * rackThickness;
                rack = new Rack(rackMidPoint, rackVector, perpAxis, rackLength, 1, rackFaceWidth, otherAxis, rackThickness, 20);
                //Then make the confining structure, which are 2 bars beside rack and some caps. bars are higher than rack by 0.3 and capped at rack end and farthest position
                Plane rackPlane = new Plane(rackContactPointOnRackBack, mainAxis, otherAxis);

                Box bar1Box = new Box(rackPlane, new Interval(-innerCavityMainAxisLength, -(rackExtraLength - 2))
                    , new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, -rackFaceWidth / 2 - 0.3),
                    //, new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, 0),
                    new Interval(-0.3, 0.3 + teethHeight  + rackThickness));
                Box bar2Box = new Box(rackPlane, new Interval(-innerCavityMainAxisLength, -(rackExtraLength - 2))
                    , new Interval(rackFaceWidth / 2 + 0.3, rackFaceWidth / 2 + 0.3 + rackHolderWidth), 
                    new Interval(-0.3, 0.3 + teethHeight  + rackThickness));
                
                //Add cap
                Box cap1Box = new Box(rackPlane, new Interval(-rackExtraLength, -(rackExtraLength - 2)),
                    new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, rackFaceWidth / 2 + 0.3 + rackHolderWidth),
                    new Interval(0.3 + teethHeight  + rackThickness, 0.3 + teethHeight  + rackThickness + 2));
                Box cap2Box = new Box(rackPlane, new Interval(-eeMovingDistance, -(eeMovingDistance - 2)),
                    new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, rackFaceWidth / 2 + 0.3 + rackHolderWidth),
                    new Interval(0.3 + teethHeight  + rackThickness, 0.3 + teethHeight  + rackThickness + 2));
                Box cap3Box = new Box(rackPlane, new Interval(-innerCavityMainAxisLength, -(innerCavityMainAxisLength - 2)),
                    new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, rackFaceWidth / 2 + 0.3 + rackHolderWidth),
                    new Interval(0.3 + teethHeight  + rackThickness, 0.3 + teethHeight  + rackThickness + 2));
                Box cap4Box = new Box(rackPlane, new Interval(-rackExtraLength, -(rackExtraLength - 2)),
                    new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, rackFaceWidth / 2 + 0.3 + rackHolderWidth),
                    new Interval(-2.3,-0.3));
                Box cap5Box = new Box(rackPlane, new Interval(-eeMovingDistance, -(eeMovingDistance - 2)),
                    new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, rackFaceWidth / 2 + 0.3 + rackHolderWidth),
                    new Interval(-2.3, -0.3));
                Box cap6Box = new Box(rackPlane, new Interval(-innerCavityMainAxisLength, -(innerCavityMainAxisLength - 2)),
                    new Interval(-rackFaceWidth / 2 - 0.3 - rackHolderWidth, rackFaceWidth / 2 + 0.3 + rackHolderWidth),
                    new Interval(-2.3, -0.3));
                if (rackPlane.Normal * perpAxis < 0)
                {
                    bar1Box.Transform(Transform.Mirror(rackPlane));
                    bar2Box.Transform(Transform.Mirror(rackPlane));
                    cap1Box.Transform(Transform.Mirror(rackPlane));
                    cap2Box.Transform(Transform.Mirror(rackPlane));
                    cap3Box.Transform(Transform.Mirror(rackPlane));
                    cap4Box.Transform(Transform.Mirror(rackPlane));
                    cap5Box.Transform(Transform.Mirror(rackPlane));
                    cap6Box.Transform(Transform.Mirror(rackPlane));
                }
                Brep bar1 = bar1Box.ToBrep(), bar2 = bar2Box.ToBrep();
                Brep cap1 = cap1Box.ToBrep(), cap2 = cap2Box.ToBrep(), cap3 = cap3Box.ToBrep(),cap4= cap4Box.ToBrep(), cap5 = cap5Box.ToBrep(), cap6 = cap6Box.ToBrep();
                Brep union = Brep.CreateBooleanUnion(new List<Brep> { bar1, cap1, cap2, cap3, bar2,cap4,cap5,cap6 }, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
                constrainingStructure.Add(union);
                gapMiddlePart2EE = rackExtraLength;
                #region Deal with the case when rack is longer than inner cavity. Just dig a hole out
                //Just use a box to make space for rack.
                Brep cuttingBox = new Box(rackPlane, new Interval(-eeMovingDistance, rackExtraLength), new Interval(-0.3 - rackFaceWidth / 2, 0.3 + rackFaceWidth / 2),
                    new Interval(-0.3, 0.3 + teethHeight / 2 + rackThickness)).ToBrep();
                constrainingStructureSpaceTaken = cuttingBox;
                //TODO cut model mid part and start part with this box 

                //if (innerCavityMainAxisLength < eeMovingDistance)
                //{
                //Then we need to use bounding box of rack to boolean main model

                //}
                #endregion
            }
            #endregion
            endEffectorRack = rack;
            if(connectingStructure!=null)
                endEffectorConnectingStructure = new Entity(connectingStructure, false, "connecting structure");
            foreach (Brep b in constrainingStructure)
            {
                Entity newEntity = new Entity(b, false, "constraining structure");
                endEffectorRackConfiningStructure.Add(newEntity);
                entityList.Add(newEntity);
            }
            entityList.Add(endEffectorRack);
            if (endEffectorConnectingStructure != null)
                entityList.Add(endEffectorConnectingStructure);
        }

        private Cylinder GetCylinder(Brep c, Vector3d d)
        {
            Brep m = c.DuplicateBrep();
            m.Transform(Transform.Rotation(d, Vector3d.XAxis, Point3d.Origin));
            BoundingBox b = m.GetBoundingBox(true);
            double r = (b.Max.Y - b.Min.Y) / 2;
            double l = b.Max.X - b.Min.X;
            Point3d startPoint = b.PointAt(0, 0.5, 0.5);
            startPoint.Transform(Transform.Rotation(Vector3d.XAxis, d, Point3d.Origin));
            Plane p = new Plane(startPoint, d);
            Circle circle = new Circle(p, r);
            Cylinder cylinder = new Cylinder(circle, l);
            return cylinder;
        }

        public void Set3Parts(double T1, double T2, Brep B1, Brep B2, Brep B3)
        {
            t1 = T1;
            t2 = T2;
            if(t1>t2)
            {
                double t = t1;
                t1 = t2;
                t2 = t;
            }
            b1 = B1;
            b2 = B2;
            b3 = B3;
            //Tell if motion control point is closer to B1 or B3. make sure b1 is driving part and b3 is ee part
            double dis1 = _motorRefPt.DistanceTo(b1.ClosestPoint(_motorRefPt));
            double dis3 = _motorRefPt.DistanceTo(b3.ClosestPoint(_motorRefPt));
            if (dis1> dis3)
            {
                Brep t = b1;
                b1 = b3;
                b3 = t;
            }
        }
        public void CreateShell()
        {
            double shellThickness = 2;
            Brep part2;
            GearParameter lgp = _gearParam.parameters.Last();
            Brep lgCylinder = new Cylinder(new Circle(new Plane(lgp.center, lgp.norm), lgp.radius + 2), lgp.faceWidth+0.6).ToBrep(true,true);
            lgCylinder.Transform(Transform.Translation(-lgp.norm * 0.3));
            part2=Brep.CreateBooleanDifference(b2, lgCylinder,myDoc.ModelAbsoluteTolerance)[0];
            Brep[] shells = Brep.CreateOffsetBrep(b2, (-1) * shellThickness, false, true, myDoc.ModelRelativeTolerance, out _, out _);
            Brep innerShell = shells[0];
            part2= Brep.CreateBooleanDifference(part2, innerShell, myDoc.ModelAbsoluteTolerance)[0];
            //Cut b3 with gear cylinder
            Brep part3 = Brep.CreateBooleanDifference(b3, lgCylinder, myDoc.ModelAbsoluteTolerance)[0];
            //Cut part 3 with plane to greate a gap
            Plane cutter;
            try
            {
                cutter = new Plane(_skeleton.PointAtNormalizedLength(t1), _mainAxis);
                cutter.Transform(Transform.Translation(_mainAxis * 0.3));
                Brep[] Cut_Brep = part3.Trim(cutter, myDoc.ModelAbsoluteTolerance);
                part3 = Cut_Brep[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            }
            catch
            {
                cutter = new Plane(_skeleton.PointAtNormalizedLength(t2), _mainAxis);
                cutter.Transform(Transform.Translation(_mainAxis * 0.3));
                Brep[] Cut_Brep = part3.Trim(cutter, myDoc.ModelAbsoluteTolerance);
                part3 = Cut_Brep[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            }
            
            //Cut part3 with constraining structure
            try
            {
                part3= Brep.CreateBooleanDifference(part3,constrainingStructureSpaceTaken , myDoc.ModelAbsoluteTolerance)[0];
            }
            catch
            {
                
            }
            entityList.Remove(p1);
            entityList.Remove(p2);
            entityList.Remove(p3);
            p1 = new Entity(b1);
            entityList.Add(p1);
            p2 = new Entity(part2);
            entityList.Add(p2);
            p3 = new Entity(part3);
            entityList.Add(p3);
        }
        public void CalculateShaftNumAndGearRadius(int level, out double n, out double r)
        {
            // default shaft number and gear radius
            n = 2;
            r = 4.5;

            if (_shaft_radius_pool == null)
                return;

            double interval = _shaft_radius_pool.Count / 10.0;

            List<double> ele = _shaft_radius_pool.ElementAt((int)Math.Floor(level * interval));

            n = (int)ele.ElementAt(0);
            r = ele.ElementAt(1);
        }

        public void CalculateSpaceForKineticUnit(Vector3d kineticUnitDir, Vector3d axelDir, double axelSpace, double gearSpace, double unitLen, double initialOffset, double finalGearPositionRatio)
        {
            double R = 0;
            List<double> gearRList = new List<double>();
            
            if(_r_shaft_num_List != null && _r_shaft_num_List.Count > 0)
            {
                _r_shaft_num_List.Clear();
            }

            double r_ceiling = Math.Min(unitLen - 4.5, gearSpace / 2 - clearance - 2);
            if (r_ceiling < 4.5)
                return;
            
            for(R = 5; R <= r_ceiling; R = R + 0.5)
            {
                if (R <= (unitLen - 9.3) / 2)
                {
                    // double n_ceiling = Math.Min((unitLen - 4.5 - R) / (R + 4.8) + 1, (axelSpace - initialOffset + 0.3) / 3.9);
                    double n_ceiling = Math.Min((unitLen - 4.5 - R) / (R + 4.8) + 1, (axelSpace/2) / 3.9);
                    List<int> n_seque = new List<int>();
                    for(int i=2; i<=Math.Floor(n_ceiling); i++)
                    {
                        n_seque.Add(i);
                    }

                    // update the list r_shaft_num_list and gearRList
                    _r_shaft_num_List.Add(n_seque);
                    gearRList.Add(R);

                    // update the list shaft_radius_pool
                    foreach(int n in n_seque)
                    {
                        double gearRatio = Math.Pow(R / 4.5, n-1);

                        int idx = 0;
                        if(_shaft_radius_pool == null)
                        {
                            _shaft_radius_pool = new List<List<double>>();
                            List<double> temp = new List<double>();
                            temp.Add(n);
                            temp.Add(R);

                            _shaft_radius_pool.Add(temp);
                        }
                        else
                        {
                            bool isDeleted = false;

                            foreach (var ele in _shaft_radius_pool)
                            {
                                double shaftNum = ele.ElementAt(0);
                                double gearR = ele.ElementAt(1);
                                double testGearRatio = Math.Pow(gearR / 4.5, shaftNum-1);

                                if (gearRatio > testGearRatio)
                                {
                                    // keep looking 
                                    idx++;   
                                }
                                else if (gearRatio == testGearRatio)
                                {
                                    // replace the current item
                                    isDeleted = true;
                                    idx = _shaft_radius_pool.IndexOf(ele);
                                    break;
                                }
                                else
                                {
                                    // insert the item
                                    idx = _shaft_radius_pool.IndexOf(ele);
                                    break;
                                }
                            }

                            if(idx == _shaft_radius_pool.Count)
                            {
                                List<double> temp = new List<double>();
                                temp.Add(n);
                                temp.Add(R);
                                _shaft_radius_pool.Add(temp);
                            }
                            else
                            {
                                if (isDeleted)
                                {
                                    _shaft_radius_pool.RemoveAt(idx);
                                    List<double> temp = new List<double>();
                                    temp.Add(n);
                                    temp.Add(R);
                                    _shaft_radius_pool.Insert(idx, temp);
                                }
                                else
                                {
                                    List<double> temp = new List<double>();
                                    temp.Add(n);
                                    temp.Add(R);
                                    _shaft_radius_pool.Insert(idx, temp);
                                }
                            }
                        }
                    }
                }
            }
        }

        public void GenerateSpringMotor(Point3d eeCen, int speed_input, int dis_input, int energy_input)
        {
            double shaftNum = 0;
            double gearRadius = 0;
            double clearance = 0.3;
            CalculateShaftNumAndGearRadius(speed_input, out shaftNum, out gearRadius);

            double dis = clearance + 4.5 + (shaftNum - 1) * (gearRadius + clearance + 4.5);

            double t = 0;
            _skeleton.ClosestPoint(eeCen, out t);
            if (t > 0.5)
            {
                dis = -dis;
            }
            Point3d enginePos = eeCen + direction * dis;
            

            if (_inputType == 1)
            {
                // press control



            }
            else
            {
                // turn control

            }

        }

       
        
    }
}
