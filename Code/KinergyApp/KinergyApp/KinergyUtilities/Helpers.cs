﻿using Kinergy.Geom;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace KinergyUtilities
{
    class Helpers
    {
        public Helpers()
        {

        }

        public void mapSpeedToGears(int speed_input, List<double> gr_list, List<GearTrainScheme> gear_schemes, out int schemeNum, out int paramNum)
        {
            schemeNum = -1;
            paramNum = -1;

            double target_gr = 0;

            if (gr_list.Count < 10)
            {
                // the number of generated gearsets is less than 10
                int leftover = 10 - gr_list.Count;
                int left_leftover = leftover / 2;
                int right_leftover = leftover - left_leftover;


                if (speed_input <= left_leftover)
                {
                    target_gr = gr_list.ElementAt(0);
                }
                else if (speed_input >= (gr_list.Count + left_leftover))
                {
                    target_gr = gr_list.ElementAt(gr_list.Count - 1);
                }
                else
                {
                    target_gr = gr_list.ElementAt(speed_input - left_leftover - 1);
                }
            }
            else
            {
                // the number of generated gear sets is more than 10
                target_gr = gr_list.ElementAt(gr_list.Count / 10 * (speed_input - 1));
            }

            #region find the schemeNum and paramNum based on the calculated target_gr
            bool isfoundIndexes = false;
            foreach (GearTrainScheme gts in gear_schemes)
            {
                schemeNum = gear_schemes.IndexOf(gts);
                foreach (GearTrainParam gtp in gts.parameters)
                {
                    if (gtp.gearRatio == target_gr)
                    {
                        paramNum = gts.parameters.IndexOf(gtp);
                        isfoundIndexes = true;
                        break;
                    }
                }

                if (isfoundIndexes)
                    break;
            }
            #endregion
        }

        /// <summary>
        /// Return the axel and spacer entities - 
        ///      if the control is press-helix, the last three elements in the return list are the connector bars
        ///      if the control is turn-spiral, the last three elements in the return list art the turning shaft (lock handler)
        /// </summary>
        /// <param name="gear_info">the list of gear parameters for the calculated gear train</param>
        /// <param name="body">the model body</param>
        /// <param name="controlType">the selected controlling method: helix or spiral</param>
        /// <param name="clearance">the clearance for printing - 0.3mm</param>
        /// <returns></returns>
        public List<Entity> genAxelsStoppers(List<GearParameter> gear_info, Brep body, int controlType, double clearance)
        {
            List<Entity> models = new List<Entity>();
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            var sweep = new SweepOneRail();
            sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            sweep.ClosedSweep = false;
            sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            double rad1 = 1.5;
            double rad2 = 2.2;

            //TODO register entity relations
            if (controlType == 1)
            {
                // press control using helical springs

                foreach (GearParameter gp in gear_info)
                {
                    int idx = gear_info.IndexOf(gp);
                    Vector3d axelDir = gp.norm;
                    Point3d gearCen = gp.center;

                    #region generate the axel

                    // only genearting the shaft when it is the odd gear order 
                    if(idx % 2 == 0)
                    {
                        Curve crossLineCrv = new Line(gearCen - axelDir * int.MaxValue, gearCen + axelDir * int.MaxValue).ToNurbsCurve();
                        Curve[] crvs;
                        Point3d[] pts;
                        Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, body, myDoc.ModelAbsoluteTolerance, out crvs, out pts);

                        Point3d ptEnd = new Point3d();
                        Point3d ptStart = new Point3d();
                        Vector3d intersectVec = (pts[0] - pts[1]);
                        intersectVec.Unitize();
                        if (intersectVec*axelDir>0.99)
                        {
                            ptEnd = pts[0] - axelDir * 1;
                            ptStart = pts[1] + axelDir * 1;
                        }
                        else
                        {
                            ptEnd = pts[1] - axelDir * 1;
                            ptStart = pts[0] + axelDir * 1;
                        }
                        Curve lineCrv = new Line(ptStart, ptEnd).ToNurbsCurve();
                        Shaft axelShaft = new Shaft(ptStart, ptStart.DistanceTo(ptEnd), rad1, axelDir);

                        //TODO If it is the last gear shaft, give it a name for later use
                        if (idx == gear_info.Count() - 1)
                            axelShaft.SetName("lastShaft");

                        models.Add(axelShaft);
                    }
                    #endregion

                    #region generate the spacers

                    double offset1 = gp.faceWidth + clearance;
                    double offset2 = clearance;
                    Vector3d initialDir = gear_info.ElementAt(1).center - gear_info.ElementAt(0).center;
                    initialDir.Unitize();

                    if (idx % 2 == 0)
                    {
                        // pinion gear

                        // add the first spacer
                        Spacer sp1 = new Spacer(gearCen - initialDir * offset2, 1, rad2, 3, (-initialDir));
                        models.Add(sp1);

                        if(idx == gear_info.Count - 1)
                        {
                            // the last gear is a pinion
                            Spacer sp2 = new Spacer(gearCen + axelDir * (offset2 + gp.faceWidth), 1, rad2, 3, axelDir);
                            models.Add(sp2);
                        }
                    }
                    else
                    {
                        // bull gear
                        Spacer sp2 = new Spacer(gearCen - axelDir * offset2, 1, rad2, 3, -axelDir);
                        models.Add(sp2);
                    }

                    #endregion
                }

                #region the last three entities are the connector bars for the first and the second gears

                Point3d firstGearCen = gear_info.ElementAt(0).center;
                Plane gearPln = new Plane(firstGearCen, gear_info.ElementAt(0).norm);
                Vector3d con_pt_X = gearPln.XAxis;
                Vector3d con_pt_Y = gearPln.YAxis;

                Point3d rect1 = firstGearCen + con_pt_X * 2 + con_pt_Y * 2;
                Point3d rect2 = firstGearCen + con_pt_X * 2 - con_pt_Y * 2;
                Point3d rect3 = firstGearCen + con_pt_X * 3 - con_pt_Y * 2;
                Point3d rect4 = firstGearCen + con_pt_X * 3 + con_pt_Y * 2;

                List<Point3d> gearConnector = new List<Point3d>();
                gearConnector.Add(rect1);
                gearConnector.Add(rect2);
                gearConnector.Add(rect3);
                gearConnector.Add(rect4);
                gearConnector.Add(rect1);

                Polyline gearConRect = new Polyline(gearConnector);
                Curve gearConRectCrv = gearConRect.ToNurbsCurve();

                Line conLine = new Line(firstGearCen, gear_info.ElementAt(1).center);
                Curve conCrv = conLine.ToNurbsCurve();

                #region add the first connector
                Brep[] conBreps = sweep.PerformSweep(conCrv, gearConRectCrv);
                Brep conBrep = conBreps[0];
                Brep connector1 = conBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                connector1.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == connector1.SolidOrientation)
                    connector1.Flip();
                #endregion

                #region add the second connector

                Brep connector2 = connector1.DuplicateBrep();
                Transform rot = Transform.Rotation((Math.PI * 2) / 3, gear_info.ElementAt(0).norm, firstGearCen);
                connector2.Transform(rot);

                #endregion

                #region add the third connector

                Brep connector3 = connector2.DuplicateBrep();
                connector3.Transform(rot);

                #endregion

                BoxLike con1 = new BoxLike(connector1);
                BoxLike con2 = new BoxLike(connector2);
                BoxLike con3 = new BoxLike(connector3);

                models.Add(con1);
                models.Add(con2);
                models.Add(con3);

                #endregion
            }
            else
            {
                // turn control using spiral springs

                foreach (GearParameter gp in gear_info)
                {
                    int idx = gear_info.IndexOf(gp);

                    if (idx == 0)
                        continue;
                    
                    Vector3d axelDir = gp.norm;
                    Point3d gearCen = gp.center;
                    

                    #region generate the axel

                    // only genearting the shaft when it is the odd gear order 
                    if (idx % 2 == 0 && idx != 0)
                    {
                        Curve crossLineCrv = new Line(gearCen - axelDir * int.MaxValue, gearCen + axelDir * int.MaxValue).ToNurbsCurve();
                        Curve[] crvs;
                        Point3d[] pts;
                        Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, body, myDoc.ModelAbsoluteTolerance, out crvs, out pts);

                        Point3d ptEnd = new Point3d();
                        Point3d ptStart = new Point3d();
                        Vector3d intersectVec = (pts[0] - pts[1]);
                        intersectVec.Unitize();
                        if (intersectVec* axelDir>0.99)
                        {
                            ptEnd = pts[0] - axelDir * 1;
                            ptStart = pts[1] + axelDir * 1;
                        }
                        else
                        {
                            ptEnd = pts[1] - axelDir * 1;
                            ptStart = pts[0] + axelDir * 1;
                        }
                        Curve lineCrv = new Line(ptStart, ptEnd).ToNurbsCurve();
                        Shaft axelShaft = new Shaft(ptStart, ptStart.DistanceTo(ptEnd), rad1, axelDir);

                        //TODO If it is the last gear shaft, give it a name for later use
                        if (idx == gear_info.Count() - 1)
                            axelShaft.SetName("lastShaft");
                        models.Add(axelShaft);
                    }
                    #endregion

                    #region generate the spacers

                    double offset = gp.faceWidth + clearance;

                    if (idx % 2 == 0)
                    {
                        // pinion gear

                        // add the first spacer
                        Spacer sp1 = new Spacer(gearCen - axelDir * clearance, 1, rad2, 3, -axelDir);
                        models.Add(sp1);

                        if (idx == gear_info.Count - 1)
                        {
                            // the last gear is a pinion
                            Spacer sp2 = new Spacer(gearCen + axelDir * offset, 1, rad2, 3, axelDir);
                            models.Add(sp2);
                        }
                    }
                    else
                    {
                        // bull gear
                        if (idx != 1)
                        {
                            Spacer sp2 = new Spacer(gearCen + axelDir * offset, 1, rad2, 3, axelDir);
                            models.Add(sp2);
                        }
                    }

                    #endregion
                }

                #region add the first shaft as the lock handler

                Point3d firstGearCen = gear_info.ElementAt(0).center;
                Vector3d firstGearDir = gear_info.ElementAt(0).center - gear_info.ElementAt(1).center;
                firstGearDir.Unitize();
                Curve lockLineCrv = new Line(firstGearCen - firstGearDir * int.MaxValue, firstGearCen + firstGearDir * int.MaxValue).ToNurbsCurve();
                Curve[] lockLineCrvs;
                Point3d[] lockLinePts;
                Rhino.Geometry.Intersect.Intersection.CurveBrep(lockLineCrv, body, myDoc.ModelAbsoluteTolerance, out lockLineCrvs, out lockLinePts);

                Point3d lockPtEnd = new Point3d();
                Point3d lockPtStart = new Point3d();
                Vector3d lockLineVec = lockLinePts[0] - lockLinePts[1];
                lockLineVec.Unitize();
                if (lockLineVec*firstGearDir>0.99)
                {
                    lockPtEnd = lockLinePts[0] + firstGearDir * 7;
                    lockPtStart = lockLinePts[1] + firstGearDir * 4.5;

                    Socket lockShaftSocket = new Socket(lockPtStart, firstGearDir);
                    models.Add(lockShaftSocket);


                    Point3d shaftStartPt = lockLinePts[1] + firstGearDir * 2.75;
                    Point3d shaftEndPt = lockPtEnd;
                    Shaft lockAxelShaft = new Shaft(shaftStartPt, shaftStartPt.DistanceTo(shaftEndPt), rad1, firstGearDir);
                    models.Add(lockAxelShaft);

                    Shaft lockAxelShaftDisc = new Shaft(shaftStartPt, 1.5, 3.8, firstGearDir);
                    models.Add(lockAxelShaftDisc);

                    lockAxelShaft.SetName("MiddleShellBreakerShaft");

                    Point3d handlerPt = shaftEndPt + firstGearDir * 3;
                    Plane handlerPln = new Plane(handlerPt, firstGearDir);
                    Vector3d handlerPlnX = handlerPln.XAxis;
                    handlerPlnX.Unitize();
                    Point3d handlerPtNew = handlerPt - handlerPlnX * 1.5;
                    Shaft handlerDisc = new Shaft(handlerPtNew, 3, 5, handlerPlnX);
                    models.Add(handlerDisc);
                }
                else
                {
                    lockPtEnd = lockLinePts[1] + firstGearDir * 7;
                    lockPtStart = lockLinePts[0] + firstGearDir * 4.5;
                    Socket lockShaftSocket = new Socket(lockPtStart, firstGearDir);
                    models.Add(lockShaftSocket);

                    Point3d shaftStartPt = lockLinePts[0] + firstGearDir * 2.75;
                    Point3d shaftEndPt = lockPtEnd;
                    Shaft lockAxelShaft = new Shaft(shaftStartPt, shaftStartPt.DistanceTo(shaftEndPt), rad1, firstGearDir);

                    lockAxelShaft.SetName("MiddleShellBreakerShaft");

                    models.Add(lockAxelShaft);

                    Shaft lockAxelShaftDisc = new Shaft(shaftStartPt, 1.5, 3.8, firstGearDir);
                    models.Add(lockAxelShaftDisc);

                    Point3d handlerPt = shaftEndPt + firstGearDir * 3;
                    Plane handlerPln = new Plane(handlerPt, firstGearDir);
                    Vector3d handlerPlnX = handlerPln.XAxis;
                    handlerPlnX.Unitize();
                    Point3d handlerPtNew = handlerPt - handlerPlnX * 1.5;
                    Shaft handlerDisc = new Shaft(handlerPtNew, 3, 5, handlerPlnX);
                    models.Add(handlerDisc);
                }
                


                #endregion
            }

            return models;
        }

        /// <summary>
        /// Return the gear entitites
        /// </summary>
        /// <param name="gear_info">the list of gear parameters for the calculated gear train</param>
        /// <param name="controlType">the selected controlling method: helix or spiral</param>
        /// <param name="clearance">the clearance between the shaft and the gear - 0.4mm</param>
        /// <returns></returns>
        public List<Gear> genGears(List<GearParameter> gear_info, int controlType, double clearance,bool lastGearMovable=true)
        {
            List<Gear> models = new List<Gear>();
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            //TODO register gear relations
            if(controlType == 1)
            {
                // press control with helix springs
                bool isGroove = false;
                foreach (GearParameter gp in gear_info)
                {
                    int numTeeth = (int)(gp.radius * 2);
                    double stepAngle = 360.0 / numTeeth;
                    double boundary = 90.0 / stepAngle;
                    int floorNum = (int)Math.Floor(boundary);
                    int ceilingNum = (int)Math.Ceiling(boundary);
                    double selfRotationAngle = 0;

                    if (gear_info.IndexOf(gp) == 0)
                    {
                        // the first pinion
                        if (floorNum == ceilingNum)
                        {
                            // the mating tooth is actually symmetric around the X axis
                            selfRotationAngle = stepAngle / 2;
                        }
                        else
                        {
                            double leftoverAngle = 90 - stepAngle * floorNum;
                            selfRotationAngle = stepAngle / 2 - leftoverAngle;
                        }

                        Gear newGear = new Gear(gp.center, gp.norm, gp.xDirection, numTeeth, 1, 20, gp.faceWidth, selfRotationAngle, true);
                        newGear.Generate();
                        models.Add(newGear);
                    }
                    else if(gear_info.IndexOf(gp) == 1)
                    {
                        // the first bull gear
                        if (floorNum == ceilingNum)
                        {
                            // the mating tooth is actually symmetric around the X axis
                            selfRotationAngle = stepAngle / 2;
                        }
                        else
                        {
                            double leftoverAngle = 90 - stepAngle * floorNum;
                            selfRotationAngle = stepAngle / 2 - leftoverAngle;
                        }
                        isGroove = true;

                        Gear newGear = new Gear(gp.center, gp.norm, gp.xDirection, numTeeth, 1, 20, gp.faceWidth, selfRotationAngle, true);
                        newGear.Generate();
                        models.Add(newGear);
                    }
                    else
                    {
                        if (isGroove)
                        {
                            selfRotationAngle = 90;
                            if (numTeeth % 2 == 1){ isGroove = true; }
                            else { isGroove = false; }
                        }
                        else
                        {
                            stepAngle = 360.0 / numTeeth;
                            selfRotationAngle = 90 - stepAngle / 2;

                            if (numTeeth % 2 == 1) { isGroove = false; }
                            else { isGroove = true; }
                        }
                        Gear newGear;
                        if (gear_info.IndexOf(gp) == gear_info.Count-1 && lastGearMovable==false)
                            newGear= new Gear(gp.center, gp.norm, gp.xDirection, numTeeth, 1, 20, gp.faceWidth, selfRotationAngle, false);
                        else
                            newGear = new Gear(gp.center, gp.norm, gp.xDirection, numTeeth, 1, 20, gp.faceWidth, selfRotationAngle, true);
                        newGear.Generate();
                        models.Add(newGear);
                    } 
                }
            }
            else
            {
                // turn control with spiral springs
                bool isGroove = false;
                foreach (GearParameter gp in gear_info)
                {
                    int numTeeth = (int)(gp.radius * 2);
                    double stepAngle = 360.0 / numTeeth;
                    double boundary = 90.0 / stepAngle;
                    int floorNum = (int)Math.Floor(boundary);
                    int ceilingNum = (int)Math.Ceiling(boundary);
                    double selfRotationAngle = 0;

                    if (gear_info.IndexOf(gp) != 0)
                    {
                        if(gear_info.IndexOf(gp) == 1)
                        {
                            // the first bull gear

                            if (floorNum == ceilingNum)
                            {
                                // the mating tooth is actually symmetric around the X axis
                                selfRotationAngle = stepAngle / 2;
                            }
                            else
                            {
                                double leftoverAngle = 90 - stepAngle * floorNum;
                                selfRotationAngle = stepAngle / 2 - leftoverAngle;
                            }
                            isGroove = true;

                            Gear newGear = new Gear(gp.center, gp.norm, gp.xDirection, (int)(gp.radius * 2), 1, 20, gp.faceWidth, selfRotationAngle, false);
                            newGear.Generate();
                            models.Add(newGear);
                        }
                        else
                        {
                            if (isGroove)
                            {
                                selfRotationAngle = 90;
                                if (numTeeth % 2 == 1) { isGroove = true; }
                                else { isGroove = false; }
                            }
                            else
                            {
                                stepAngle = 360.0 / numTeeth;
                                selfRotationAngle = 90 - stepAngle / 2;

                                if (numTeeth % 2 == 1) { isGroove = false; }
                                else { isGroove = true; }
                            }
                            Gear newGear;
                            if (gear_info.IndexOf(gp) == gear_info.Count - 1 && lastGearMovable == false)
                                newGear = new Gear(gp.center, gp.norm, gp.xDirection, numTeeth, 1, 20, gp.faceWidth, selfRotationAngle, false);
                            else
                                newGear = new Gear(gp.center, gp.norm, gp.xDirection, numTeeth, 1, 20, gp.faceWidth, selfRotationAngle, true);
                           
                            newGear.Generate();
                            models.Add(newGear);
                        }
                    }
                }

            }
            return models;
        }

        /// <summary>
        /// Genearte the springs
        /// </summary>
        /// <param name="gear_info">the information of the generated gear sets</param>
        /// <param name="body">the input selected body</param>
        /// <param name="controlType">the controlling method: press or turn</param>
        /// <param name="displacement">displacement with a helical spring and revolution angle with a spiral spring, the input value from the slider</param>
        /// <param name="energyLevel">the input value from the slider</param>
        /// <param name="dir">the move direction of the end-effector</param>
        /// <param name="lockPos"></param>
        /// <returns></returns>
        public List<Entity> genSprings(List<GearParameter> gear_info, Brep body, Curve skeleton,Vector3d mainAxis,int controlType, int displacement, int energyLevel, int dir, out Point3d lockPos)
        {
            List<Entity> models = new List<Entity>();
            lockPos = new Point3d();
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;

            if (controlType == 1)
            {
                // helical spring control
                double springPadThickness = 2;
                #region Step 1: find the spring position and length
                //TODO find position - Liang's 
                //Now just use first gear center with some offset as the spring end point
                Point3d helicalEndPoint = gear_info[0].center - mainAxis * (gear_info[0].radius + 0.3 + springPadThickness);
                //Find length - 1.5 times the available space
                double helicalLengthMultiplier = 1.5;//TODO adjust this value;
                Point3d skeletonStartPoint;
                if (new Vector3d(skeleton.PointAtNormalizedLength(0)) * mainAxis < new Vector3d(skeleton.PointAtNormalizedLength(1)) * mainAxis)
                    skeletonStartPoint = skeleton.PointAtNormalizedLength(0);
                else
                    skeletonStartPoint = skeleton.PointAtNormalizedLength(1);
                double availableSpace = (gear_info[0].center - skeletonStartPoint) * mainAxis - gear_info[0].radius-0.3-springPadThickness;
                double helicalLength = availableSpace * helicalLengthMultiplier;
                Point3d helicalStartPoint = helicalEndPoint - mainAxis * helicalLength;

                #endregion
                #region Step 2: construct spring and rack
                //TODO find spring parameters. Now just using some random fixed value.
                Helix helical = new Helix(helicalStartPoint, helicalEndPoint, 12, 1.2, 5, displacement/10.0, 0.5);
                models.Add(helical);
                #endregion
                #region Step 3: construct connecting structure

                #endregion

            }
            else
            {
                // spiral spring control

                #region Step 1: find the spring position and orientation

                Point3d springCen = new Point3d();

                Point3d firstGearCen = gear_info.ElementAt(0).center;
                Point3d secondGearCen = gear_info.ElementAt(1).center;
                Vector3d axelDir = firstGearCen - secondGearCen;
                axelDir.Unitize();

                Curve crossLineCrv = new Line(firstGearCen - axelDir * int.MaxValue, firstGearCen + axelDir * int.MaxValue).ToNurbsCurve();
                Curve[] crvs;
                Point3d[] pts;
                Rhino.Geometry.Intersect.Intersection.CurveBrep(crossLineCrv, body, myDoc.ModelAbsoluteTolerance, out crvs, out pts);

                Point3d ptEnd = new Point3d();
                Point3d ptStart = new Point3d();
                if ((pts[0] - pts[1]) / (pts[0].DistanceTo(pts[1])) == axelDir)
                {
                    ptEnd = pts[0] - axelDir * 1;
                    ptStart = pts[1] + axelDir * 1;
                }
                else
                {
                    ptEnd = pts[1] - axelDir * 1;
                    ptStart = pts[0] + axelDir * 1;
                }

                springCen = (ptEnd + firstGearCen) / 2;
                Point3d axisStart = ptEnd;
                Vector3d springDir = ptStart - ptEnd;
                springDir.Unitize();

                bool isCW = true;
                int predDir = 1;

                // determine the spring rotation direction based on the direction of the end rack
                if((gear_info.Count - 1)%2 == 1)
                {
                    if(dir != 1 && dir != 2)
                    {
                        // perpendicular down
                        isCW = false;
                    }
                    else
                    {
                        isCW = true;
                    }
                }
                else
                {
                    if (dir != 1 && dir != 2)
                    {
                        // perpendicular down
                        isCW = true;
                    }
                    else
                    {
                        isCW = false;
                    }
                }

                Spiral spiralSpring = new Spiral(axisStart, springDir, springCen, gear_info.ElementAt(1).radius + 0.5, isCW, displacement, true, energyLevel);

                models.Add(spiralSpring);

                #endregion

                #region Step 2: generate the lock position



                #endregion
            }

            return models;
        }
        public static List<Brep> cutModel(Brep model, Curve skeleton, double t1,double t2,Vector3d v,RhinoDoc myDoc)
        {
            Plane pl1, pl2;
            Plane p1Reverse, p2Reverse;
            List<Brep> brepCut = new List<Brep>();
            if (t1 >= t2)
            {
                p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), v);
                p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), -v);

                pl1 = new Plane(skeleton.PointAtNormalizedLength(t1), -v);
                pl2 = new Plane(skeleton.PointAtNormalizedLength(t2), v);
            }
            else
            {
                p1Reverse = new Plane(skeleton.PointAtNormalizedLength(t2), v);
                p2Reverse = new Plane(skeleton.PointAtNormalizedLength(t1), -v);

                pl1 = new Plane(skeleton.PointAtNormalizedLength(t2), -v);
                pl2 = new Plane(skeleton.PointAtNormalizedLength(t1), v);
            }

            Brep[] Cut_Brep1 = model.Trim(pl1, myDoc.ModelAbsoluteTolerance);
            Brep Brep1 = Cut_Brep1[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            brepCut.Add(Brep1);

            Brep[] Cut_Brep1rest = model.Trim(p1Reverse, myDoc.ModelAbsoluteTolerance);
            Brep BrepRest = null;
            try
            {
                BrepRest = Cut_Brep1rest[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            }
            catch
            {
                BrepRest = model;
            }
            Brep[] Cut_Brep2 = BrepRest.Trim(pl2, myDoc.ModelAbsoluteTolerance);
            Brep Brep2 = null;
            try
            {
                Brep2 = Cut_Brep2[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            }
            catch
            {
                Brep2 = BrepRest;
            }

            Brep[] Cut_Brep3 = BrepRest.Trim(p2Reverse, myDoc.ModelAbsoluteTolerance);
            Brep Brep3 = Cut_Brep3[0].CapPlanarHoles(myDoc.ModelAbsoluteTolerance);
            brepCut.Add(Brep3);
            brepCut.Add(Brep2);
            return brepCut;
        }
        public static Brep getInnerCavity(List<Brep> brepCut,Vector3d selectedAxisVector)
        {
            Brep innerCavity;
            BoxLike b = new BoxLike(brepCut[1], selectedAxisVector);
            double volumn = 0;
            Brep result1 = null;
            Cylinder result2 = Cylinder.Unset;
            Brep b2 = null;
            double v_box = 0.0, v_cylinder = 0.0;

            // Calculate the volume of the inner box
            for (double i = 0.2; i <= 0.8; i += 0.1)
            {
                if (b.GetInnerEmptySpaceBox(i))
                {
                    BoundingBox bbox = b.InnerEmptySpaceBbox;
                    if (volumn < bbox.Volume)
                    {
                        volumn = bbox.Volume;
                        result1 = b.InnerEmptySpaceBoxBrep;
                        result1.Transform(b.RotateBack);
                        v_box = result1.GetVolume();
                    }
                }
            }

            // Calculate the volume of the inner cylinder 
            if (b.GetInnerEmptySpaceCylinder())
            {
                Cylinder c = b.InnerEmptyCylinder;
                //result2 = c.ToBrep(true,true);
                result2 = c;
                b2 = result2.ToBrep(true, true);
                b2.Transform(b.RotateBack);
                v_cylinder = b2.GetVolume();
                //DA.SetData(2, b2);
            }

            if (v_box >= v_cylinder)
                innerCavity = result1;
            else
                innerCavity = b2;
            //TODO check if this translation matters
            //Transform cavityTranslation = Transform.Translation(brepCut[1].GetBoundingBox(true).Center - innerCavity.GetBoundingBox(true).Center);
            //innerCavity.Transform(cavityTranslation);
            return innerCavity;
        }
    }
}
