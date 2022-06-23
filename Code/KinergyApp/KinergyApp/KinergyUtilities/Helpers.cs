using Kinergy.Geom;
using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using Kinergy.Relationship;

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
        public List<Entity> genAxelsStoppers(List<GearParameter> gear_info, Brep body, int controlType, double clearance=0.25)
        {
            List<Entity> models = new List<Entity>();
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            var sweep = new SweepOneRail();
            sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
            sweep.ClosedSweep = false;
            sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

            double rad1 = 2;
            double rad2 = 2.7;
            double rad3 = 3.5;

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
                        Spacer sp1 = new Spacer(gearCen - initialDir * offset1, 1, rad2, rad3, (-initialDir));
                        models.Add(sp1);

                        if(idx == gear_info.Count - 1)
                        {
                            // the last gear is a pinion
                            Spacer sp2 = new Spacer(gearCen + axelDir * offset1, 1, rad2, rad3, axelDir);
                            models.Add(sp2);
                        }
                    }
                    else
                    {
                        // bull gear
                        Spacer sp2 = new Spacer(gearCen - axelDir * offset2, 1, rad2, rad3, -axelDir);
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
                        Spacer sp1 = new Spacer(gearCen - axelDir * clearance, 1, rad2, rad3, -axelDir);
                        models.Add(sp1);

                        if (idx == gear_info.Count - 1)
                        {
                            // the last gear is a pinion
                            Spacer sp2 = new Spacer(gearCen + axelDir * offset, 1, rad2, rad3, axelDir);
                            models.Add(sp2);
                        }
                    }
                    else
                    {
                        // bull gear
                        if (idx != 1)
                        {
                            Spacer sp2 = new Spacer(gearCen + axelDir * offset, 1, rad2, rad3, axelDir);
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

                    Shaft lockAxelShaftDisc = new Shaft(shaftStartPt, 2, 3.8, firstGearDir);
                    models.Add(lockAxelShaftDisc);

                    lockAxelShaft.SetName("SpiralShaft");

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

                    Point3d shaftStartPt = lockLinePts[0] + firstGearDir * 2.75;//Xia's note:reduced this value by 0.25 (from 2.75 to 2.5) to make tighter//Revoked
                    Point3d shaftEndPt = lockPtEnd;
                    Shaft lockAxelShaft = new Shaft(shaftStartPt, shaftStartPt.DistanceTo(shaftEndPt), rad1, firstGearDir);

                    lockAxelShaft.SetName("SpiralShaft");

                    models.Add(lockAxelShaft);

                    Shaft lockAxelShaftDisc = new Shaft(shaftStartPt, 2, 4, firstGearDir);//Xia's note: expanded shaft disc radius by 0.2 (3.8 to 4) to make tighter
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
                Gear prevGear = null;
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
                        prevGear = newGear;
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
                        _ = new Fixation(newGear, prevGear);
                        prevGear = newGear;
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
                        if(gp.PinionOrBull==1)
                            _ = new Fixation(newGear, prevGear);
                        else
                            _ = new Engagement(newGear, prevGear);
                        prevGear = newGear;
                    } 
                }
            }
            else
            {
                // turn control with spiral springs
                Gear prevGear = null;
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
                            prevGear = newGear;
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
                            if (gp.PinionOrBull == 1)
                                _ = new Fixation(newGear, prevGear);
                            else
                                _ = new Engagement(newGear, prevGear);
                            prevGear = newGear;
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
        public List<Entity> genSprings(List<GearParameter> gear_info, Brep body, Curve skeleton,Vector3d mainAxis,int controlType, int displacement, int energyLevel, bool dirCtrl, out List<Point3d> lockPos, out bool lockNorm, out Vector3d lockDir, out Brep helicalSpringSocket, Gear firstPinion, Point3d eePos = new Point3d())
        {
            List<Entity> models = new List<Entity>();
            lockPos = new List<Point3d>();
            lockNorm = false;
            lockDir = new Vector3d();
            helicalSpringSocket = null;
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;

            if (controlType == 1)
            {
                var sweep = new SweepOneRail();
                sweep.AngleToleranceRadians = myDoc.ModelAngleToleranceRadians;
                sweep.ClosedSweep = false;
                sweep.SweepTolerance = myDoc.ModelAbsoluteTolerance;

                // helical spring control
                double springPadThickness = 2;
                double gearThickness = 3.6;
                double rkTeethHeight = 2.25;

                #region Step 1: find the spring position and length
                //Now just use first gear center with some offset as the spring end point
                Vector3d shaftDir = gear_info[0].center - gear_info[1].center;
                shaftDir.Unitize();
                //Find the vector that is orthogonal to both the mainAxis and the shaftDir
                Vector3d rkDir = Vector3d.CrossProduct(shaftDir, mainAxis);
                rkDir.Unitize();

                //if ((gear_info.Count - 1) % 2 == 1)
                //{
                //    if (dir != 1 && dir != 2)
                //    {
                //        // perpendicular down
                //        rkDir = -rkDir;
                //        lockNorm = false;
                //    }
                //    else
                //    {
                //        rkDir = rkDir;
                //        lockNorm = true;
                //    }
                //}
                //else
                //{
                //    if (dir != 1 && dir != 2)
                //    {
                //        // perpendicular down
                //        rkDir = rkDir;
                //        lockNorm = true;
                //    }
                //    else
                //    {
                //        rkDir = -rkDir;
                //        lockNorm = false;
                //    }
                //}

                if (dirCtrl)
                {
                    rkDir = rkDir;
                    lockNorm = false;
                }
                else
                {
                    rkDir = -rkDir;
                    lockNorm = true;
                }

                Point3d springRkGrConPt = new Point3d();
                if (lockNorm)
                    springRkGrConPt = gear_info[0].center - rkDir * (gear_info[0].radius + 0.6 + rkTeethHeight/2) + shaftDir * gearThickness / 2;
                else
                    springRkGrConPt = gear_info[0].center - rkDir * (gear_info[0].radius + 0.6 + rkTeethHeight / 2) - shaftDir * gearThickness / 2;

                #region compute spring start point, spring end point, spring length

                //Find length - 1.5 times the available space
                double helicalLengthMultiplier = 1.5;//TODO adjust this value;
                Point3d skeletonStartPoint;
                if (new Vector3d(skeleton.PointAtNormalizedLength(0)) * mainAxis < new Vector3d(skeleton.PointAtNormalizedLength(1)) * mainAxis)
                    skeletonStartPoint = skeleton.PointAtNormalizedLength(0);
                else
                    skeletonStartPoint = skeleton.PointAtNormalizedLength(1);

                double springGearGap = 4;
                double availableSpace = gear_info[0].center.DistanceTo(skeletonStartPoint) - gear_info[1].radius - springGearGap;
                double helicalLength = availableSpace * helicalLengthMultiplier;
                Point3d helicalStartPoint = springRkGrConPt - mainAxis * (helicalLength + gear_info[1].radius + springGearGap);
                Point3d helicalEndPoint = helicalStartPoint + mainAxis * helicalLength;

                #endregion
                #endregion

                #region Step 2: calculate spring parameters

                double min_wire_diamter = 2;
                double min_coil_num = 2;
                double wireRadius = 2;
                int roundNum = 0;
                double springRadius = 0;

                double maxDisp = Math.Max(helicalLength - min_wire_diamter * min_coil_num, min_coil_num * 0.6);
                double dis = (displacement * 0.05 + 0.5) * maxDisp / helicalLength;     // convert the input displacement level into percentage

                // Parse the energy based on E ~= d^4/n * x^2
                double x = dis * helicalLength;
                double energy = energyLevel / 10.0;

                Curve springCrossLineCrv = new Line(helicalEndPoint - shaftDir * int.MaxValue, helicalEndPoint + shaftDir * int.MaxValue).ToNurbsCurve();
                Curve[] springBodyCrvs;
                Point3d[] springBodyPts;
                Rhino.Geometry.Intersect.Intersection.CurveBrep(springCrossLineCrv, body, myDoc.ModelAbsoluteTolerance, out springBodyCrvs, out springBodyPts);

                springRadius = (springBodyPts[0].DistanceTo(springBodyPts[1]) - 2) / 2 - wireRadius - gearThickness / 2;

                #endregion

                #region Step 3: construct spring and rack
                Helix helical = new Helix(helicalStartPoint, helicalEndPoint, springRadius, wireRadius, roundNum, dis, energy);
                helical.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == helical.Model.SolidOrientation)
                    helical.Model.Flip();
                models.Add(helical);

                // create the cylinder for deduction
                double springSocketRadius = springRadius + wireRadius + 0.6;
                Line socketTraj = new Line(helicalEndPoint - mainAxis * wireRadius, helicalEndPoint - mainAxis * wireRadius - mainAxis * helicalLength * 5);
                Curve socketCrv = socketTraj.ToNurbsCurve();
                helicalSpringSocket = Brep.CreatePipe(socketCrv, springSocketRadius, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
                helicalSpringSocket.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == helicalSpringSocket.SolidOrientation)
                    helicalSpringSocket.Flip();

                #endregion

                #region Step 4: construct the handler, the central rack, and the holed base

                #region generate the handler
                double handlerThickness = wireRadius*2;
                double handlerR = springRadius + wireRadius;
                Point3d handlerPos = helicalStartPoint + mainAxis*handlerThickness/2;
                Line handlerTraj = new Line(handlerPos, handlerPos - mainAxis * handlerThickness);
                Curve handlerCrv = handlerTraj.ToNurbsCurve();
                Brep handlerBrep = Brep.CreatePipe(handlerCrv, handlerR, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
                handlerBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == handlerBrep.SolidOrientation)
                    handlerBrep.Flip();

                Entity helicalHandler = new Entity(handlerBrep, false, "HelicalHandler");
                models.Add(helicalHandler);

                //ToDo: add the relationship between the handler and the spring end

                #endregion

                #region generate the holded base

                double baseThickness = wireRadius*2;
                double baseR = springRadius + wireRadius;
                Point3d basePos = helicalEndPoint - mainAxis*baseThickness/2;
                Line baseTraj = new Line(basePos, basePos + mainAxis * baseThickness);
                Curve baseCrv = baseTraj.ToNurbsCurve();
                Brep baseBrep = Brep.CreatePipe(baseCrv, baseR, false, PipeCapMode.Flat, false, myDoc.ModelAbsoluteTolerance, myDoc.ModelAngleToleranceRadians)[0];
                //myDoc.Objects.AddBrep(baseBrep);
                baseBrep.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == baseBrep.SolidOrientation)
                    baseBrep.Flip();

                //Entity helicalBase = new Entity(baseBrep, false, "HelicalBase");
                //models.Add(helicalBase);

                #endregion

                #region generate the central rack that mate with the first gear in the geartrain

                double rackOverflowLen = 4;
                double rackLen = springRkGrConPt.DistanceTo(helicalStartPoint) + rackOverflowLen + Math.PI * 2;
                Vector3d rackDir = springRkGrConPt - helicalStartPoint;
                rackDir.Unitize();
                Point3d rackStartPt = new Point3d();
                if(lockNorm)
                    rackStartPt = helicalStartPoint - shaftDir * gearThickness / 2 - mainAxis * wireRadius;
                else
                    rackStartPt = helicalStartPoint + shaftDir * gearThickness / 2 - mainAxis * wireRadius;

                Point3d rackEndPt = rackStartPt + rackDir * rackLen;
                Point3d rackCen = (rackStartPt + rackEndPt) / 2;
                double gearModule = 1;
                double gearPressureAngle = 20;
                Rack cenRack = new Rack(rackCen, rackDir, rkDir, rackLen, gearModule, gearThickness, shaftDir, springPadThickness, gearPressureAngle);

                if (lockNorm)
                    cenRack.MoveAndEngage(firstPinion, -rkDir);
                else
                    cenRack.MoveAndEngage(firstPinion, rkDir);

                cenRack.Model.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == cenRack.Model.SolidOrientation)
                    cenRack.Model.Flip();

                #endregion

                #region prepare the holes for boolean difference

                // add the barrel

                double barrelThickness = 1;
                double protrusionLen = 20;
                double tolerance = 0.4;
                //Find the closest point on base plane to rack center.
                Point3d baseRackPt = basePos-(basePos - rackStartPt) + (basePos - rackStartPt) * mainAxis * mainAxis;
                //Point3d b_pt1 = baseRackPt - shaftDir * (tolerance + barrelThickness) + rkDir * (rkTeethHeight + tolerance + barrelThickness);
                //Point3d b_pt2 = baseRackPt + shaftDir * (gearThickness + tolerance + barrelThickness) + rkDir * (rkTeethHeight + tolerance + barrelThickness);
                //Point3d b_pt3 = baseRackPt + shaftDir * (gearThickness + tolerance + barrelThickness) - rkDir * (springPadThickness + tolerance + barrelThickness);
                //Point3d b_pt4 = baseRackPt - shaftDir * (tolerance + barrelThickness) - rkDir * (springPadThickness + tolerance + barrelThickness);
                //Point3d b_pt5 = b_pt1;
                //Change the barrel to just upper half and 2 sides
                Point3d b_pt1 = baseRackPt - shaftDir * (tolerance + barrelThickness) ;
                Point3d b_pt2 = baseRackPt - shaftDir * (tolerance + barrelThickness) - rkDir * (springPadThickness + tolerance + barrelThickness);
                Point3d b_pt3 = baseRackPt + shaftDir * (gearThickness + tolerance + barrelThickness) - rkDir * (springPadThickness + tolerance + barrelThickness);
                Point3d b_pt4 = baseRackPt + shaftDir * (gearThickness + tolerance + barrelThickness);
                Point3d b_pt5 = baseRackPt + shaftDir * (gearThickness + tolerance );
                Point3d b_pt6 = baseRackPt + shaftDir * (gearThickness + tolerance) - rkDir * (springPadThickness + tolerance);
                Point3d b_pt7 = baseRackPt - shaftDir * (tolerance ) - rkDir * (springPadThickness + tolerance);
                Point3d b_pt8 = baseRackPt - shaftDir * (tolerance );
                Point3d b_pt9 = b_pt1;

                List<Point3d> barrelConnector = new List<Point3d>();
                barrelConnector.Add(b_pt1);
                barrelConnector.Add(b_pt2);
                barrelConnector.Add(b_pt3);
                barrelConnector.Add(b_pt4);
                barrelConnector.Add(b_pt5);
                barrelConnector.Add(b_pt6);
                barrelConnector.Add(b_pt7);
                barrelConnector.Add(b_pt8);
                barrelConnector.Add(b_pt9);
                Polyline barrelRect = new Polyline(barrelConnector);
                Curve barrelRectCrv = barrelRect.ToNurbsCurve();

                Line barrelLn = new Line( baseRackPt - mainAxis * 10,rackEndPt - mainAxis * 5);
                Curve barrelCrv = barrelLn.ToNurbsCurve();

                Brep[] barrelBreps = sweep.PerformSweep(barrelCrv, barrelRectCrv);
                Brep barrelBrep = barrelBreps[0];
                Brep barrel = barrelBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                barrel.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == barrel.SolidOrientation)
                    barrel.Flip();
                //myDoc.Objects.AddBrep(barrel);




                // open the base hole
                Point3d rkDeductPt = rackStartPt;
                Point3d pt1 = rkDeductPt - shaftDir * tolerance + rkDir * ( rkTeethHeight + tolerance );
                Point3d pt2 = rkDeductPt + shaftDir * (gearThickness + tolerance) + rkDir * (rkTeethHeight + tolerance);
                Point3d pt3 = rkDeductPt + shaftDir * (gearThickness + tolerance) - rkDir * (springPadThickness + tolerance);
                Point3d pt4 = rkDeductPt - shaftDir * tolerance - rkDir * (springPadThickness + tolerance);
                Point3d pt5 = pt1;

                List<Point3d> rkDeductConnector = new List<Point3d>();
                rkDeductConnector.Add(pt1);
                rkDeductConnector.Add(pt2);
                rkDeductConnector.Add(pt3);
                rkDeductConnector.Add(pt4);
                rkDeductConnector.Add(pt1);

                Polyline rkDeductRect = new Polyline(rkDeductConnector);
                Curve rkDeductRectCrv = rkDeductRect.ToNurbsCurve();

                Line rkDeductLn = new Line(rackStartPt, rackEndPt);
                Curve rkDeductCrv = rkDeductLn.ToNurbsCurve();

                Brep[] rkDeductBreps = sweep.PerformSweep(rkDeductCrv, rkDeductRectCrv);
                Brep rkDeductBrep = rkDeductBreps[0];
                Brep rkDeduct = rkDeductBrep.CapPlanarHoles(myDoc.ModelAbsoluteTolerance);

                rkDeduct.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == rkDeduct.SolidOrientation)
                    rkDeduct.Flip();

                Brep rkDeductDup = rkDeduct.DuplicateBrep();

                Brep baseBrepFinal = Brep.CreateBooleanDifference(baseBrep, rkDeduct, myDoc.ModelAbsoluteTolerance)[0];
                baseBrepFinal.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == baseBrepFinal.SolidOrientation)
                    baseBrepFinal.Flip();

                //Brep barrelBrepFinal = Brep.CreateBooleanDifference(barrel, rkDeductDup, myDoc.ModelAbsoluteTolerance)[0];
                Brep barrelBrepFinal = barrel;
                barrelBrepFinal.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == barrelBrepFinal.SolidOrientation)
                    barrelBrepFinal.Flip();

                baseBrepFinal = Brep.CreateBooleanUnion(new List<Brep> { baseBrepFinal, barrelBrepFinal }, myDoc.ModelAbsoluteTolerance)[0];
                baseBrepFinal.Faces.SplitKinkyFaces(RhinoMath.DefaultAngleTolerance, true);
                if (BrepSolidOrientation.Inward == baseBrepFinal.SolidOrientation)
                    baseBrepFinal.Flip();

                Entity baseHoledEntity = new Entity(baseBrepFinal, false, "HelicalSpringBase");
                models.Add(baseHoledEntity);
                models.Add(cenRack);

                #endregion
                #endregion

            }
            else
            {
                // spiral spring control

                #region Step 1: find the spring position and orientation

                Point3d springCen = new Point3d();

                Point3d firstGearCen = gear_info.ElementAt(0).center;
                Point3d secondGearCen = gear_info.ElementAt(1).center;
                //Vector3d axelDir = firstGearCen - secondGearCen;
                Vector3d axelDir = gear_info.ElementAt(0).norm;
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

                // Project the end-effector's position to the first shaft 
                Vector3d sDir = new Vector3d();
                if (eePos != new Point3d())
                {
                    double t = -1;
                    crossLineCrv.ClosestPoint(eePos, out t);
                    Point3d eePos_proj = crossLineCrv.PointAt(t);
                    sDir = secondGearCen - eePos_proj;
                    sDir.Unitize();
                    eePos_proj = eePos_proj + sDir * 7;

                    //myDoc.Objects.AddPoint(eePos_proj);
                    //myDoc.Views.Redraw();

                    //myDoc.Objects.AddPoint(secondGearCen);
                    //myDoc.Views.Redraw();

                    if (secondGearCen.DistanceTo(eePos_proj) > 13)
                    {
                        springCen = eePos_proj + sDir * 0.6;
                    }
                    else
                    {
                        return new List<Entity>();
                    }
                }
                else
                {
                    springCen = (ptEnd + firstGearCen) / 2;
                    sDir = firstGearCen - ptEnd;
                    sDir.Unitize();
                    springCen = springCen - sDir * 3;
                }
                
                Point3d axisStart = ptEnd;
                Vector3d springDir = ptStart - ptEnd;
                springDir.Unitize();

                bool isCW = true;
                int predDir = 1;

                isCW = dirCtrl;
                // determine the spring rotation direction based on the direction of the end rack
                //if((gear_info.Count - 1)%2 == 1)
                //{
                //    if(dir != 1 && dir != 2)
                //    {
                //        // perpendicular down
                //        isCW = false;
                //    }
                //    else
                //    {
                //        isCW = true;
                //    }
                //}
                //else
                //{
                //    if (dir != 1 && dir != 2)
                //    {
                //        // perpendicular down
                //        isCW = true;
                //    }
                //    else
                //    {
                //        isCW = false;
                //    }
                //}

                //Spiral spiralSpring = new Spiral(body, axisStart, springDir, springCen, (gear_info.ElementAt(1).radius + gear_info.ElementAt(0).radius) * 0.8, isCW, displacement, true, energyLevel, -Math.PI/4);
                Spiral spiralSpring = new Spiral(body, axisStart, springDir, springCen, (gear_info.ElementAt(1).radius + gear_info.ElementAt(0).radius) * 0.7, isCW, displacement, true, energyLevel, 0);

                models.Add(spiralSpring);

                #endregion

                #region Step 2: generate the lock position

                lockNorm = isCW;
                lockDir = sDir;

                Point3d lockCen = springCen + sDir * (3 + 6 + 1.8);

                // find the vector that is orthogonal to both sDir and mainAxis
                Vector3d lockV = Vector3d.CrossProduct(sDir, mainAxis);
                lockV.Unitize();
                Curve lockCrossLineCrv = new Line(lockCen - lockV * int.MaxValue, lockCen + lockV * int.MaxValue).ToNurbsCurve();
                Curve[] lockCrvs;
                Point3d[] lockPts;
                Rhino.Geometry.Intersect.Intersection.CurveBrep(lockCrossLineCrv, body, myDoc.ModelAbsoluteTolerance, out lockCrvs, out lockPts);

                lockPos.Add(lockPts[0]);
                lockPos.Add(lockPts[1]);
                lockPos.Add(lockCen);

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
            for (double i = 0.1; i <= 0.9; i += 0.05)
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

            if (v_box >= v_cylinder*4/Math.PI)
                innerCavity = result1;
            else
                innerCavity = b2;

            //innerCavity = result1;
            //TODO check if this translation matters
            //Transform cavityTranslation = Transform.Translation(brepCut[1].GetBoundingBox(true).Center - innerCavity.GetBoundingBox(true).Center);
            //innerCavity.Transform(cavityTranslation);
            return innerCavity;
        }
    }
}
