using Rhino;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Kinergy.Geom;
using System.Threading.Tasks;

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

            double rad = 1.5;

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
                        Curve lineCrv = new Line(ptStart, ptEnd).ToNurbsCurve();
                        Shaft axelShaft = new Shaft(ptStart, ptStart.DistanceTo(ptEnd), rad, axelDir);
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
                        Spacer sp1 = new Spacer(gearCen - initialDir * offset2, 1, rad, 3, (-initialDir));
                        models.Add(sp1);

                        if(idx == gear_info.Count - 1)
                        {
                            // the last gear is a pinion
                            Spacer sp2 = new Spacer(gearCen + axelDir * (offset2 + gp.faceWidth), 1, rad, 3, axelDir);
                            models.Add(sp2);
                        }
                    }
                    else
                    {
                        // bull gear
                        Spacer sp2 = new Spacer(gearCen - axelDir * offset2, 1, rad, 3, -axelDir);
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
                        Curve lineCrv = new Line(ptStart, ptEnd).ToNurbsCurve();
                        Shaft axelShaft = new Shaft(ptStart, ptStart.DistanceTo(ptEnd), rad, axelDir);
                        models.Add(axelShaft);
                    }
                    #endregion

                    #region generate the spacers

                    double offset = gp.faceWidth + clearance;

                    if (idx % 2 == 0)
                    {
                        // pinion gear

                        // add the first spacer
                        Spacer sp1 = new Spacer(gearCen - axelDir * clearance, 1, rad, 3, -axelDir);
                        models.Add(sp1);

                        if (idx == gear_info.Count - 1)
                        {
                            // the last gear is a pinion
                            Spacer sp2 = new Spacer(gearCen + axelDir * offset, 1, rad, 3, axelDir);
                            models.Add(sp2);
                        }
                    }
                    else
                    {
                        // bull gear
                        if (idx != 1)
                        {
                            Spacer sp2 = new Spacer(gearCen + axelDir * offset, 1, rad, 3, axelDir);
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
                if ((lockLinePts[0] - lockLinePts[1]) / (lockLinePts[0].DistanceTo(lockLinePts[1])) == firstGearDir)
                {
                    lockPtEnd = lockLinePts[0] + firstGearDir * 7;
                    lockPtStart = lockLinePts[1] + firstGearDir * 4.5;
                    Socket lockShaftSocket = new Socket(lockPtStart, firstGearDir);
                    models.Add(lockShaftSocket);


                    Point3d shaftStartPt = lockLinePts[1] + firstGearDir * 2.75;
                    Point3d shaftEndPt = lockPtEnd;
                    Shaft lockAxelShaft = new Shaft(shaftStartPt, shaftStartPt.DistanceTo(shaftEndPt), rad, firstGearDir);
                    models.Add(lockAxelShaft);

                    Shaft lockAxelShaftDisc = new Shaft(shaftStartPt, 1.5, 3.8, firstGearDir);
                    models.Add(lockAxelShaft);

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
                    Shaft lockAxelShaft = new Shaft(shaftStartPt, shaftStartPt.DistanceTo(shaftEndPt), rad, firstGearDir);
                    models.Add(lockAxelShaft);

                    Shaft lockAxelShaftDisc = new Shaft(shaftStartPt, 1.5, 3.8, firstGearDir);
                    models.Add(lockAxelShaft);

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
        public List<Entity> genGears(List<GearParameter> gear_info, int controlType, double clearance)
        {
            List<Entity> models = new List<Entity>();
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;

            if(controlType == 1)
            {
                // press control with helix springs
                foreach(GearParameter gp in gear_info)
                {
                    Gear newGear = new Gear(gp.center, gp.norm, gp.xDirection, (int)(gp.radius * 2), 1, 20, gp.faceWidth, 0, true);
                    newGear.Generate(); 
                    models.Add(newGear);
                }
            }
            else
            {
                // turn control with spiral springs
                foreach (GearParameter gp in gear_info)
                {
                    if(gear_info.IndexOf(gp) != 0)
                    {
                        if(gear_info.IndexOf(gp) == 1)
                        {
                            Gear newGear = new Gear(gp.center, gp.norm, gp.xDirection, (int)(gp.radius * 2), 1, 20, gp.faceWidth, 0, false);
                            newGear.Generate();
                            models.Add(newGear);
                        }
                        else
                        {
                            Gear newGear = new Gear(gp.center, gp.norm, gp.xDirection, (int)(gp.radius * 2), 1, 20, gp.faceWidth, 0, true);
                            newGear.Generate();
                            models.Add(newGear);
                        }
                    }
                }

            }
            return models;
        }
    }
}
