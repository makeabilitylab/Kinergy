using System;
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
        private int _speed;  // the range on the interface is 0-9
        private int _distance;    // the range on the interface is 0-9
        private int _energy; //  the range on the interface is 0-9
        private Vector3d _direction = Vector3d.Unset;
        private bool _addLock;
        private RhinoDoc _myDoc;
        private int _inputType;
        private Cylinder _innerSpce;
        private double _skeletonLen;
        private Point3d _motorRefPt;

        private Curve _skeleton = null;
        private List<Shape> _modelCut;
        private List<Lock> _locks;
        private Helix _spring;

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
        double t1 = 0, t2 = 0;

        public int ShaftNum { get => _shaftNum; set => _shaftNum = value; }

        public ContinuousTranslation(Brep Model, Vector3d Direction, Brep innerCylinder, Point3d motionCtrlPt, int speed, int dis, int eneryg, int InputType)
        {
            _model = Model;
            _speed = speed;
            _energy = eneryg;
            _distance = dis;
            _direction = Direction;
            _modelCut = new List<Shape>();
            _myDoc = RhinoDoc.ActiveDoc;
            _locks = new List<Lock>();
            _inputType = InputType;
            _motorRefPt = motionCtrlPt;

            _innerSpce = GetCylinder(innerCylinder, Direction);
            BoxLike currB = new BoxLike(_model, _direction);

            _skeleton = currB.Skeleton;
            _skeleton.Transform(currB.RotateBack);
            _skeletonLen = _skeleton.PointAtNormalizedLength(0).DistanceTo(_skeleton.PointAtNormalizedLength(1));

            _shaftNum = 0;
            _r_shaft_num_List = new List<List<int>>();
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
            b1 = B1;
            b2 = B2;
            b3 = B3;
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

        public void GenerateGearTrain(double finalGearPosRatio, Point3d eeCen, int speed_input, Vector3d kineticUnitDir, Vector3d axelDir)
        {
            double shaftNum = 0;
            double gearRadius = 0;
            CalculateShaftNumAndGearRadius(speed_input, out shaftNum, out gearRadius);
            Vector3d gearDirection = direction;

            double dis = clearance + 4.5;

            double t = 0;
            _skeleton.ClosestPoint(eeCen, out t);
            if (t > 0.5)
            {
                dis = -dis;
            }
            else
            {
                gearDirection = -direction;
            }
            Vector3d gearDevDir = new Vector3d();
            if(finalGearPosRatio > 0.5)
            {
                gearDevDir = axelDir;
            }
            else
            {
                gearDevDir = -axelDir;
            }

            //Point3d pinionPosPre = eeCen - axelDir * (0.5 - finalGearPosRatio);
            Point3d pinionPosPre = eeCen;
            Point3d pinionPos = pinionPosPre + dis * gearDirection - gearDirection * (shaftNum - 1) * (4.5 + clearance + gearRadius);

            #region create the gears and stoppers on the first shaft (far away from the end gear)

            bool isGroove = false;
            int numTeeth = 0;
            double stepAngle = 0;
            double boundary = 0;
            int floorNum = 0;
            int ceilingNum = 0;
            double selfRotationAngle = 0;


            if (_inputType == 1)
            {
                #region press control

                numTeeth = 9;
                stepAngle = 360.0 / numTeeth;
                boundary = 90.0 / stepAngle;
                floorNum = (int)Math.Floor(boundary);
                ceilingNum = (int)Math.Ceiling(boundary);
                selfRotationAngle = 0;

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
                

                // create the pinion gear
                Point3d pinionGearPos = pinionPos - gearDevDir * gearFaceWidth / 2;
                Gear pinionGear = new Gear(pinionGearPos, gearDevDir, gearDirection, numTeeth, gearModule, gearPressureAngle, gearFaceWidth + clearance, selfRotationAngle, true);
                Spacer sp_middle1 = new Spacer(pinionGearPos - gearDevDir * clearance, 1, shaftRadius, 3, -gearDevDir);

                // create the bull gear
                int bullGearTeethNum = (int)(2 * gearRadius / gearModule);

                numTeeth = bullGearTeethNum;
                stepAngle = 360.0 / numTeeth;
                boundary = 90.0 / stepAngle;
                floorNum = (int)Math.Floor(boundary);
                ceilingNum = (int)Math.Ceiling(boundary);
                selfRotationAngle = 0;

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

                Point3d bullGearPos = pinionGearPos + gearDevDir * (gearFaceWidth + clearance);
                Gear bullGear = new Gear(bullGearPos, gearDevDir, gearDirection, bullGearTeethNum, gearModule, gearPressureAngle, gearFaceWidth, selfRotationAngle, true);
                Spacer sp_middle2 = new Spacer(bullGearPos + gearDevDir * (clearance + gearFaceWidth), 1, shaftRadius, 3, gearDevDir);

                // get two intersecting points on the modle to determine the shaft length and the starting point
                Curve shaftMidCrv = new Line(pinionGearPos - axelDir * 10000, pinionGearPos + axelDir * 10000).ToNurbsCurve();
                Point3d[] interMidPts;
                Intersection.CurveBrep(shaftMidCrv, _model, myDoc.ModelAbsoluteTolerance, out _, out interMidPts);

                Vector3d shrinkMidVector0 = interMidPts[1] - interMidPts[0];
                Vector3d shrinkMidVector1 = interMidPts[0] - interMidPts[1];
                shrinkMidVector0.Unitize();
                shrinkMidVector1.Unitize();

                Point3d realInterMidPt0 = interMidPts[0] + shrinkMidVector0;
                Point3d realInterMidPt1 = interMidPts[1] + shrinkMidVector1;

                double shaftMidLen = realInterMidPt0.DistanceTo(realInterMidPt1);
                Shaft shaftMid = new Shaft(realInterMidPt0, shaftMidLen, shaftRadius, axelDir);

                EntityList.Add(bullGear);
                EntityList.Add(pinionGear);
                EntityList.Add(sp_middle1);
                EntityList.Add(sp_middle2);
                EntityList.Add(shaftMid);

                #endregion

                pinionPos = bullGearPos;
            }
            else
            {
                #region turn control

                // create the bull gear
                int bullGearTeethNum = (int)(2 * gearRadius / gearModule);
                numTeeth = bullGearTeethNum;
                stepAngle = 360.0 / numTeeth;
                boundary = 90.0 / stepAngle;
                floorNum = (int)Math.Floor(boundary);
                ceilingNum = (int)Math.Ceiling(boundary);
                selfRotationAngle = 0;

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

                Point3d bullGearPos = pinionPos - gearDevDir * gearFaceWidth / 2;
                Gear bullGear = new Gear(bullGearPos, gearDevDir, gearDirection, bullGearTeethNum, gearModule, gearPressureAngle, gearFaceWidth, selfRotationAngle, false);

                // get two intersecting points on the modle to determine the shaft length and the starting point
                Curve shaftMidCrv = new Line(bullGearPos - axelDir * 10000, bullGearPos + axelDir * 10000).ToNurbsCurve();
                Point3d[] interMidPts;
                Intersection.CurveBrep(shaftMidCrv, _model, myDoc.ModelAbsoluteTolerance, out _, out interMidPts);

                Vector3d shrinkMidVector0 = interMidPts[1] - interMidPts[0];
                Vector3d shrinkMidVector1 = interMidPts[0] - interMidPts[1];
                shrinkMidVector0.Unitize();
                shrinkMidVector1.Unitize();

                Point3d realInterMidPt0 = interMidPts[0] + shrinkMidVector0;
                Point3d realInterMidPt1 = interMidPts[1] + shrinkMidVector1;

                double shaftMidLen = realInterMidPt0.DistanceTo(realInterMidPt1);
                Shaft shaftMid = new Shaft(realInterMidPt0, shaftMidLen, shaftRadius, axelDir);

                EntityList.Add(bullGear);
                EntityList.Add(shaftMid);

                #endregion

                pinionPos = bullGearPos;
            }

            isGroove = true;

            #endregion

            #region generate the rest of the gears and shaft except for the last shaft and the gear(s) on it

            for (int i = 1; i < shaftNum-1; i++)
            {
                // create the pinion gear

                numTeeth = 9;
                int bullGearTeethNum = (int)(2 * gearRadius / gearModule);

                if (isGroove)
                {
                    selfRotationAngle = 90;
                    if (bullGearTeethNum % 2 == 1)
                    {
                        isGroove = true;
                    }
                    else
                    {
                        isGroove = false;
                    }
                }
                else
                {
                    stepAngle = 360.0 / numTeeth;
                    selfRotationAngle = 90 - stepAngle / 2;

                    if (bullGearTeethNum % 2 == 1)
                    {
                        isGroove = false;
                    }
                    else
                    {
                        isGroove = true;
                    }
                }

                Point3d pinionGearPos = pinionPos + gearDirection * (clearance + gearRadius + 4.5);
                Gear pinionGear = new Gear(pinionGearPos, gearDevDir, gearDirection, numTeeth, gearModule, gearPressureAngle, gearFaceWidth + clearance, selfRotationAngle, true);
                Spacer sp_middle1 = new Spacer(pinionGearPos - gearDevDir * clearance, 1, shaftRadius, 3, -gearDevDir);

                // create the bull gear
                Point3d bullGearPos = pinionGearPos + gearDevDir * (clearance + gearFaceWidth);
                Gear bullGear = new Gear(bullGearPos, gearDevDir, gearDirection, bullGearTeethNum, gearModule, gearPressureAngle, gearFaceWidth, selfRotationAngle, true);
                Spacer sp_middle2 = new Spacer(bullGearPos + gearDevDir * (clearance + gearFaceWidth), 1, shaftRadius, 3, gearDevDir);

                // get two intersecting points on the modle to determine the shaft length and the starting point
                Curve shaftMidCrv = new Line(pinionGearPos - axelDir * 10000, pinionGearPos + axelDir * 10000).ToNurbsCurve();
                Point3d[] interMidPts;
                Intersection.CurveBrep(shaftMidCrv, _model, myDoc.ModelAbsoluteTolerance, out _, out interMidPts);

                Vector3d shrinkMidVector0 = interMidPts[1] - interMidPts[0];
                Vector3d shrinkMidVector1 = interMidPts[0] - interMidPts[1];
                shrinkMidVector0.Unitize();
                shrinkMidVector1.Unitize();

                Point3d realInterMidPt0 = interMidPts[0] + shrinkMidVector0;
                Point3d realInterMidPt1 = interMidPts[1] + shrinkMidVector1;

                double shaftMidLen = realInterMidPt0.DistanceTo(realInterMidPt1);
                Shaft shaftMid = new Shaft(realInterMidPt0, shaftMidLen, shaftRadius, axelDir);

                EntityList.Add(bullGear);
                EntityList.Add(pinionGear);
                EntityList.Add(sp_middle1);
                EntityList.Add(sp_middle2);
                EntityList.Add(shaftMid);

                pinionPos = bullGearPos;
            }

            #endregion

            #region generate the last shaft and the gear on it

            numTeeth = 9;

            if (isGroove)
            {
                selfRotationAngle = 90;
                if (numTeeth % 2 == 1)
                {
                    isGroove = true;
                }
                else
                {
                    isGroove = false;
                }
            }
            else
            {
                stepAngle = 360.0 / numTeeth;
                selfRotationAngle = 90 - stepAngle / 2;

                if (numTeeth % 2 == 1)
                {
                    isGroove = false;
                }
                else
                {
                    isGroove = true;
                }
            }

            Point3d lastGearPos = pinionPos + gearDirection * (clearance + gearRadius + 4.5);
            Curve shaftLastCrv = new Line(lastGearPos - axelDir * 10000, lastGearPos + axelDir * 10000).ToNurbsCurve();
            Point3d[] interLastPts;
            Intersection.CurveBrep(shaftLastCrv, _model, myDoc.ModelAbsoluteTolerance, out _, out interLastPts);

            Vector3d shrinkLastVector0 = interLastPts[1] - interLastPts[0];
            Vector3d shrinkLastVector1 = interLastPts[0] - interLastPts[1];
            shrinkLastVector0.Unitize();
            shrinkLastVector1.Unitize();

            Point3d realInterLastPt0 = interLastPts[0] + shrinkLastVector0;
            Point3d realInterLastPt1 = interLastPts[1] + shrinkLastVector1;

            double shaftLastLen = realInterLastPt0.DistanceTo(realInterLastPt1);
            Shaft shaftLast = new Shaft(realInterLastPt0, shaftLastLen, shaftRadius, axelDir);

            double shaftLenFull = shaftLastLen + 2;
            Point3d shaftMidPt = eeCen + dis * direction;
            Point3d shaftTargetPt = shaftMidPt - axelDir * (0.5 - finalGearPosRatio) * shaftLenFull;
            double lastGearFaceWidth = shaftTargetPt.DistanceTo(lastGearPos) + gearFaceWidth / 2;

            Gear lastPinionGear = new Gear(lastGearPos, gearDevDir, gearDirection, numTeeth, gearModule, gearPressureAngle, lastGearFaceWidth, selfRotationAngle, true);
            Spacer sp_last1 = new Spacer(lastGearPos - gearDevDir * clearance, 1, shaftRadius, 3, -gearDevDir);
            Spacer sp_last2 = new Spacer(lastGearPos + gearDevDir * (clearance + lastGearFaceWidth), 1, shaftRadius, 3, gearDevDir);

            EntityList.Add(shaftLast);
            EntityList.Add(lastPinionGear);
            EntityList.Add(sp_last1);
            EntityList.Add(sp_last2);

            #endregion

        }
        public List<Gear> GenerateGearTrain(Vector3d mainDirection, Vector3d axisDirection, Vector3d otherDirection, Point3d lastGearCenter, double length1,double length2,double length3,int gearRatioUserSelection)
        {
            List<Gear> generated_gears = new List<Gear>();
            #region First list all key parameters
            // The module, facewidth, clearance, shaftRadius and pressure angle are given as const values. Just use them
            int numTeethPinion = 9;
            double clearance = 0.3;//This is the distance between gear teeths.
            //Unitize all input vectors
            Vector3d dir1 = mainDirection / mainDirection.Length;
            Vector3d dir2 = axisDirection / axisDirection.Length;
            Vector3d dir3 = otherDirection / otherDirection.Length;
            #endregion

            #region Secondly, list all possible gear parameters,and sort them by gear ratio. Select one by user input
            List < GearTrainParam > possibleParams= new List<GearTrainParam>();
            // The minimum bull teeth would be 15, and maximum would be limited by total size.
            double maxDis = Math.Sqrt(length1 * length1 + length3 * length3);
            for (int t=15;t<maxDis/Math.PI/2/gearModule;t++)
            {
                double bullDaimeter = t * gearModule / Math.PI;
                for (int n=1;n<Math.Min(maxDis/bullDaimeter,5);n++)//TODO check if there would be a maximum count of gear set
                {
                    GearTrainParam param = new GearTrainParam(t, num_teeth_pinion, n, distance_between_gears);
                    if(param.IfValid(length1,length2,length3))
                    {
                        possibleParams.Add(param);
                    }
                }
            }
            possibleParams.Sort();
            //select one param by 1-10 given by user
            int selectedIndex = possibleParams.Count / 10 * gearRatioUserSelection;
            if (selectedIndex >= possibleParams.Count)
                selectedIndex -= 1;
            #endregion
            #region Lastly, use the selected param to generate gears. Just pick center pos one by one?
            bool success = false;
            GearTrainParam selectedParam;
            do
            {
                if (selectedIndex >= possibleParams.Count)
                    selectedIndex -= 1;
                selectedParam = possibleParams[selectedIndex];
                //calculate params
                double bullRadius = selectedParam.bullGearTeeth * gearModule / Math.PI / 2;
                double pinionRadius = selectedParam.pinionTeeth * gearModule / Math.PI / 2;
                double gearCenterDis = bullRadius + pinionRadius + ContinuousTranslation.clearance;
                Point3d firstGearCenter = lastGearCenter - dir1 * (length1 - bullRadius) - dir3 * (length3 - bullRadius);
                Plane gearPlane = new Plane(lastGearCenter, dir1, dir3);
                //loop from 1st to last.Everytime try to stretch distance.
                List<Point3d> gearCenterPoints = new List<Point3d>();
                gearCenterPoints.Add(firstGearCenter);
                bool dirIndicator = false;
                for(int i=1;i<selectedParam.gearSetNumber;i++)
                {
                    Circle centerCircleFromStart = new Circle(gearPlane, gearCenterPoints[i - 1], gearCenterDis);
                    Circle centerCircleFromLast= new Circle(gearPlane, lastGearCenter, gearCenterDis);
                }
                success = selectedParam.CheckCenterPointValidity(gearCenterPoints);
                if(!success)
                {
                    possibleParams.RemoveAt(selectedIndex);
                }
            } while (!success);
            //Generate gears with selected params
            #endregion
            return generated_gears;
        }
        private class GearTrainParam:IComparable<GearTrainParam>
        {
            public int bullGearTeeth;
            public int gearSetNumber;
            public int pinionTeeth;
            public double gearRatio;
            public double clearance;
            public GearTrainParam(int BullGearTeeth,int PinionTeeth,int GearSetNumber,double Clearance)
            {
                bullGearTeeth = BullGearTeeth;
                gearSetNumber = GearSetNumber;
                pinionTeeth = PinionTeeth;
                clearance = Clearance;
                gearRatio = Math.Pow(bullGearTeeth / pinionTeeth, gearSetNumber);
            }
            public bool IfValid(double l1,double l2,double l3)
            {
                //See if we could arrange this param into the space
                //Threshold 1 is thickness. If l2 is not enough for all gears, then not okay
                if ((l2 - gearFaceWidth) / 2 + gearFaceWidth / 2 < gearFaceWidth * gearSetNumber + clearance * (gearSetNumber - 1))
                    return false;
                //Threshold 2 is distance. All gears need to be able to fit in
                
                double bullRadius = bullGearTeeth * gearModule / Math.PI / 2;
                double pinionRadius = pinionTeeth * gearModule / Math.PI / 2;
                double maxDis = Math.Sqrt(Math.Pow(l1-bullRadius,2) +Math.Pow( l3-bullRadius,2));
                double dis =bullRadius+pinionRadius+clearance;
                if (dis * gearSetNumber < maxDis || dis * gearSetNumber > l1+l3-bullRadius*2)
                    return false;//gears wouldn't be able to fill the distance or couldn't fit in.
                //I think it could be filled by reasonably more gear sets since the trail could zigzag.So just set l1+l3 upper limit

                return false;
            }
            public int CompareTo(GearTrainParam other)
            {
                if (gearRatio > other.gearRatio)
                    return -1;
                else if (gearRatio < other.gearRatio)
                    return 1;
                else
                    return 0;
            }
        }
    }
}
