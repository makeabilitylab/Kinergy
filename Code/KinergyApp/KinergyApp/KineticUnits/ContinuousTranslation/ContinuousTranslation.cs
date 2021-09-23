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
                    double n_ceiling = Math.Min((unitLen - 4.5 - R) / (R + 4.8) + 1, (axelSpace - initialOffset + 0.3) / 3.9);
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
                        double gearRatio = Math.Pow(R / 4.5, n);

                        int idx = 0;
                        foreach(var ele in _shaft_radius_pool)
                        {
                            double shaftNum = ele.ElementAt(0);
                            double gearR = ele.ElementAt(1);
                            double testGearRatio = Math.Pow(gearR / 4.5, shaftNum);

                            if(gearRatio > testGearRatio)
                            {
                                // keep looking 
                                idx++;
                            }
                            else if (gearRatio == testGearRatio)
                            {
                                // replace the current item
                                int currIdx = _shaft_radius_pool.IndexOf(ele);
                                _shaft_radius_pool.RemoveAt(currIdx);
                                List<double> temp = new List<double>();
                                temp.Add(n);
                                temp.Add(R);
                                _shaft_radius_pool.Insert(currIdx, temp);
                            }
                            else
                            {
                                // insert the item
                                List<double> temp = new List<double>();
                                temp.Add(n);
                                temp.Add(R);
                                int currIdx = _shaft_radius_pool.IndexOf(ele);
                                _shaft_radius_pool.Insert(currIdx, temp);
                            }
                        }

                    }
                }
               
            }

            


        }

        public void GenerateSpringMotor()
        {

        }

        public void GenerateGearTrain(double finalGearPosRatio)
        {

        }

    }
}
