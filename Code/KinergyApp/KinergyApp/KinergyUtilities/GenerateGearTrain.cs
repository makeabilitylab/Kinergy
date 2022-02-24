using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
namespace KinergyUtilities
{
    class GenerateGearTrain
    {
        const double GearModule = 1;
        const int PinionMinTeeth = 9;
        const int BullMinTeeth = 15;
        public static List<GearTrainScheme> GetGearTrainSchemes(Vector3d mainDirection, Vector3d axisDirection, Point3d lastGearCenter,Box InnerCavity, double GearFaceWidth)
        {
            List<GearTrainScheme> schemes = new List<GearTrainScheme>();
            for(int i=1;i<=10;i++)//try all cases with less than 10 gear sets. more than 10 will be not so feasible with printed material.
            {
                GearTrainScheme newScheme = new GearTrainScheme(i, mainDirection, axisDirection, lastGearCenter, InnerCavity, GearFaceWidth);
                if(newScheme.valid)
                {
                    if (newScheme.parameters.Count > 0)
                        schemes.Add(newScheme);
                }
            }
            return schemes;
        }
        /// <summary>
        /// This function returns a list of gear parameters according to given params
        /// </summary>
        /// <param name="axisDirection">The norm direction of gears, also the direction of gear axises. Could be not among World XYZ!</param>
        /// <param name="firstGearCenter">The center position of first gear. Note that this position will move along axis direction but we would expand the first gear facewidth so it could be precisely connected to latch or driving gear at the middle of box</param>
        /// <param name="lastGearCenter">The center position of last gear. Note that this position will not move along axis direction thus be precise</param>
        /// <param name="InnerCavity">The available space for gear train, as a box. Be sure that this box has 3 dimensions same as world XYZ</param>
        /// <param name="GearFaceWidth"></param>
        /// <returns></returns>
        public static List<GearTrainParam> GetGearTrainByParameter(int gearSetNumber,Vector3d mainDirection,Vector3d otherDirection, Vector3d axisDirection, Point3d firstGearCenter, Point3d lastGearCenter,Box InnerCavity ,Box InnerCavityEnlarged,double GearFaceWidth)
        {
            List<GearTrainParam> paramList=new List<GearTrainParam>();
            axisDirection.Unitize();
            double distance = (firstGearCenter - (axisDirection * (new Vector3d(firstGearCenter) * axisDirection))).DistanceTo
                (lastGearCenter - (axisDirection * (new Vector3d(lastGearCenter) * axisDirection)));
            int gearsetCount = gearSetNumber;
            double unitLength = distance / gearsetCount;
            //For each gearset count, list all possible R and rs, add them to the possible list
            int pinionTeeth = 9;
            while (unitLength - 0.3 - pinionTeeth * GearModule / 2 > BullMinTeeth * GearModule / 2 && unitLength - 0.3 - pinionTeeth * GearModule / 2 > pinionTeeth * GearModule / 2 + 1)
            {
                double pinionRadius = pinionTeeth * GearModule / 2;
                double bullGearRadius =Math.Floor(unitLength - 0.3 - pinionRadius);
                GearTrainParam param = new GearTrainParam(mainDirection, otherDirection, axisDirection, firstGearCenter, lastGearCenter, InnerCavity,InnerCavityEnlarged, bullGearRadius, pinionRadius, gearSetNumber, GearFaceWidth);
                if (param.IfValid())
                {
                    paramList.Add(param);
                }
                pinionTeeth++;
            }
            //Don't select Index, just return everything back
            return paramList;
        }
    }

    public class GearTrainScheme
    {
        public int gearSetNumber;
        Point3d lastGearCenter;
        Vector3d mainDirection;
        Vector3d axisDirection;
        Box innerCavity;
        double gearFaceWidth;
        double clearance = 0.3;
        double gearModulus = 1;
        int minPinionTeeth = 9;
        int minBullTeeth = 15;

        public bool valid = true;
        double boxLength;
        double distanceAvailable;
        Point3d firstGearCenter;
        double maxBullGearRadius;
        public List<GearTrainParam> parameters;

        public GearTrainScheme(int _gearSetNumber, Vector3d _mainDirection, Vector3d _axisDirection, Point3d _lastGearCenter, Box _InnerCavity, double _gearFaceWidth)
        {
            gearSetNumber = _gearSetNumber;
            mainDirection = _mainDirection;
            axisDirection = _axisDirection;
            lastGearCenter = _lastGearCenter;
            innerCavity = _InnerCavity;
            gearFaceWidth = _gearFaceWidth;
            parameters = new List<GearTrainParam>();

            Vector3d otherDirection = new Plane(lastGearCenter, mainDirection, axisDirection).Normal;
            otherDirection.Unitize();
            int mainDirectionIndex = 0;

            #region First calculate max bull gear radius
            //First figure out the distance within inner cavity box that's available for gear train
            Vector3d boxXaxis = innerCavity.Plane.XAxis;
            Vector3d boxYaxis = innerCavity.Plane.YAxis;
            Vector3d boxZaxis = innerCavity.Plane.Normal;
            double middle_part_thickness = gearSetNumber * (gearFaceWidth + clearance);
            Box innerCavityEnlarged = new Box(innerCavity.BoundingBox);
            
            if (Math.Abs(mainDirection * boxXaxis) > 0.9)
            {
                mainDirectionIndex = 1;
                boxLength = innerCavity.X.Length;
                if (mainDirection * boxXaxis > 0)
                {
                    distanceAvailable = boxLength - innerCavity.X.Max + lastGearCenter.X;
                }
                else
                {
                    distanceAvailable = boxLength + innerCavity.X.Min - lastGearCenter.X;
                }
                innerCavityEnlarged.Inflate(1,0,0);
            }
            else if (Math.Abs(mainDirection * boxYaxis) > 0.9)
            {
                mainDirectionIndex = 2;
                boxLength = innerCavity.Y.Length;
                if (mainDirection * boxYaxis > 0)
                {
                    distanceAvailable = boxLength - innerCavity.Y.Max + lastGearCenter.Y;
                }
                else
                {
                    distanceAvailable = boxLength + innerCavity.Y.Min - lastGearCenter.Y;
                }
                innerCavityEnlarged.Inflate(0, 1, 0);
            }
            else
            {
                mainDirectionIndex = 3;
                boxLength = innerCavity.Z.Length;
                if (mainDirection * boxZaxis > 0)
                {
                    distanceAvailable = boxLength - innerCavity.Z.Max + lastGearCenter.Z;
                }
                else
                {
                    distanceAvailable = boxLength + innerCavity.Z.Min - lastGearCenter.Z;
                }
                innerCavityEnlarged.Inflate(0, 0, 1);
            }
            //Threshold 1 is not exceeding box edges
            Line l1 = new Line(lastGearCenter, axisDirection, 1);//Try to draw a line to intersect with inner cavity box
            Interval interval1, interval2;
            Point3d midpoint;
            
            if (Intersection.LineBox(l1, innerCavityEnlarged, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out interval1))
            {
                midpoint = l1.PointAt(interval1.Mid);
                Line l2 = new Line(midpoint, otherDirection, 1);//Try to draw a line to intersect with inner cavity box
                Intersection.LineBox(l1, innerCavityEnlarged, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out interval2);
                Point3d p1 = l2.PointAt(interval2.Max);
                Point3d p2 = l2.PointAt(interval2.Min);
                maxBullGearRadius = Math.Min(p1.DistanceTo(midpoint), p2.DistanceTo(midpoint));
            }
            else
            { 
                valid = false;
                return;
            }
            //Threshold 2 is along the main direction: (n)r + (n+1)R <= Dis
            if (maxBullGearRadius> (distanceAvailable - (gearSetNumber * minPinionTeeth * gearModulus / 2)) / (gearSetNumber + 1))
                maxBullGearRadius = (distanceAvailable - (gearSetNumber * minPinionTeeth * gearModulus / 2)) / (gearSetNumber + 1);
            #endregion
            //Then figure out the first gear position
            if(mainDirectionIndex==1)//main direation is x axis
            {
                if (mainDirection * boxXaxis > 0)
                    firstGearCenter = new Point3d(innerCavity.X.Min + maxBullGearRadius + clearance, innerCavity.Y.Mid, innerCavity.Z.Mid);
                else
                    firstGearCenter = new Point3d(innerCavity.X.Max - maxBullGearRadius + clearance, innerCavity.Y.Mid, innerCavity.Z.Mid);
            }
            else if(mainDirectionIndex == 2)
            {
                if (mainDirection * boxYaxis > 0)
                    firstGearCenter = new Point3d(innerCavity.X.Mid, innerCavity.Y.Min + maxBullGearRadius + clearance, innerCavity.Z.Mid);
                else
                    firstGearCenter = new Point3d(innerCavity.X.Mid, innerCavity.Y.Max - maxBullGearRadius + clearance, innerCavity.Z.Mid);
            }
            else
            {
                if (mainDirection * boxZaxis > 0)
                    firstGearCenter = new Point3d(innerCavity.X.Mid, innerCavity.Y.Mid, innerCavity.Z.Min + maxBullGearRadius + clearance);
                else
                    firstGearCenter = new Point3d(innerCavity.X.Mid, innerCavity.Y.Mid, innerCavity.Z.Max - maxBullGearRadius + clearance);
            }
            //Then use all these params and existing function to generate all needed params
            parameters=GenerateGearTrain.GetGearTrainByParameter(gearSetNumber,mainDirection,otherDirection,axisDirection, firstGearCenter, lastGearCenter, innerCavity, innerCavityEnlarged,gearFaceWidth);
        }
    }
    public class GearTrainParam : IComparable<GearTrainParam>
    {
        public double bullGearRadius;
        public int gearSetNumber;
        public double pinionRadius;
        public double gearRatio;
        public double clearance;
        public double gearFaceWidth;
        Vector3d boxXaxis, boxYaxis, boxZaxis;
        Vector3d mainDirection, otherDirection, axisDirection;
        Vector3d offset_direction_first_to_last;
        Point3d fgct,fgct_offset,lgct;
        Box innerCavity;
        public List<GearParameter> parameters;
        public GearTrainParam(Vector3d _mainDirection, Vector3d _otherDirection, Vector3d _axisDirection, Point3d firstGearCenter, Point3d lastGearCenter, Box InnerCavity, Box InnerCavityEnlarged,double BullGearRadius, double PinionRadius, int GearSetNumber, double GearFaceWidth, double Clearance = 0.3, double ClearanceDepth = 0.3)
        {
            mainDirection = _mainDirection;
            mainDirection.Unitize();
            otherDirection = _otherDirection;
            axisDirection = _axisDirection;
            bullGearRadius = BullGearRadius;
            gearSetNumber = GearSetNumber;
            pinionRadius = PinionRadius;
            clearance = Clearance;
            gearFaceWidth = GearFaceWidth;
            gearRatio = Math.Pow(bullGearRadius / pinionRadius, gearSetNumber);
            innerCavity = InnerCavity;
            fgct = firstGearCenter;
            lgct = lastGearCenter;
            //calculate the position of first and last gear, and the box thickness
            //First consider the distance between fgct and lgct along main axis. It should be exactly n(R+r+c)
            Vector3d first2last = new Vector3d(lgct) - new Vector3d(fgct);
            double disAlongMain = first2last * mainDirection;
            fgct += mainDirection * (disAlongMain - gearSetNumber * (bullGearRadius + pinionRadius + clearance));
            boxXaxis = InnerCavity.Plane.XAxis;
            boxYaxis = InnerCavity.Plane.YAxis;
            boxZaxis = InnerCavity.Plane.Normal;
            Line l1 = new Line(lastGearCenter, axisDirection, 1);//Try to draw a line to intersect with inner cavity box
            Interval interval1;
            //Calculate the midpoint of axis
            Point3d midpoint;
            Intersection.LineBox(l1,InnerCavityEnlarged ,RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, out interval1);
            midpoint = l1.PointAt(interval1.Mid);
            if ((new Vector3d(lastGearCenter)-new Vector3d(midpoint)) * axisDirection >= 0)//if the offset from first to last is along axis direction
                offset_direction_first_to_last = axisDirection;
            else
                offset_direction_first_to_last = -axisDirection;
            offset_direction_first_to_last.Unitize();
            double middle_part_thickness = gearSetNumber * (gearFaceWidth + clearance);
            //Move fgct to make sure the gear train is at the middle of box.If fgct_offset is too close to fgct(so it might come across bull gear beside it) then move it away a bit
            fgct_offset = fgct - offset_direction_first_to_last * Math.Max(middle_part_thickness/2,2*gearFaceWidth+clearance);
            fgct_offset -= offset_direction_first_to_last * (gearFaceWidth / 2);//Since gear generation is on one side, not in middle
            parameters = GetParameterList();
        }
        public bool IfValid()//See if we could arrange this param into the box
        {
            //just try to get the bounding box of all gears and see if they could fit into inner cavity. Exclude the last pinion
            for(int i=0;i<parameters.Count-1;i++)
            {
                GearParameter param = parameters[i];
                Box box = new Box(new Plane(param.center, mainDirection, otherDirection), new Interval(-param.radius, param.radius), new Interval(-param.radius, param.radius),
                    new Interval(-param.faceWidth/2, param.faceWidth / 2));
                BoundingBox bbox=box.BoundingBox;
                if (!innerCavity.Contains(bbox))
                    return false;
            }
            return true;
        }
        public List<GearParameter> GetParameterList()
        {
            List<GearParameter> parameters = new List<GearParameter>();
            //TODO Add all gear params one by one
            Vector3d start2end = new Vector3d(lgct) - new Vector3d(fgct_offset);//This excludes axis direction
            start2end -= offset_direction_first_to_last * start2end * offset_direction_first_to_last;
            start2end.Unitize();
            GearParameter fgp_pinion = new GearParameter();
            //make first gear thicker to reach middle point
            fgp_pinion.center = fgct;
            fgp_pinion.radius = pinionRadius;
            fgp_pinion.faceWidth = gearFaceWidth;
            fgp_pinion.norm = offset_direction_first_to_last;
            fgp_pinion.xDirection = start2end;
            fgp_pinion.PinionOrBull = 1;
            parameters.Add(fgp_pinion);
            GearParameter fgp_bull = new GearParameter();
            //make first gear thicker to reach middle point
            fgp_bull.center = fgct_offset;
            fgp_bull.radius = bullGearRadius;
            fgp_bull.faceWidth = gearFaceWidth;
            fgp_bull.norm = offset_direction_first_to_last;
            fgp_bull.xDirection = start2end;
            fgp_bull.PinionOrBull = 2;
            parameters.Add(fgp_bull);
            Point3d currPt = fgct_offset;

            for (int i = 1; i < gearSetNumber; i++)
            {
                currPt += start2end * (bullGearRadius + pinionRadius + clearance);
                //currPt += offset_direction_first_to_last * clearance;
                GearParameter small = new GearParameter();
                small.center = currPt;
                small.radius = pinionRadius;
                small.faceWidth = gearFaceWidth+clearance;
                small.norm = offset_direction_first_to_last;
                small.xDirection = start2end;
                small.PinionOrBull = 1;
                parameters.Add(small);
                GearParameter big = new GearParameter();
                currPt += offset_direction_first_to_last * (gearFaceWidth);
                currPt += offset_direction_first_to_last * clearance;
                big.center = currPt;
                big.radius = bullGearRadius;
                big.faceWidth = gearFaceWidth;
                big.norm = offset_direction_first_to_last;
                big.xDirection = start2end;
                big.PinionOrBull = 2;
                parameters.Add(big);


            }
            currPt += start2end * (bullGearRadius + pinionRadius + clearance);
            //currPt += lgct_offset_direction * clearance;
            GearParameter lgp = new GearParameter();
            lgp.center = currPt;
            lgp.radius = pinionRadius;
            lgp.faceWidth = Math.Max(gearFaceWidth,currPt.DistanceTo(lgct)+gearFaceWidth/2);
            lgp.norm = offset_direction_first_to_last;
            lgp.xDirection = start2end;
            lgp.PinionOrBull = 1;
            parameters.Add(lgp);

            return parameters;
        }
        public int CompareTo(GearTrainParam other)
        {
            if (gearRatio < other.gearRatio)
                return -1;
            else if (gearRatio > other.gearRatio)
                return 1;
            else
                return 0;
        }
    }
    public struct GearParameter
    {
        public Point3d center;
        public Vector3d norm;
        public Vector3d xDirection;
        public double radius;
        public double faceWidth;
        public int PinionOrBull; // 1: pinion; 2: bull
    }
}
