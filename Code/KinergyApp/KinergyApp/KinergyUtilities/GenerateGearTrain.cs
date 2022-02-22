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
        /// <summary>
        /// This function returns a list of gear parameters according to given params
        /// </summary>
        /// <param name="axisDirection">The norm direction of gears, also the direction of gear axises</param>
        /// <param name="firstGearCenter">The center position of first gear. Note that this position will not move along axis direction since we would require it to be precisely connected to latch or driving gear</param>
        /// <param name="lastGearCenter">The center position of last gear. Note that this position might move along axis direction</param>
        /// <param name="InnerCavity">The available space for gear train, as a box. Be sure that this box has 3 dimensions same as world XYZ</param>
        /// <param name="GearFaceWidth"></param>
        /// <param name="gearRatioUserSelection">A number from 1-10. Please correct the index calculation if the actual selection is not 1-10</param>
        /// <returns></returns>
        public static List<GearParameter> GetGearTrainByParameter(Vector3d axisDirection, Point3d firstGearCenter, Point3d lastGearCenter,Box InnerCavity ,double GearFaceWidth, int gearRatioUserSelection)
        {
            int PinionMinTeeth = 9;
            int BullMinTeeth = 15;
            List<GearTrainParam> paramList=new List<GearTrainParam>();
            axisDirection.Unitize();
            double distance = (firstGearCenter - (axisDirection * (new Vector3d(firstGearCenter) * axisDirection))).DistanceTo
                (lastGearCenter - (axisDirection * (new Vector3d(lastGearCenter) * axisDirection)));
            int gearsetCount = 1;
            while(distance/(gearsetCount+1)>0.3+(PinionMinTeeth+BullMinTeeth) * GearModule/2)
            {
                //R+r+0.3=distance/(gearsetCount+1)
                double unitLength = distance / (gearsetCount + 1);
                //For each gearset count, list all possible R and rs, add them to the possible list
                int pinionTeeth = 9;
                while (unitLength - 0.3 - pinionTeeth * GearModule / 2  > BullMinTeeth * GearModule / 2 && unitLength - 0.3 - pinionTeeth * GearModule / 2 > pinionTeeth * GearModule / 2)
                {
                    double pinionRadius = pinionTeeth * GearModule / 2 ;
                    double bullGearRadius = unitLength - 0.3 - pinionRadius;
                    GearTrainParam param = new GearTrainParam(axisDirection, firstGearCenter, lastGearCenter, InnerCavity,bullGearRadius, pinionRadius,gearsetCount,GearFaceWidth);
                    if (param.IfValid())
                        paramList.Add(param);
                    pinionTeeth++;
                }
                gearsetCount++;
            }
            //SelectIndex
            int index =(int)Math.Floor(paramList.Count / 9.0 * (gearRatioUserSelection-1));
            return paramList[index].GetParameterList();
        }
        private class GearTrainParam : IComparable<GearTrainParam>
        {
            public double bullGearRadius;
            public int gearSetNumber;
            public double pinionRadius;
            public double gearRatio;
            public double clearance;
            public double gearFaceWidth;
            double boxThickness;
            Vector3d boxXaxis, boxYaxis, boxZaxis;
            Vector3d x, y;
            Point3d lgct_offset;
            Vector3d lgct_offset_direction;
            Point3d fgct;
            Box innerCavity;
            public GearTrainParam(Vector3d axisDirection, Point3d firstGearCenter, Point3d lastGearCenter, Box InnerCavity,double BullGearRadius, double PinionRadius, int GearSetNumber, double GearFaceWidth, double Clearance=0.3,double ClearanceDepth=0.3)
            {
                bullGearRadius = BullGearRadius;
                gearSetNumber = GearSetNumber;
                pinionRadius = PinionRadius;
                clearance = Clearance;
                gearFaceWidth = GearFaceWidth;
                gearRatio = Math.Pow(bullGearRadius / pinionRadius, gearSetNumber+1);
                innerCavity = InnerCavity;
                //calculate the position of first and last gear, and the box thickness
                fgct = firstGearCenter;
                Point3d lg_fuzhu = lastGearCenter - (axisDirection * (new Vector3d(firstGearCenter) * axisDirection));
                boxXaxis = InnerCavity.Plane.XAxis;
                boxYaxis = InnerCavity.Plane.YAxis;
                boxZaxis = InnerCavity.Plane.Normal;
                double middle_part_thickness = gearSetNumber * (gearFaceWidth + clearance);
                if (Math.Abs(axisDirection * boxXaxis) > 0.9)
                {
                    boxThickness = InnerCavity.X.Length;
                    x = boxYaxis;
                    y = boxZaxis;

                    if (Math.Abs(firstGearCenter.X + middle_part_thickness - InnerCavity.X.Mid) > Math.Abs(firstGearCenter.X - middle_part_thickness - InnerCavity.X.Mid))
                    {
                        lgct_offset = new Point3d(firstGearCenter.X - middle_part_thickness, lg_fuzhu.Y, lg_fuzhu.Z);
                        lgct_offset_direction = -boxXaxis;
                    }
                    else
                    {
                        lgct_offset = new Point3d(firstGearCenter.X + middle_part_thickness, lg_fuzhu.Y, lg_fuzhu.Z);
                        lgct_offset_direction = boxXaxis;
                    }

                }
                else if (Math.Abs(axisDirection * boxYaxis) > 0.9)
                {
                    boxThickness = InnerCavity.Y.Length;
                    x = boxXaxis;
                    y = boxZaxis;
                    if (Math.Abs(firstGearCenter.Y + middle_part_thickness - InnerCavity.Y.Mid) > Math.Abs(firstGearCenter.Y - middle_part_thickness - InnerCavity.Y.Mid))
                    {
                        lgct_offset = new Point3d(lg_fuzhu.X, firstGearCenter.Y - middle_part_thickness, lg_fuzhu.Z);
                        lgct_offset_direction = -boxYaxis;
                    }
                    else
                    {
                        lgct_offset = new Point3d(lg_fuzhu.X, firstGearCenter.Y + middle_part_thickness, lg_fuzhu.Z);
                        lgct_offset_direction = boxYaxis;
                    }
                }
                else
                {
                    boxThickness = InnerCavity.Z.Length;
                    x = boxXaxis;
                    y = boxYaxis;
                    if (Math.Abs(firstGearCenter.Z + middle_part_thickness - InnerCavity.Z.Mid) > Math.Abs(firstGearCenter.Z - middle_part_thickness - InnerCavity.Z.Mid))
                    {
                        lgct_offset = new Point3d(lg_fuzhu.X, lg_fuzhu.Y, firstGearCenter.Z - middle_part_thickness);
                        lgct_offset_direction = -boxZaxis;
                    }
                    else
                    {
                        lgct_offset = new Point3d(lg_fuzhu.X, lg_fuzhu.Y, firstGearCenter.Z + middle_part_thickness);
                        lgct_offset_direction = boxZaxis;
                    }
                }
                lgct_offset_direction.Unitize();

            }
            public bool IfValid()
            {
                //See if we could arrange this param into the box
                //Threshold 1 is thickness. If l2 is not enough for all gears, then not okay
                if ((gearSetNumber+1)*gearFaceWidth+gearSetNumber*clearance>boxThickness)
                    return false;
                //Threshold 2 is first and last gear staying inside box. which can be simplified as the outer 4 points of both gears stay within box.
                List<Point3d> ptsToVerify = new List<Point3d>();
                ptsToVerify.Add(fgct + x * bullGearRadius);
                ptsToVerify.Add(fgct - x * bullGearRadius);
                ptsToVerify.Add(fgct + y * bullGearRadius);
                ptsToVerify.Add(fgct - y * bullGearRadius);
                ptsToVerify.Add(lgct_offset + x * pinionRadius);
                ptsToVerify.Add(lgct_offset - x * pinionRadius);
                ptsToVerify.Add(lgct_offset + y * pinionRadius);
                ptsToVerify.Add(lgct_offset - y * pinionRadius);
                foreach (Point3d pt in ptsToVerify)
                    if (!innerCavity.Contains(pt))
                        return false;
                return true;
            }
            public List<GearParameter> GetParameterList()
            {
                List<GearParameter> parameters = new List<GearParameter>();
                //TODO Add all gear params one by one
                Vector3d start2end = new Vector3d(lgct_offset) - new Vector3d(fgct);
                start2end -= lgct_offset_direction * start2end * lgct_offset_direction;
                start2end.Unitize();
                GearParameter fgp = new GearParameter();
                fgp.center = fgct;
                fgp.radius = bullGearRadius;
                fgp.faceWidth = gearFaceWidth;
                fgp.norm = lgct_offset_direction;
                fgp.xDirection = start2end;
                parameters.Add(fgp);
                Point3d currPt = fgct;
                
                for (int i=0;i<gearSetNumber;i++)
                {
                    currPt += start2end * (bullGearRadius+pinionRadius+clearance);
                    currPt+= lgct_offset_direction * clearance;
                    GearParameter small = new GearParameter();
                    small.center = currPt;
                    small.radius = pinionRadius;
                    small.faceWidth = gearFaceWidth;
                    small.norm = lgct_offset_direction;
                    small.xDirection = start2end;
                    parameters.Add(small);
                    GearParameter big = new GearParameter();
                    currPt += lgct_offset_direction * (gearFaceWidth);
                    big.center = currPt;
                    big.radius = bullGearRadius;
                    big.faceWidth = gearFaceWidth;
                    big.norm = lgct_offset_direction;
                    big.xDirection = start2end;
                    parameters.Add(big);
                    
                    
                }
                currPt += start2end * (bullGearRadius+pinionRadius + clearance);
                //currPt += lgct_offset_direction * clearance;
                GearParameter lgp = new GearParameter();
                lgp.center = currPt;
                lgp.radius = pinionRadius;
                lgp.faceWidth = gearFaceWidth;
                lgp.norm = lgct_offset_direction;
                lgp.xDirection = start2end;
                parameters.Add(lgp);

                return parameters;
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
    public struct GearParameter
    {
        public Point3d center;
        public Vector3d norm;
        public Vector3d xDirection;
        public double radius;
        public double faceWidth;
    }
}
