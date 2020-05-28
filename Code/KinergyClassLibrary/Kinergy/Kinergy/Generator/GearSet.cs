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
using Kinergy.Geom;

namespace Kinergy
{
    namespace Generator
    {
        public class GearSet
        {
            //This class automatically generate a serie of gears to accomplish a certain transmission
            private Movement movement1, movement2;
            private bool success = false;
            private BoundingBox confineBox;
            private List<Gear> gearList;
            

            public List<Gear> GearList { get => gearList;private set => gearList = value; }
            public bool Success { get => success;private set => success = value; }


            /// <summary>
            /// automatically generate a serie of gears to accomplish transmission between two movements
            /// </summary>
            /// <param name="move1"></param>
            /// <param name="move2"></param>
            public GearSet(Movement move1,Movement move2,BoundingBox bbox)
            {
                movement1 = move1;
                movement2 = move2;
                confineBox = bbox;
                gearList = new List<Gear>();
                GenerateGears();
                if(GearList.Count()>0)
                {
                    success = true;
                }
            }
            
            
            public bool GenerateGears()
            {
                //TODO using all given confinements, generate a list of gear 
                //First check the types
                if(movement1.Obj.GetType()==typeof(Rack) && movement2.Obj.GetType()==typeof(RodLike))
                {
                    Rack rack =(Rack) movement1.Obj;
                    RodLike rod = (RodLike)movement2.Obj;
                    List<Vector3d> directions = new List<Vector3d> 
                    { Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis, -Vector3d.XAxis, -Vector3d.YAxis, -Vector3d.ZAxis, };
                    Vector3d mainDirection1=Vector3d.Unset;
                    Vector3d mainDirection2 = Vector3d.Unset;
                    Vector3d mainDirection = new Vector3d(rod.Center) - new Vector3d(rack.Center);
                    foreach(Vector3d d in directions)
                    {
                        if(d*rack.FaceDirection>0.8)
                        { 
                            mainDirection1 = d;
                            break;
                        }
                    }
                    if(mainDirection1==Vector3d.Unset)
                    { return false; }
                    foreach(Vector3d d in directions)
                    {
                        if(d==mainDirection1 || d==-mainDirection1)
                        { continue; }
                        if(Math.Abs(d*rod.Direction)>0.8)
                        { continue; }
                        if(d*mainDirection>0)
                        { mainDirection2 = d; }
                    }
                    if (mainDirection2 == Vector3d.Unset)
                    { return false; }
                    double distance1 = mainDirection * mainDirection1;
                    double distance2 = mainDirection * mainDirection1;
                    //Here a simple formula is Z1/Z2*R/r=angle/360
                    int Z1=rack.Z;
                    double multiplier = movement2.MovementValue / 360;
                    int Z2 = (int)Math.Round(Math.Sqrt(1 / multiplier) * Z1);
                    Gear finalGear = new Gear(Z2);
                    Gear firstGear = new Gear(Z2);
                    double r = firstGear.RootRadius;
                    double R = multiplier * r * Z2 / Z1;
                    Gear secondGear = new Gear(R);
                    //Then place these gears in place. If the space is not enough, return false
                    //If there's gap in between, fill it with extra gear.
                    finalGear.SetPosition(rod.Center + rod.Direction / rod.Direction.Length * 10,rod.Direction);
                    Vector3d v = distance1 * mainDirection1 + distance2 * mainDirection2;
                    v.Unitize();
                    secondGear.SetPosition(finalGear.CenterPoint + v * (finalGear.RootRadius + secondGear.RootRadius + finalGear.ToothDepth),rod.Direction);
                    firstGear.SetPosition
                        (secondGear.CenterPoint - rod.Direction / rod.Direction.Length * (10 + mainDirection * rod.Direction / rod.Direction.Length),
                         rod.Direction);
                    if((new Vector3d(firstGear.CenterPoint)-new Vector3d(rack.CenterPoint))*mainDirection1<0)
                    { return false; }
                    if ((new Vector3d(firstGear.CenterPoint) - new Vector3d(rack.CenterPoint)).Length<firstGear.RootRadius)
                    { return false; }
                    double extraRadius = firstGear.CenterPoint.DistanceTo(rack.Model.ClosestPoint(firstGear.CenterPoint));
                    Point3d extraCenter = firstGear.CenterPoint-mainDirection1*extraRadius;
                    Gear extraGear = new Gear(extraCenter, rod.Direction, extraRadius);
                    gearList.Add(extraGear);
                    gearList.Add(firstGear);
                    gearList.Add(secondGear);
                    gearList.Add(finalGear);
                    return true;
                }
                return false;
            }
            
            private static bool Contain(Box box,Object obj)
            {
                //TODO tell if the box contain the model
                throw new Exception("This method hasn't been implemented.");
                //return false;
            }
            public List<Gear> GetGearList()
            {
                return gearList;
            }

        }
    }
}

