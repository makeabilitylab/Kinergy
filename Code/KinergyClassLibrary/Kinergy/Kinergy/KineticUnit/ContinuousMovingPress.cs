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
using Kinergy.Generator;
using Kinergy.Utilities;
using Kinergy.Relationship;

namespace Kinergy
{
    namespace KineticUnit
    {
        public class ContinuousMovingPress:KineticUnit
        {
            private Brep mainBody = null;
            private Brep axleBrep;
            
            private List<Brep> wheels;
            private double energy = 0;
            private double distance = 0;
            private Vector3d moveDirection = Vector3d.Unset;
            private bool addLock = false;
            private bool addRack = false;
            private bool addWheel = false;

            private Vector3d springDirection = Vector3d.Unset;
            private Point3d springPosition = Point3d.Unset;
            private Vector3d lockDirection = Vector3d.Unset;
            private Point3d lockPosition=Point3d.Unset;

            private double springLength = 0;
            private double springRadius = 0;
            private double wireRadius = 0;
            private int roundNum = 0;

            BoxLike mainBodyBox;
            private Transform rotate;
            private Transform rotateBack;
            private List<Brep> modelCut;
            private Helix springEntity;
            private RodLike axleEntity;
            private Rack rackEntity;
            private GearSet gearSet;
            private List<Brep> locks;
            private Generator.GearSet gears;
            RhinoDoc myDoc;

            public Brep MainBody { get => mainBody;private set => mainBody = value; }
            public Brep AxleBrep { get => axleBrep; private set => axleBrep = value; }
            public List<Brep> Wheels { get => wheels; private set => wheels = value; }
            public Vector3d MoveDirection { get => moveDirection; private set => moveDirection = value; }
            public Vector3d SpringDirection { get => springDirection; private set => springDirection = value; }
            public Point3d SpringPosition { get => springPosition; private set => springPosition = value; }
            public Point3d LockPosition { get => lockPosition; private set => lockPosition = value; }
            internal BoxLike MainBodyBox { get => mainBodyBox; private set => mainBodyBox = value; }
            public List<Brep> ModelCut { get => modelCut; private set => modelCut = value; }
            
            public List<Brep> Locks { get => locks; private set => locks = value; }
            public GearSet Gears { get => gears; private set => gears = value; }
            public Helix SpringEntity { get => springEntity;protected set => springEntity = value; }

            public ContinuousMovingPress(Brep Body,Brep Axle, double E, double D, Vector3d Move_Direction,bool AddLock,bool AddRack,bool AddWheel)
            {
                mainBody = Body;
                axleBrep = Axle;
                energy =E;
                distance =D;
                moveDirection = Move_Direction;
                modelCut = new List<Brep>();
                
                locks = new List<Brep>();
                rotate = Transform.Rotation(moveDirection, Vector3d.XAxis, Point3d.Origin);
                rotateBack = Transform.Rotation(Vector3d.XAxis, moveDirection, Point3d.Origin);
                mainBody.Transform(rotate);
                addLock = AddLock;
                addRack = AddRack;
                addWheel = AddWheel;
                myDoc = RhinoDoc.ActiveDoc;
            }
            public bool Process()
            {
                //TODO rewrite the getting directory part
                string path3 = System.IO.Directory.GetCurrentDirectory();
                RhinoApp.WriteLine(path3);
                //first receive base model and preprocess with main direction.
                //then let user choose where to add spring(since we already know the main direction)
                //then ask user select lock position
                //build lock and return model
                if (PreprocessModel() == false)
                { throw new Exception("Unable to process this model,please provide valid model and vector"); }
                

                if (SetSpringPosition() == false)
                { throw new Exception("Failed to set spring position."); }

                
                if (ConstructSpring() == false)
                { throw new Exception("Failed to build spring."); }

                if (addLock)
                {
                    
                    if (SetLockPosition() == false)
                    { throw new Exception("Failed to select lock position."); }

                    if (lockPosition == Point3d.Unset)
                    { return true; }

                    
                    if (ConstructLock() == false)
                    { throw new Exception("Failed to build lock structure."); }
                }
                if(addRack)
                {
                    if(ConstructRack()==false)
                    { throw new Exception("Failed to build rack structure."); }
                }
                if(addWheel)
                {
                    if(ConstructGearSet()==false)
                    { throw new Exception("Failed to build gearset structure to drive wheels."); }
                }
                foreach(Entity e in entityList)
                {
                    e.SetRotateBack(rotateBack);
                }
                return true;
            }
            /// <summary>
            /// This private method is for understanding the given model, make sure it fulfills requirements(it is a single box shape,)
            /// </summary>
            /// <returns></returns>
            private bool PreprocessModel()
            {
                //First check the validity of the model
                mainBodyBox=new Geom.BoxLike(mainBody);
                string body = string.Format("The main body box bbox is from {0},{1},{2} to {3},{4},{5}",
                    mainBodyBox.Bbox.Min.X, mainBodyBox.Bbox.Min.Y, mainBodyBox.Bbox.Min.Z,
                    mainBodyBox.Bbox.Max.X, mainBodyBox.Bbox.Max.Y, mainBodyBox.Bbox.Max.Z);
                RhinoApp.WriteLine(body);
                //if(mainBodyBox.Validity==false)
                //{ return false; }
                if (mainBodyBox.GetInnerEmptySpaceBox()==false)
                { return false; }
                body = string.Format("The main body box bbox is from {0},{1},{2} to {3},{4},{5}",
                    mainBodyBox.Bbox.Min.X, mainBodyBox.Bbox.Min.Y, mainBodyBox.Bbox.Min.Z,
                    mainBodyBox.Bbox.Max.X, mainBodyBox.Bbox.Max.Y, mainBodyBox.Bbox.Max.Z);
                RhinoApp.WriteLine(body);
                body = string.Format("The inner empty space box is from {0},{1},{2} to {3},{4},{5}",
                    mainBodyBox.InnerEmptySpaceBbox.Min.X, mainBodyBox.InnerEmptySpaceBbox.Min.Y, mainBodyBox.InnerEmptySpaceBbox.Min.Z,
                    mainBodyBox.InnerEmptySpaceBbox.Max.X, mainBodyBox.InnerEmptySpaceBbox.Max.Y, mainBodyBox.InnerEmptySpaceBbox.Max.Z);
                RhinoApp.WriteLine(body);
                entityList.Add(mainBodyBox);
                if(addWheel)
                {
                    axleEntity = new RodLike(axleBrep, false, "axle");
                    entityList.Add(axleEntity);
                }
                return true;
            }
            private bool CalculateSpringParameter()
            {
                double span = mainBodyBox.SpanAtDirection(springDirection);
                if(span==0)
                { return false; }
                //TODO adjust spring parameter setting here
                //Here I only scale the parameters using span, D and E
                if(span>50)
                { springLength = 25; }
                else { springLength = span / 2; }

                springRadius = springLength / 25 * 7.5;
                wireRadius = springLength * 0.5 / 25 * Math.Pow(energy / distance*2, 0.25);
                return true;
            }
            public bool SetSpringPosition()
            {
                //First calculate possible spring direction and let user select
                List<Vector3d> springDirectionCandidates=mainBodyBox.GetSurroundingDirections(moveDirection);
                List<Curve> Arrows = new List<Curve>();
                foreach(Vector3d v in springDirectionCandidates)
                {
                    Curve a = Arrow.ConstructArrow();
                    a.Transform(Transform.Rotation(Vector3d.XAxis, v, Point3d.Origin));
                    a.Transform(Transform.Translation(v * 20));
                    a.Transform(Transform.Translation(new Vector3d(mainBodyBox.Center)));
                    Arrows.Add(a);
                }
                springDirection = springDirectionCandidates[UserSelection.UserSelectCurveInRhino(Arrows,myDoc)];
                if(CalculateSpringParameter()==false)
                { return false; }

                //then use the bounding box offsetface to generate candidate points;
                List<Point3d> SpringPositionCandidates = mainBodyBox.GetPointCandidatesByDirection(springDirection,mainBodyBox.InnerEmptySpaceBbox, springRadius+4);
                if(SpringPositionCandidates.Count==0)
                { throw new Exception("Failed to provide spring position candidate: No space available"); }
                springPosition = SpringPositionCandidates[ UserSelection.UserSelectPointInRhino(SpringPositionCandidates, myDoc)];
                if(springPosition==Point3d.Unset)
                { return false; }
                return true;
            }
            
            public bool SetLockPosition()
            {
                //select from only two sides
                List<Vector3d> directions1 = mainBodyBox.GetSurroundingDirections(SpringDirection);
                List<Vector3d> directions2= mainBodyBox.GetSurroundingDirections(moveDirection);
                List<Vector3d> candidateDirections = new List<Vector3d>();
                foreach(Vector3d v in directions1)
                {
                    if(directions2.Contains(v))
                    { candidateDirections.Add(v);continue; }
                    
                }
                List<Curve> Arrows = new List<Curve>();
                foreach (Vector3d v in candidateDirections)
                {
                    Curve a = Arrow.ConstructArrow();
                    a.Transform(Transform.Rotation(Vector3d.XAxis, v, Point3d.Origin));
                    a.Transform(Transform.Translation(v * 10));
                    a.Transform(Transform.Translation(new Vector3d(mainBodyBox.Center)));
                    Arrows.Add(a);
                }
                lockDirection = candidateDirections[UserSelection.UserSelectCurveInRhino(Arrows, myDoc)];
                //and generate position candidate with springlen and D
                List<Point3d> positionCandidates=mainBodyBox.GetPointCandidatesByDirection(lockDirection, mainBodyBox.InnerEmptySpaceBbox, springRadius + 4);
                //delete points which are too near to spring end
                List<Point3d> candidates = new List<Point3d>();
                foreach(Point3d p in positionCandidates)
                {
                    if((new Vector3d(p)-new Vector3d(springEntity.EndPoint))*springEntity.Direction/springEntity.Direction.Length>-springLength*distance-5)
                    {
                        continue; 
                    }
                    candidates.Add(p);
                }
                lockPosition = candidates[UserSelection.UserSelectPointInRhino(candidates, myDoc)];
                if(lockPosition==Point3d.Unset)
                { return false; }
                return true;
            }
            
            
            private bool ConstructSpring()
            {
                Point3d springCenter = BoxLike.GetInnerPointBySurfacePoint(springPosition, mainBodyBox.InnerEmptySpaceBbox,0.8);

                Point3d startPoint = springCenter-springDirection*springLength/2;
                Point3d endPoint = springCenter + springDirection * springLength / 2;
                springEntity = new Helix(startPoint, endPoint, springRadius, wireRadius, roundNum);
                if (springEntity.Model == null)
                { 
                    return false;
                }
                entityList.Add(springEntity);
                Connector c = new Connector(springEntity, mainBodyBox);
                if(c.IsValid())
                {
                    entityList.Add(c.ConnectingStructure);
                    //Add knob,r=1.5,r=3.5
                    Vector3d direction = new Vector3d(endPoint) - new Vector3d(startPoint);
                    direction.Unitize();
                    double h1 = springLength * distance+mainBodyBox.SpanAtDirection(springDirection)/2-springLength/2;
                    Plane basePlane1 = new Plane(endPoint, direction);
                    Plane basePlane2 = new Plane(endPoint + h1 * direction, direction);
                    Circle c1 = new Circle(basePlane1, 1.5);
                    Circle c2 = new Circle(basePlane2, 3.5);
                    Cylinder rod1 = new Cylinder(c1, h1);
                    Cylinder rod2 = new Cylinder(c2, 2);
                    var knob = Brep.CreateBooleanUnion(new List<Brep> { rod1.ToBrep(true, true), rod2.ToBrep(true, true) }, myDoc.ModelAbsoluteTolerance);
                    Shape k = new Shape(knob[0]);
                    entityList.Add(k);
                    Fixation f = new Fixation(springEntity, k);
                    //Make hole
                    mainBodyBox.CreateRoundHoleOnSurface(springPosition, 2);
                }
                return true;
            }
            
            private bool ConstructLock()
            {
                //First build a panel connecting spring end and all other things
                Plane p = new Plane(springEntity.EndPoint, lockDirection,Vector3d.CrossProduct(lockDirection,springEntity.Direction));
                double xlen = (new Vector3d(mainBodyBox.InnerEmptySpaceBbox.ClosestPoint(lockPosition)) - new Vector3d(springEntity.EndPoint)) * lockDirection / lockDirection.Length;
                Interval x =new Interval(-springRadius,xlen-3);
                Interval y =new Interval(-springRadius-5,springRadius+5);
                Interval z =new Interval(0,2);
                Box panel = new Box(p, x, y, z);
                Shape panel1 = new Shape(panel.ToBrep(), false, "panel");
                entityList.Add(panel1);
                //Then read lock head and place it at the right place
                string dir = Environment.CurrentDirectory;
                System.IO.DirectoryInfo pathInfo = new System.IO.DirectoryInfo(dir);
                string newPath = pathInfo.Parent.FullName;
                //Then read the locker and place it
                Brep lockHeadBrep = FileOperation.SingleBrepFromResourceFileDirectory(newPath + "\\Plug - ins\\Grasshopper\\Components\\KinergyResources\\lockHeadContinuousMovingPress.3dm");
                Brep lockBaseBrep = FileOperation.SingleBrepFromResourceFileDirectory(newPath + "\\Plug - ins\\Grasshopper\\Components\\KinergyResources\\lockBaseContinuousMovingPress.3dm");
                
                //directory for test
                //Brep lockHeadBrep = FileOperation.SingleBrepFromResourceFile(FileOperation.FindCurrentFolderResourceDirectory() + "\\lockHeadContinuousMovingPress.3dm");
                //Brep lockBaseBrep = FileOperation.SingleBrepFromResourceFile(FileOperation.FindCurrentFolderResourceDirectory() + "\\lockBaseContinuousMovingPress.3dm");

                Transform move1 = Transform.Translation(new Vector3d(SpringEntity.EndPoint)+lockDirection/lockDirection.Length*(xlen-3));
                Transform rotate1 = Transform.Rotation(Vector3d.XAxis, -springEntity.Direction, Point3d.Origin);
                //another rotation here
                lockHeadBrep.Transform(rotate1);
                lockHeadBrep.Transform(move1);
                Shape lockHead = new Shape(lockHeadBrep, false, "lockHead");
                entityList.Add(lockHead);
                _ = new Fixation(lockHead, panel1);
                lockBaseBrep.Transform(rotate1);
                lockBaseBrep.Transform(move1);
                Transform move2 = Transform.Translation(SpringEntity.Direction / SpringEntity.Direction.Length * springLength * distance);
                lockBaseBrep.Transform(move2);
                Shape lockBase = new Shape(lockBaseBrep, false, "lockBase");
                entityList.Add(lockBase);
                _ = new Fixation(lockBase, mainBodyBox);
                //Then open a hole
                mainBodyBox.CreateRectangularHoleOnSurface(lockPosition, springDirection, moveDirection, springLength * distance + 3, 3);
                return true;
            }
            private bool ConstructRack()
            {
                //No need to select anything
                //Connect rack to panel or build one if there isn't any.
                Point3d startPoint = SpringEntity.EndPoint - moveDirection / moveDirection.Length * (springRadius + 4);
                Point3d endPoint = startPoint - springDirection / springDirection.Length * springLength * distance;//TODO
                Rack rack = new Rack(startPoint, endPoint, -moveDirection, 3, 1);
                entityList.Add(rack);
                Entity panel=null;
                foreach(Entity e in entityList)
                {
                    if(e.Name=="panel")
                    { 
                        panel = e;
                        break; 
                    }
                }
                _ = new Fixation(rack, panel);
                rackEntity = rack;
                return true;
            }

            private bool ConstructGearSet()
            {
                //Gear set is established between two movements. Here they are rotation of axle and translation of rack.
                Movement m1 = new Movement(rackEntity, 1, Transform.Translation(-springDirection / springDirection.Length * springLength * distance));
                Movement m2 = new Movement(axleEntity, 2, 1800);//Make axle rotate 5 times.
                gearSet = new GearSet(m1, m2, MainBodyBox.InnerEmptySpaceBbox);
                if(gearSet.Success==false)
                { return false; }
                List<Gear> gears = gearSet.GetGearList();
                foreach(Gear g in gears)
                { 
                    entityList.Add(g);
                    Connector c = new Connector(g, mainBodyBox);
                    entityList.Add(c.ConnectingStructure);
                }
                return true;
            }
            
        } 
    
    }
}
