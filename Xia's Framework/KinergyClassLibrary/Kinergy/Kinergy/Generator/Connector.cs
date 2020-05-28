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
using Kinergy.Geom;
using Kinergy.Constraints;

namespace Kinergy.Generator
{
    /// <summary>
    /// This class help generate connecting structure between floating component and outer shell
    /// </summary>
    public class Connector
    {
        private Entity connectingStructure=null;
        private Entity object1;
        private Entity object2;
        public Entity ConnectingStructure { get => connectingStructure; set => connectingStructure = value; }
        /// <summary>
        /// Constructor of connector. This class generate connecting structure. Fixation is added here
        /// </summary>
        /// <param name="obj1">Entity to be connected</param>
        /// <param name="obj2">Entity for obj1 to be connected to</param>
        public Connector(Entity obj1,Entity obj2)
        {
            object1 = obj1;
            object2 = obj2;
            Generate();
        }
        private void Generate()
        {
            //First do a type check
            
            if(object1.GetType()==typeof(Spring))
            {
                Spring s = (Spring)object1;
                GenerateConnectorForSpring(s);
            }
            else if(object1.GetType() == typeof(Gear))
            {
                Gear g = (Gear)object1;
                GenerateConnectorForGear(g);
            }
            else if(object1.GetType() == typeof(Spiral))
            {
                Spiral s = (Spiral)object1;
                GenerateConnectorForSpiral(s);
            }
            else if(object1.GetType() == typeof(Rack))
            {
                Rack r = (Rack)object1;
                GenerateConnectorForRack(r);
            }
            else if(object1.GetType() == typeof(Lock))
            {
                Lock l = (Lock)object1;
                GenerateConnectorForLock(l);
            }
            else
            {
                throw new Exception("Invalid parameter type passed to connector."); 
            }
        }
        private void GenerateConnectorForSpring(Spring s)
        {
            //Just generate a cylinder(as a Shape) to connect the start of spring to base structure
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            double r = s.SpringRadius * 1.1;
            Vector3d springVector = new Vector3d(s.Start) - new Vector3d(s.End);

            var pts = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(new List<Brep> { object2.Model.GetBoundingBox(true).ToBrep() }, // brep on which to project
                   new List<Point3d> { s.Start }, 
                   springVector, 
                   myDoc.ModelAbsoluteTolerance);
            if (pts.Count() > 0)
            {
                double height = new Line(s.Start, pts[0]).Length;
                Circle baseCircle = new Circle(new Plane(s.Start, springVector), r);
                Cylinder c = new Cylinder(baseCircle, height);
                connectingStructure = new Shape(c.ToBrep(true, true));
            }
            //If the cylinder is generated, set up fixation relationship between them
            if(connectingStructure!=null)
            {
                _ = new Fixation(s, connectingStructure);
                _ = new Fixation( connectingStructure,object2);

            }
        }
        private void GenerateConnectorForGear(Gear g)
        {
            //Just generate a rodlike to connect the center of the gear to base structure
            RhinoDoc myDoc = RhinoDoc.ActiveDoc;
            double r = g.RootRadius*0.2;
            r = Math.Sqrt(r / 3) * 3;//In order to prevent r being too small or too big
            Vector3d gearVector =g.Direction;

            var pts1 = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(new List<Brep> { object2.Model.GetBoundingBox(true).ToBrep() }, // brep on which to project
                   new List<Point3d> { g.CenterPoint },
                   gearVector, 
                   myDoc.ModelAbsoluteTolerance);

            var pts2 = Rhino.Geometry.Intersect.Intersection.ProjectPointsToBreps(new List<Brep> { object2.Model.GetBoundingBox(true).ToBrep() }, // brep on which to project
                   new List<Point3d> { g.CenterPoint },
                   -gearVector, 
                   myDoc.ModelAbsoluteTolerance);
            Line l = new Line(pts1[0], pts2[0]);
            
            double height = l.Length;
            Circle baseCircle = new Circle(new Plane(pts1[0], -g.Direction), r);
            Cylinder c = new Cylinder(baseCircle,height);
            connectingStructure = new Shape(c.ToBrep(true, true));
            
            //If the rodlike is generated, set up fixation relationship between the rodlike and gear, and hinge relationship between rodlike and base structure.
            if (connectingStructure != null)
            {
                Point3d[] cp = new Point3d[1];
                cp[0] = g.CenterPoint;
                _ = new Hinge(g, connectingStructure,cp,g.Direction);
                _ = new Fixation(connectingStructure, object2);

            }
        }
        private void GenerateConnectorForSpiral(Spiral s)
        {
            //TODO I don't know how to connect a spiral to other structure
        }
        private void GenerateConnectorForRack(Rack r)
        {
            //Generate box shape to connect end of rack or back of rack to base structure
        }
        private void GenerateConnectorForLock(Lock l)
        {
            //Generate box shape to connect lock to base structure
        }
        public bool IsValid()
        {
            if(connectingStructure != null)
            { return true; }
            return false;
        }
    }
}
