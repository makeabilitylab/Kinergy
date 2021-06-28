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
using Kinergy.Relationship;

namespace Kinergy.Geom
{
    /// <summary>
    /// This is the base class of all solid models in Kinergy, including both mechanical components and static models.
    /// </summary>
    public class Entity
    {
        protected Brep model;
        protected bool staticEntity = false;
        protected List<Relationship.Relationship> constraints;
        protected BoundingBox bbox;
        protected Point3d center;
        private bool dfsMark;
        private string name;
        protected Transform rotateBack = Transform.Identity;
        protected Transform offset = Transform.Unset;

        // Added by Liang He on 6/17/2021
        protected Vector3d axis;

        public Brep Model 
        { 
            get 
            { return model; } 
            set 
            {
                model = value;
                //UpdateBasicParams(); 
            } 
        }
        public Vector3d Axis { get => axis; protected set => axis = value; }
        public Point3d Center { get => center; protected set => center = value; }
        public BoundingBox Bbox { get => bbox; protected set => bbox = value; }
        
        public bool StaticEntity { get => staticEntity; protected set => staticEntity = value; }
        public bool DfsMark { get => dfsMark;protected set => dfsMark = value; }
        public string Name { get => name;}
        public  Transform RotateBack { get => rotateBack;protected set => rotateBack = value; }
        public List<Relationship.Relationship> Constraints { get => constraints;protected set => constraints = value; }
        public Transform Offset { get => offset;protected set => offset = value; }

        public Entity(bool isStatic = false,string n="")
        { 
            staticEntity = isStatic;
            constraints = new List<Relationship.Relationship>();
            dfsMark = false;
            name = n;
            offset = Transform.Identity;
            axis = new Vector3d(0, 0, 0);
        }
        
        public Entity(Brep m,bool isStatic=false, string n= "")
        {
            model = m;
            staticEntity = isStatic;
            constraints = new List<Relationship.Relationship>();
            dfsMark = false;
            name = n;

            offset = Transform.Identity;
            axis = new Vector3d(0, 0, 0);
        }
        public virtual bool AddConstraint(Relationship.Relationship constraint)
        {
            if(constraint.IsValid() && constraint.Include(this))
            {
                constraints.Add(constraint);
                return true;
            }
            return false;
        }
        /// <summary>
        /// Method to apply movement on certain entity. Whether the movement is conductable is recursively checked. 
        /// </summary>
        /// <param name="move"></param>
        /// <returns></returns>
        public virtual bool Move(Movement move)
        {
            if(!this.Equals(move.Obj))
            {
                throw new Exception("Wrong movement parameter given to entity!");
            }
            if(staticEntity)
            { 
                return false; 
            }
            dfsMark = true;
            bool CanIMove = true;
            //Then move all other constraints to know if this movement can be operated
            foreach(Relationship.Relationship c in constraints)
            {
                
                if(c.TheOtherEntity(this).DfsMark==true)//Skip the already visited component to avoid cycle triggering.
                { continue; }
                if (c.Move(move) == false)
                { 
                    CanIMove = false;
                    string body = string.Format("A movement on {0} typed {1} with value {2} is stopped by {3} to {4}", this.GetType(), move.Type, move.MovementValue, c.GetType(), c.TheOtherEntity(this).GetType());
                    Rhino.RhinoApp.WriteLine(body);
                    break;
                }
            }
            if(CanIMove)
            {
                this.ConductMoveAndUpdateParam(move);
            }
            dfsMark = false;
            return CanIMove;
        }
        protected virtual void ConductMoveAndUpdateParam(Movement move)
        {
            //This method would be overridden by different kinds of entity.Here constraints dosen't matter, just move the model by changing offset and rotate!
            bool valid = true ;
            if (!offset.IsValid)
            { 
                Rhino.RhinoApp.WriteLine("Invalid trans is passed");
                valid = false;
            }
            Transform tr =new Transform(offset);
            offset = Transform.Multiply(tr, move.Trans);
            if(!offset.IsValid && valid)
            {
                Rhino.RhinoApp.WriteLine("Invalid trans is caused");
                throw new Exception("Invalid trans is caused");
            }
            
        }
        public void UpdateBasicParams()
        {
            if (model != null)
            {
                bbox = model.GetBoundingBox(true);
                center = bbox.Center;
            }
            //And more parameters if needed
        }
        public virtual int GetContactPosition(Entity obj)
        {
            //Should be override by spring and spiral;
            return 0;
        }
        public virtual bool CreateRoundHoleOnSurface(Point3d p,double r)
        {
            return false;
        }
        public virtual bool CreateRectangularHoleOnSurface(Point3d center,Vector3d xDirection,Vector3d yDirection, double x,double y)
        {
            return false;
        }
        public void SetRotateBack(Transform rb)
        {
            rotateBack = rb;
        }
        public Brep GetModelinWorldCoordinate()
        {
            Brep m = null;
            if(Model != null)
            {
                m = Model.DuplicateBrep();
                //Brep m = model;
                if (rotateBack != Transform.Unset)
                { m.Transform(rotateBack); }
                //if (Offset != Transform.ZeroTransformation)
                //{
                m.Transform(offset);
                // }
            }
            return m;
        }
        public virtual void ResetState()
        {
            offset = Transform.Identity;
        }
        public virtual void SetModel(Brep m)
        {
            model = m;
        }
    }
}
