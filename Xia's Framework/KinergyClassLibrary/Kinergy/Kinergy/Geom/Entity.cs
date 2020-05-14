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
using Kinergy.Constraints;

namespace Kinergy.Geom
{
    /// <summary>
    /// This is the base class of all solid models in Kinergy, including both mechanical components and static models.
    /// </summary>
    public class Entity
    {
        protected Brep model;
        protected bool staticEntity = false;
        protected List<Constraint> constraints;
        protected BoundingBox bbox;
        protected Point3d center;
        private bool dfsMark;
        private string name;
        protected Transform rotateBack = Transform.Identity;
        protected Transform offset = Transform.Unset;
        public Brep Model 
        { 
            get 
            { return model; } 
            protected set 
            {
                model = value;
                UpdateBasicParams(); 
            } 
        }
        public Point3d Center { get => center; protected set => center = value; }
        public BoundingBox Bbox { get => bbox; protected set => bbox = value; }
        
        public bool StaticEntity { get => staticEntity; protected set => staticEntity = value; }
        public bool DfsMark { get => dfsMark;protected set => dfsMark = value; }
        public string Name { get => name;}
        public  Transform RotateBack { get => rotateBack;protected set => rotateBack = value; }
        public List<Constraint> Constraints { get => constraints;protected set => constraints = value; }
        public Transform Offset { get => offset;protected set => offset = value; }

        public Entity(bool isStatic = false,string n="")
        { 
            staticEntity = isStatic;
            constraints = new List<Constraint>();
            dfsMark = false;
            name = n;
            offset = Transform.Identity;
        }
        
        public Entity(Brep m,bool isStatic=false, string n= "")
        {
            model = m;
            staticEntity = isStatic;
            constraints = new List<Constraint>();
            dfsMark = false;
            name = n;

            offset = Transform.Identity;
        }
        public virtual bool AddConstraint(Constraint constraint)
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
            if(this.Equals(move.Obj)==false)
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
            foreach(Constraint c in constraints)
            {
                
                if(c.TheOtherEntity(this).DfsMark==true)//Skip the already visited component to avoid cycle triggering.
                { continue; }
                if (c.Move(move) == false)
                { 
                    CanIMove = false;
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
            
            offset = Transform.Multiply(offset, move.Trans);
            
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
            Brep m = Model.DuplicateBrep();
            //Brep m = model;
            if(rotateBack!=Transform.Unset)
            { m.Transform(rotateBack);}
            if (Offset != Transform.ZeroTransformation)
            { m.Transform(Offset); }
            
            return m;
        }
        public virtual void ResetState()
        {
            offset = Transform.Identity;
        }
    }
}
