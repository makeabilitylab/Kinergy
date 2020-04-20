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
namespace Kinergy.Motion
{
    public class Motion
    {
        protected List<Entity> entityList;
        protected bool loaded;
        private bool simulating = false;
        public List<Entity> EntityList { get => entityList;protected set => entityList = value; }
        public bool Simulating { get => simulating;protected set => simulating = value; }

        public Motion()
        {
            entityList = new List<Entity>();
            loaded = false;
        }
        public List<Brep> GetModel()
        {
            List<Brep> models = new List<Brep>();
            foreach(Entity e in entityList)
            {
                Brep b = e.Model.DuplicateBrep();
                if(e.RotateBack!=Transform.Unset)
                { b.Transform(e.RotateBack); }
                models.Add(b);
            }
            return models;
        }
        public void SetLoad(bool load)
        {
            if(simulating==false)
            { 
                if(load)
                {
                    loaded = true;
                    this.LoadMotion();
                
                }
            }
        }
        public void UnLoad()
        {
            //Check if all locking relationship is gone
            bool lockingClear = true;
            foreach(Entity e in entityList)
            {
                if(e.GetType()==typeof(Lock))
                {
                    foreach(Constraint c in e.Constraints)
                    {
                        if(c.GetType()==typeof(Locking))
                        { 
                            lockingClear = false;
                            break; 
                        }
                    }
                }
            }
            //if no locking exist, reset the loaded indicator to false
            if(lockingClear)
            {
                loaded = false;
            }
        }
        public virtual void LoadMotion()
        {
            //to be overritten
        }
        
    }
}
