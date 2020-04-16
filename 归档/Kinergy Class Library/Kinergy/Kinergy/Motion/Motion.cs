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

namespace Kinergy.Motion
{
    public class Motion
    {
        protected List<Entity> entityList;
        public List<Entity> EntityList { get => entityList;protected set => entityList = value; }
        public Motion()
        {
            entityList = new List<Entity>();
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
        

        
        
    }
}
