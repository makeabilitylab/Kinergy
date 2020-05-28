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
using Grasshopper.Kernel;
namespace Kinergy.Motion
{
    public class Motion
    {
        protected List<Entity> entityList;
        private bool loaded;
        public List<Entity> EntityList { get => entityList;protected set => entityList = value; }
        public bool Loaded { get => loaded;protected set => loaded = value; }

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
                models.Add(e.GetModelinWorldCoordinate());
            }
            return models;
        }
        public List<Mesh> GetMeshModel()
        {
            List<Mesh> models = new List<Mesh>();
            foreach (Entity e in entityList)
            {
                Mesh[] ms = Mesh.CreateFromBrep(e.GetModelinWorldCoordinate(), MeshingParameters.FastRenderMesh);
                foreach(Mesh m in ms)
                {
                    if(m.Faces.Count>0)
                    {models.Add(m); }
                }
            }
            return models;
        }

        public virtual bool LoadMotion()
        {
            //to be overritten
            loaded = true;
            return false;
        }
        public virtual bool Trigger()
        {
            //to be overritten
            return false;
        }
        public virtual bool TriggerWithoutInteraction()
        {
            //to be overritten
            return false;
        }
        public virtual Movement Simulate(double interval = 20, double precision=0.01)
        {
            Movement m=null;
            return m;
        }
        public virtual void ResetMotion()
        {
            //To be overridden. Reset all travel and offset to zero
            foreach(Entity e in entityList)
            {
                e.ResetState();
            }
            loaded = false;
        }
    }
}
