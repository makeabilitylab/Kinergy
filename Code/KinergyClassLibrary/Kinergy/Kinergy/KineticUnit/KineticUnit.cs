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
using Kinergy.Relationship;
using Grasshopper.Kernel;
namespace Kinergy.KineticUnit
{
    public class KineticUnit
    {
        protected List<Entity> entityList;
        private bool loaded;
        public List<Entity> EntityList { get => entityList;protected set => entityList = value; }
        public bool Loaded { get => loaded;protected set => loaded = value; }

        public KineticUnit()
        {
            entityList = new List<Entity>();
            loaded = false;
        }
        public virtual List<Brep> GetModel()
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
        public BoundingBox GetKineticUnitBoundingBox()
        {
            double minX = 1000000000, minY = 1000000000, minZ = 1000000000, maxX = -1000000000, maxY = -1000000000, maxZ = -1000000000;
            if(entityList.Count==0)
            {
                return BoundingBox.Empty;
            }
            foreach(Entity e in entityList)
            {
                BoundingBox b=e.Model.GetBoundingBox(true);
                if (b.Min.X < minX) minX = b.Min.X;
                if (b.Min.Y < minY) minY = b.Min.Y;
                if (b.Min.Z < minZ) minZ = b.Min.Z;
                if (b.Max.X > maxX) maxX = b.Max.X;
                if (b.Max.Y > maxY) maxY = b.Max.Y;
                if (b.Max.Z > maxZ) maxZ = b.Max.Z;
            }
            return new BoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
        }
        public virtual bool LoadKineticUnit()
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
        public virtual void ResetKineticUnit()
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
