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
        public Transform Translation = Transform.Identity;
        public Vector3d direction = Vector3d.XAxis;
        public List<Entity> EntityList { get => entityList; protected set => entityList = value; }
        public bool Loaded { get => loaded; protected set => loaded = value; }

        public KineticUnit()
        {
            entityList = new List<Entity>();
            loaded = false;
        }
        public void Translate(double dis)
        {
            Translation = Transform.Multiply(Translation, Transform.Translation(direction / direction.Length * dis));
        }
        public virtual List<Brep> GetModel()
        {
            List<Brep> models = new List<Brep>();
            foreach (Entity e in entityList)
            {
                if (e.Model != null)
                {
                    Brep m = e.GetModelinWorldCoordinate();
                    m.Transform(Translation);
                    models.Add(m);
                }
            }
            return models;
        }
        public List<Mesh> GetMeshModel()
        {
            List<Mesh> models = new List<Mesh>();
            foreach (Entity e in entityList)
            {
                //if (e.Model != null && e.GetType() != typeof(Gear))
                if (e.Model != null)
                    {
                    Brep m = e.GetModelinWorldCoordinate();
                    m.Transform(Translation);
                    if (m.IsValid)
                    {
                        Mesh[] ms = Mesh.CreateFromBrep(m, MeshingParameters.Coarse);
                        foreach (Mesh me in ms)
                        {
                            if (me.Faces.Count > 0)
                            { models.Add(me); }
                        }
                    }
                    //else
                    //{
                    //    int i = 1;
                    //}
                }

            }
            return models;
        }
        public BoundingBox GetKineticUnitBoundingBox()
        {
            double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;
            if (entityList.Count == 0)
            {
                return BoundingBox.Empty;
            }
            foreach (Entity e in entityList)
            {
                if (e.Model != null)
                {
                    BoundingBox b = e.Model.GetBoundingBox(true);
                    if (b.Min.X < minX) minX = b.Min.X;
                    if (b.Min.Y < minY) minY = b.Min.Y;
                    if (b.Min.Z < minZ) minZ = b.Min.Z;
                    if (b.Max.X > maxX) maxX = b.Max.X;
                    if (b.Max.Y > maxY) maxY = b.Max.Y;
                    if (b.Max.Z > maxZ) maxZ = b.Max.Z;
                }

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
        public virtual Movement Simulate(double interval = 20, double precision = 0.01)
        {
            Movement m = null;
            return m;
        }
        public virtual void ResetKineticUnit()
        {
            //To be overridden. Reset all travel and offset to zero
            foreach (Entity e in entityList)
            {
                e.ResetState();
            }
            loaded = false;
        }
        public bool RemoveEntity(Entity e)
        {
            if (entityList.Contains(e))
            {
                entityList.Remove(e);
                int Count = e.Constraints.Count;
                for (int i = 0; i < Count; i++)
                {
                    e.Constraints[i].Release();
                    Count--;
                    i--;
                }
                return true;
            }
            else
                return false;
        }
    }
}
