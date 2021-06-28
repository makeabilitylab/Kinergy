using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Kinergy.Geom;
namespace Kinergy
{
    namespace Relationship
    { 
        public class Relationship
        {
            //A constraint is a relationship between 2 objects. This class is then inherited by several to name the actual relationship.
            protected Entity obj1;
            protected Entity obj2;

            public Entity Obj1 { get => obj1; protected set => obj1 = value; }
            public Entity Obj2 { get => obj2; protected set => obj2 = value; }

            public virtual bool IsValid()
            {
                //TODO set up conditions in every kind of constraint, to ensure that no invalid relationship is set
                return true;
            }
            
            public Relationship(Entity object1,Entity object2)
            {
                obj1 = object1;
                obj2 = object2;
                obj1.AddConstraint(this);
                obj2.AddConstraint(this);
            }
            public virtual bool Move(Movement move)
            {
                //This method would be overridden in all of the actual relationship classes.
                //The calculation of movement would be done in these classes 
                return true;
            }
            public Entity TheOtherEntity(Entity obj)
            {
                if(obj.Equals(obj1))
                { return obj2; }
                else if(obj.Equals(obj2))
                { return obj1; }
                else { throw new Exception("This constraint dosen't include the given Entity!"); }
            }
            public bool Include(Entity obj)
            {
                if(obj1.Equals(obj) || obj2.Equals(obj))
                {
                    return true;
                }
                return false;
            }
            public void Release()
            {
                this.obj1.Constraints.Remove(this);
                this.obj2.Constraints.Remove(this);
            }
        }
    }
}
