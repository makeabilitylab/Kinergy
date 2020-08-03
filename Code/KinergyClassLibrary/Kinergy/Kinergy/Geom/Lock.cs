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
using Kinergy.Relationship;

namespace Kinergy.Geom
{
    public class Lock:Entity
    {
        private bool headOrBase;
        private Lock otherPart=null;
        private bool locked = false;
        private Point3d releasePosition = Point3d.Unset;
        /// <summary>
        /// Lock entity.This constructor is for lock head.
        /// </summary>
        /// <param name="brep"></param>
        /// <param name="LockHeadOrLockBase">True for head and false for base</param>
        /// <param name="stat">Whether the entity is static</param>
        /// <param name="n"></param>
        public Lock(Brep brep,bool LockHeadOrLockBase, bool stat= false, string n = "") : base(brep, false, n)
        {
            headOrBase = LockHeadOrLockBase;
        }
        public Lock(Brep brep, bool LockHeadOrLockBase, Point3d Release, bool stat = false, string n = "") : base(brep, false, n)
        {
            
            headOrBase = LockHeadOrLockBase;
            releasePosition = Release;
        }
        public Lock(Point3d CenterPoint,Vector3d direction,double radius, bool stat = false, string n = "") : base(null, false, n)
        {
            model = BuildRatchetLock(CenterPoint, direction, radius);
            headOrBase = true;
        }
        public bool HeadOrBase { get => headOrBase;private set => headOrBase = value; }
        public Lock OtherPart { get => otherPart;private set => otherPart = value; }
        public bool Locked { get => locked;private set => locked = value; }
        public Point3d ReleasePosition { get => releasePosition;private set => releasePosition = value; }
        public static Brep BuildRatchetLock(Point3d CenterPoint, Vector3d direction, double radius)
        {
            List<Point3d> outerPts = new List<Point3d>();
            List<Point3d> innerPts = new List<Point3d>();
            Plane p = new Plane(CenterPoint, direction);
            PolyCurve s = new PolyCurve();
            for (int i=0;i<8;i++)
            {
                double angle = Math.PI / 4 * i;
                Vector3d x = p.XAxis, y = p.YAxis;
                outerPts.Add(CenterPoint + x * Math.Sin(angle) * radius * 1.1 + y * Math.Cos(angle) * radius * 1.1);
                innerPts.Add(CenterPoint + x * Math.Sin(angle) * radius * 0.9 + y * Math.Cos(angle) * radius * 0.9);
            }
            for(int i=0;i<8;i++)
            {
                s.Append(new Line(outerPts[i], innerPts[i]).ToNurbsCurve());
                s.Append(new Line(innerPts[i], outerPts[(i+1)%8]).ToNurbsCurve());
            }
            Point3d start = CenterPoint - direction * radius * 0.05;
            Point3d end = CenterPoint + direction * radius * 0.05;
            Curve c = s.ToNurbsCurve();
            Curve c1 = s.ToNurbsCurve();
            c.Transform(Transform.Translation((new Vector3d(start) - new Vector3d(end)) / 2));
            c1.Transform(Transform.Translation((new Vector3d(end) - new Vector3d(start))/2));
            Curve rail = new Line(start, end).ToNurbsCurve();
            Brep b1 = Brep.CreateFromSweep(rail, c, true, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance)[0];
            Brep b2 = Brep.CreatePatch(new List<Curve> { c }, null, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            Brep b3 = Brep.CreatePatch(new List<Curve> { c1 }, null, RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
            //Brep m = Brep.CreateSolid(new List<Brep> { b1, b2, b3 }, 0.001)[0];
            Brep m = new Brep();
            m.Append(b1);
            m.Append(b2);
            m.Append(b3);
            return m;
        }
        protected override void ConductMoveAndUpdateParam(Movement move)
        {
            /*base.model.Transform(move.Trans);
            UpdateBasicParams();*/
            base.ConductMoveAndUpdateParam(move);
        }
        public bool RegisterOtherPart(Lock other)
        {
            if(other.OtherPart==null || other.OtherPart==this)
            { 
                if(this.headOrBase==other.HeadOrBase)
                {
                    return false;
                }
                otherPart = other;
                if(other.OtherPart == null)
                {other.RegisterOtherPart(this); }
                return true;
            }
            return false;
        }
        public override bool AddConstraint(Relationship.Relationship constraint)
        {
            if(constraint.GetType()==typeof(Locking))
            {
                //Check type
                if(constraint.TheOtherEntity(this).GetType()!=typeof(Lock))
                {
                    return false;
                }
                Lock other = (Lock)constraint.TheOtherEntity(this);
                //check head or base
                if (this.headOrBase==other.HeadOrBase)
                {
                    return false;
                }
                if (constraint.IsValid() && constraint.Include(this))
                {
                    constraints.Add(constraint);
                    return true;
                }
                else { return false; }
            }
            else 
            {
                return base.AddConstraint(constraint);
            }
        }
        public bool SetLocked()
        {
            if(otherPart==null)
            {
                return false;
            }
            if(locked==true)
            {
                return false;
            }
            if (locked == false)
                locked = true;
            Locking l = new Locking(this, otherPart);
            //AddConstraint(l);
            return true;
        }
        public bool Activate()
        {
            //First check if this is lock base. only base could generate pointing position
            if(otherPart==null)
            {
                return false;
            }
            if(headOrBase==true)
            {
                return otherPart.Activate();
            }
            /*if(releasePosition==Point3d.Unset)
            {
                return false;
            }*/
            //generate Point at releasePosition and let user select it
            if(Utilities.UserSelection.UserSelectPointInRhino(new List<Point3d> { releasePosition }, RhinoDoc.ActiveDoc)==0)
            {
                //Release();
                Unlock();
            }
            return true;
        }
        public bool ActivateWithoutInteraction()
        {
            //First check if this is lock base. only base could generate pointing position
            if (otherPart == null)
            {
                return false;
            }
            /*if (headOrBase == true)
            {
                return otherPart.ActivateWithoutInteraction();
            }*/
            /*
            if (releasePosition == Point3d.Unset)
            {
                return false;
            }*/
            //generate Point at releasePosition and let user select it
            //Release();
            Unlock();
            return true;
        }
        /// <summary>
        /// This method deletes the locking relationship between locks. Don't use it unless necessary
        /// </summary>
        private void Release()
        {
            int count = constraints.Count();
            string body = string.Format("Release method called");
            Rhino.RhinoApp.WriteLine(body);
            for (int i= 0; i < count; i++)
            {
                if(constraints[i].GetType()==typeof(Locking))
                {
                    Locking l = (Locking)constraints[i];
                    l.Release();
                    i -= 1;
                    count -= 1;
                }
            }
        }
        /// <summary>
        /// This method make sure that both part of the lock is not set locked
        /// </summary>
        private void Unlock()
        {
            locked = false;
            otherPart.Locked = false;
            Rhino.RhinoApp.WriteLine("Unlock executed!");
        }
        public override bool Move(Movement move)
        {
            if (this.Equals(move.Obj) == false)
            {
                throw new Exception("Wrong movement parameter given to entity!");
            }
            if (staticEntity)
            {
                return false;
            }
            DfsMark = true;
            bool CanIMove = true;
            //Then move all other constraints to know if this movement can be operated
            foreach (Relationship.Relationship c in constraints)
            {
                
                if (c.TheOtherEntity(this).DfsMark == true)//Skip the already visited component to avoid cycle triggering.
                { continue; }
                if (c.Move(move) == false)
                {
                    CanIMove = false;
                    string body = string.Format("A movement on {0} typed {1} with value {2} is stopped by {3} to {4}", this.GetType(), move.Type, move.MovementValue, c.GetType(), c.TheOtherEntity(this).GetType());
                    Rhino.RhinoApp.WriteLine(body);
                    break;
                }
            }
            if (CanIMove)
            {
                this.ConductMoveAndUpdateParam(move);
            }
            DfsMark = false;
            return CanIMove;
        }
    }
}
