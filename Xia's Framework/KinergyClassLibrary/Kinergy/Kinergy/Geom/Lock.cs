﻿using System;
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
using Kinergy.Constraints;

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
            //TODO add lock features to this class
            if(LockHeadOrLockBase==false)
                { throw new Exception("Should provide release point for lock base"); }
            headOrBase = LockHeadOrLockBase;
        }
        public Lock(Brep brep, bool LockHeadOrLockBase, Point3d Release, bool stat = false, string n = "") : base(brep, false, n)
        {
            //This is base,so LockHeadOrBase should be false
            if (LockHeadOrLockBase == true)
            { throw new Exception("Should not provide release point for lock head"); }
            headOrBase = LockHeadOrLockBase;
            releasePosition = Release;
        }
        public bool HeadOrBase { get => headOrBase;private set => headOrBase = value; }
        public Lock OtherPart { get => otherPart;private set => otherPart = value; }
        public bool Locked { get => locked;private set => locked = value; }
        public Point3d ReleasePosition { get => releasePosition;private set => releasePosition = value; }

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
        public override bool AddConstraint(Constraint constraint)
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
            if(releasePosition==Point3d.Unset)
            {
                return false;
            }
            //generate Point at releasePosition and let user select it
            if(Utilities.UserSelection.UserSelectPointInRhino(new List<Point3d> { releasePosition }, RhinoDoc.ActiveDoc)==0)
            {
                Release();
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
            if (headOrBase == true)
            {
                return otherPart.ActivateWithoutInteraction();
            }
            if (releasePosition == Point3d.Unset)
            {
                return false;
            }
            //generate Point at releasePosition and let user select it
            Release();
            return true;
        }
        private void Release()
        {
            int count = constraints.Count();
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
    }
}
