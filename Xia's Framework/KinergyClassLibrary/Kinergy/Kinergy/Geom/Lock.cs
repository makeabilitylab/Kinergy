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
using Kinergy.Constraints;

namespace Kinergy.Geom
{
    class Lock:Entity
    {
        bool headOrBase;
        /// <summary>
        /// Lock entity. Both lock head and lock base should be 
        /// </summary>
        /// <param name="brep"></param>
        /// <param name="LockHeadOrLockBase">True for head and false for base</param>
        /// <param name="stat"></param>
        /// <param name="n"></param>
        public Lock(Brep brep,bool LockHeadOrLockBase, bool stat= false, string n = "") : base(brep, stat, n)
        {
            //TODO add lock features to this class
            headOrBase = LockHeadOrLockBase;
        }

        public bool HeadOrBase { get => headOrBase;private set => headOrBase = value; }

        protected override void ConductMoveAndUpdateParam(Movement move)
        {
            base.model.Transform(move.Trans);
            UpdateBasicParams();
        }
    }
}
