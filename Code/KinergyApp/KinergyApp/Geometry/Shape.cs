﻿using System;
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

namespace Kinergy.Geom
{
    public class Shape:Entity
    {
        
        
        public Shape(Brep brep,bool stat=false,string n=""):base(brep,stat,n)
        {
            
        }
        protected override void ConductMoveAndUpdateParam(Movement move)
        {
            offset = Transform.Multiply(offset, move.Trans);
        }
        public override void SetModel(Brep m)
        {
            model = m;
        }
    }
}
