using System;
using System.Collections.Generic;
using System.IO;
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
using Kinergy.Utilities;
using Kinergy.Geom;
using Kinergy.Relationship;
using Kinergy.KineticUnit;
using Kinergy;
using System.Diagnostics;

namespace Kinergy.KineticUnit
{
    class ContinuousRotation : KineticUnit
    {
        //The initial inputs
        private Brep model;
        private double energy; // the range on the interface is 0.1-1, the ratio of the max energy
        private double speed; // the range on the interface is 0-9, indicating the speed level
        private Vector3d direction = Vector3d.Unset;
        private bool addLock;
        RhinoDoc myDoc;
        private int inputType;


        private Curve skeleton = null;
        private List<Shape> modelCut;
        private List<Lock> locks;
        private Helix spring;

        public ContinuousRotation(Brep Model, bool Curved, Vector3d Direction, double Energy, double Speed, int InputType)
        {
            model = Model;
            energy = Energy;
            speed = Speed;
            direction = Direction;
            modelCut = new List<Shape>();
            myDoc = RhinoDoc.ActiveDoc;
            locks = new List<Lock>();
            inputType = InputType;
        }

        public Helix Spring { get => spring; set => spring = value; }
        public List<Lock> Locks { get => locks; set => locks = value; }
        public List<Shape> ModelCut { get => modelCut; set => modelCut = value; }
        public Curve Skeleton { get => skeleton; set => skeleton = value; }
        public Brep Model { get => model; set => model = value; }
        public double Energy { get => energy; set => energy = value; }
        public double Speed { get => speed; set => speed = value; }
    }
}
