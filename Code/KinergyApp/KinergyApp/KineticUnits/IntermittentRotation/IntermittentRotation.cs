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
    class IntermittentRotation : KineticUnit
    {
        //The initial inputs
        private Brep model;
        private double energy; // the range on the interface is 0.1-1, the ratio of the max energy
        private double angle; // the input step angle
        private int speed; // the type of the speed from the dropdown list
        private Vector3d direction = Vector3d.Unset;
        private bool addLock;
        RhinoDoc myDoc;


        private Curve skeleton = null;
        private List<Shape> modelCut;
        private List<Lock> locks;
        private Helix spring;

        public IntermittentRotation(Brep Model, bool Curved, Vector3d Direction, double Energy, double Angle, int Speed)
        {
            model = Model;
            energy = Energy;
            angle = Angle;
            direction = Direction;
            modelCut = new List<Shape>();
            myDoc = RhinoDoc.ActiveDoc;
            locks = new List<Lock>();
            speed = Speed;
        }

        public Helix Spring { get => spring; set => spring = value; }
        public List<Lock> Locks { get => locks; set => locks = value; }
        public List<Shape> ModelCut { get => modelCut; set => modelCut = value; }
        public Curve Skeleton { get => skeleton; set => skeleton = value; }
        public Brep Model { get => model; set => model = value; }
        public double Energy { get => energy; set => energy = value; }
        public int Speed { get => speed; set => speed = value; }
        public double Angle { get => angle; set => angle = value; }
    }
}
