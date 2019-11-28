﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace edaCADPlugin
{
    /// <summary>
    /// This is the class for part representation in both schematic and printed circuit in the geometry
    /// _type: 1-2D (schematic)
    /// _p_ID: part ID
    /// _id: the id used in the schematic
    /// _position: a list of 2D points indicating the positions of the part in either 2D schematics or 3D geometry
    /// _rotation: CCW rotation angle in degree
    /// 
    /// Author: Liang He
    /// Created date: 8/21/2019
    /// </summary>
    /// 
    public class partRep
    {
        private int _type;
        private string _p_ID;
        private int _id;
        private List<double> _position; 
        private List<double> _rotation; 

        public partRep()
        {
            this._type = 1;
            this._position = new List<double>();
            this._rotation = new List<double>();
            this._p_ID = "";
            this._id = -1;
        }

        public partRep(int type, string p_ID, int id, List<double> position, List<double> rotation)
        {
            this._type = type;
            this._p_ID = p_ID;
            this._id = id;
            this._position = position;
            this._rotation = rotation;
        }

        public int Type
        {
            get { return this._type; }
            set { this._type = value; }
        }

        public string P_ID
        {
            get { return this._p_ID; }
            set { this._p_ID = value; }
        }
        public int ID
        {
            get { return this._id; }
            set { this._id = value; }
        }
        public List<double> Position
        {
            get { return this._position; }
            set { this._position = value; }
        }
        public List<double> Rotation
        {
            get { return this._rotation; }
            set { this._rotation = value; }
        }
    }
}
