﻿using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Droid_Robotic
{
    public class SearchParameters
    {
        #region Attribute
        public Point StartLocation { get; set; }
        public Point EndLocation { get; set; }
        public bool[,] Map { get; set; }
        #endregion

        #region Constructor
        public SearchParameters(Point startLocation, Point endLocation, bool[,] map)
        {
            this.StartLocation = startLocation;
            this.EndLocation = endLocation;
            this.Map = map;
        }
        #endregion
    }
}
