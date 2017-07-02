using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Droid_Robotic
{
    public class Cell
    {
        #region Enum
        public enum CellState
        {
            Untested,
            Open,
            Closed
        }
        #endregion

        #region Attribute
        private Cell parentNode;
        #endregion

        #region Properties
        public Point Location { get; private set; }
        public bool IsWalkable { get; set; }
        public float CostFromStart { get; private set; }
        public float costRelativeToEnd { get; private set; }
        public CellState State { get; set; }
        public float Cost
        {
            get { return this.CostFromStart + this.costRelativeToEnd; }
        }
        public Cell ParentNode
        {
            get { return this.parentNode; }
            set
            {
                // When setting the parent, also calculate the traversal cost from the start node to here (the 'G' value)
                this.parentNode = value;
                this.CostFromStart = this.parentNode.CostFromStart + GetTraversalCost(this.Location, this.parentNode.Location);
            }
        }
        #endregion

        #region Constructor
        public Cell(int x, int y, bool isWalkable, Point endLocation)
        {
            this.Location = new Point(x, y);
            this.State = CellState.Untested;
            this.IsWalkable = isWalkable;
            this.costRelativeToEnd = GetTraversalCost(this.Location, endLocation);
            this.CostFromStart = 0;
        }
        #endregion

        #region Methods public
        public override string ToString()
        {
            return string.Format("{0}, {1}: {2}", this.Location.X, this.Location.Y, this.State);
        }
        #endregion

        #region Methods private
        internal static float GetTraversalCost(Point location, Point otherLocation)
        {
            float deltaX = otherLocation.X - location.X;
            float deltaY = otherLocation.Y - location.Y;
            return (float)Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
        }
        #endregion
    }
}
