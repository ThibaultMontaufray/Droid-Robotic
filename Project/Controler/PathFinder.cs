using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Droid_Robotic
{
    public class PathFinder
    {
        #region Attributes
        private int width;
        private int height;
        private Cell[,] nodes;
        private Cell startNode;
        private Cell endNode;
        private SearchParameters searchParameters;

        public delegate void DisplayMethod(object o = null);
        #endregion

        #region Constructor
        public PathFinder(SearchParameters searchParameters)
        {
            this.searchParameters = searchParameters;
            InitializeNodes(searchParameters.Map);
            this.startNode = this.nodes[searchParameters.StartLocation.X, searchParameters.StartLocation.Y];
            this.startNode.State = Cell.CellState.Open;
            this.endNode = this.nodes[searchParameters.EndLocation.X, searchParameters.EndLocation.Y];
        }
        #endregion

        #region Methods public
        public List<Point> FindPath()
        {
            // The start node is the first entry in the 'open' list
            List<Point> path = new List<Point>();
            bool success = Search(startNode);
            if (success)
            {
                // If a path was found, follow the parents from the end node to build a list of locations
                Cell node = this.endNode;
                while (node.ParentNode != null)
                {
                    path.Add(node.Location);
                    node = node.ParentNode;
                }

                // Reverse the list so it's in the correct order when returned
                path.Reverse();
            }

            return path;
        }

        public static void DisplayRoute(bool[,] map, string title, IEnumerable<Point> path, SearchParameters searchParameters, DisplayMethod func = null)
        {
            if (func == null) { func = DisplayConsolError; }
            func(string.Format("{0}\r\n", title));
            for (int y = map.GetLength(1) - 1; y >= 0; y--) // Invert the Y-axis so that coordinate 0,0 is shown in the bottom-left
            {
                for (int x = 0; x < map.GetLength(0); x++)
                {
                    if (searchParameters.StartLocation.Equals(new Point(x, y)))
                        // Show the start position
                        func('S');
                    else if (searchParameters.EndLocation.Equals(new Point(x, y)))
                        // Show the end position
                        func('F');
                    else if (map[x, y] == false)
                        // Show any barriers
                        func('░');
                    else if (path.Where(p => p.X == x && p.Y == y).Any())
                        // Show the path in between
                        func('*');
                    else
                        // Show nodes that aren't part of the path
                        func('·');
                }
                func();
            }
        }
        public static void MapReset(ref bool[,] map, int columnsCount, int rowsCount)
        {
            map = new bool[columnsCount, rowsCount];
            for (int y = 0; y < rowsCount; y++)
                for (int x = 0; x < columnsCount; x++)
                    map[x, y] = true;
        }
        public static void MapAddObstacle(ref bool[,] map, Coord c, int columnsCount, int rowsCount)
        {
            // TODO : fix that bug coord minus
            if (c.R < 0 || c.Q < 0 || c.R > rowsCount || c.Q > columnsCount) { return; }
            map[c.Q, c.R] = false;
            if (c.Q + 1 < columnsCount) map[c.Q + 1, c.R] = false;
            if (c.R + 1 < rowsCount) map[c.Q, c.R + 1] = false;
            if (c.Q + 1 < columnsCount && c.R + 1 < rowsCount) map[c.Q + 1, c.R + 1] = false;
            if (c.Q > 0) map[c.Q - 1, c.R] = false;
            if (c.R > 0) map[c.Q, c.R - 1] = false;
            if (c.Q > 0 && c.R > 0) map[c.Q - 1, c.R - 1] = false;
        }
        #endregion

        #region Methods private
        private void InitializeNodes(bool[,] map)
        {
            this.width = map.GetLength(0);
            this.height = map.GetLength(1);
            this.nodes = new Cell[this.width, this.height];
            for (int y = 0; y < this.height; y++)
            {
                for (int x = 0; x < this.width; x++)
                {
                    this.nodes[x, y] = new Cell(x, y, map[x, y], this.searchParameters.EndLocation);
                }
            }
        }
        private bool Search(Cell currentNode)
        {
            // Set the current node to Closed since it cannot be traversed more than once
            currentNode.State = Cell.CellState.Closed;
            List<Cell> nextNodes = GetAdjacentWalkableNodes(currentNode);

            // Sort by F-value so that the shortest possible routes are considered first
            nextNodes.Sort((node1, node2) => node1.Cost.CompareTo(node2.Cost));
            foreach (var nextNode in nextNodes)
            {
                // Check whether the end node has been reached
                if (nextNode.Location == this.endNode.Location)
                {
                    return true;
                }
                else
                {
                    // If not, check the next set of nodes
                    if (Search(nextNode)) // Note: Recurses back into Search(Node)
                        return true;
                }
            }

            // The method returns false if this path leads to be a dead end
            return false;
        }
        private List<Cell> GetAdjacentWalkableNodes(Cell fromNode)
        {
            List<Cell> walkableNodes = new List<Cell>();
            IEnumerable<Point> nextLocations = GetAdjacentLocations(fromNode.Location);

            foreach (var location in nextLocations)
            {
                int x = location.X;
                int y = location.Y;

                // Stay within the grid's boundaries
                if (x < 0 || x >= this.width || y < 0 || y >= this.height)
                    continue;

                Cell node = this.nodes[x, y];
                // Ignore non-walkable nodes
                if (!node.IsWalkable)
                    continue;

                // Ignore already-closed nodes
                if (node.State == Cell.CellState.Closed)
                    continue;

                // Already-open nodes are only added to the list if their G-value is lower going via this route.
                if (node.State == Cell.CellState.Open)
                {
                    float traversalCost = Cell.GetTraversalCost(node.Location, node.ParentNode.Location);
                    float gTemp = fromNode.CostFromStart + traversalCost;
                    if (gTemp < node.CostFromStart)
                    {
                        node.ParentNode = fromNode;
                        walkableNodes.Add(node);
                    }
                }
                else
                {
                    // If it's untested, set the parent and flag it as 'Open' for consideration
                    node.ParentNode = fromNode;
                    node.State = Cell.CellState.Open;
                    walkableNodes.Add(node);
                }
            }

            return walkableNodes;
        }
        private static IEnumerable<Point> GetAdjacentLocations(Point fromLocation)
        {
            return new Point[]
            {
                new Point(fromLocation.X-1, fromLocation.Y-1),
                new Point(fromLocation.X-1, fromLocation.Y  ),
                new Point(fromLocation.X-1, fromLocation.Y+1),
                new Point(fromLocation.X,   fromLocation.Y+1),
                new Point(fromLocation.X+1, fromLocation.Y+1),
                new Point(fromLocation.X+1, fromLocation.Y  ),
                new Point(fromLocation.X+1, fromLocation.Y-1),
                new Point(fromLocation.X,   fromLocation.Y-1)
            };
        }

        public static void DisplayConsol(object o = null)
        {
            if (o is char)
            {
                Console.Write(o);
            }
            else
            {
                Console.WriteLine(o);
            }
        }
        public static void DisplayConsolError(object o = null)
        {
            if (o is char)
            {
                Console.Error.Write(o);
            }
            else
            {
                Console.Error.WriteLine(o);
            }
        }
        #endregion
    }
}
