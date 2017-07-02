using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Droid_Robotic
{
    public enum NodeType
    {
        IN,
        OUT,
        HIDDEN
    }

    public struct Node
    {
        public NodeType Type;
        public int InputValue;
        public int OutputValue;
    }

}
