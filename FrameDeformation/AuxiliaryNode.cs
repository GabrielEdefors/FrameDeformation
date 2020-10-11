using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace FrameDeformation
{
	class AuxiliaryNode
	{
		public Point3d Point { get; set; } = new Point3d();

		public AuxiliaryNode() { }

		public static bool operator ==(AuxiliaryNode node1, Node node2)
		{
			// If they have the same point they count as the same node
			if (node1.Point.Equals(node2.Point))
			{
				return true;
			}
			return false;
		}

		public static bool operator !=(AuxiliaryNode node1, Node node2)
		{
			// If they have the same point they count as the same node
			if (node1.Point.Equals(node2.Point) == false)
			{
				return true;
			}
			return false;
		}
	}
}
