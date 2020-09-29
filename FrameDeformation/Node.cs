using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace FrameDeformation
{
	class Node
	{
		// Properties	
		public Point3d Point { get; set; } = new Point3d();
		public double? ConstraintX { get; set; } = null;
		public double? ConstraintY { get; set; } = null;
		public double? ConstraintR { get; set; } = null;
		public double ForceX { get; set; } = 0.0;
		public double ForceY { get; set; } = 0.0;
		public double MomentR { get; set; } = 0.0;
		public int? ID { get; set; } = null;
		public List<int> Dofs { get; set; } = new List<int>();

		// Constructor
		public Node(Point3d point)
		{
			Point = point;
		}

		public static bool operator ==(Node node1, Node node2)
		{
			// If they have the same point they count as the same node
			if (node1.Point.Equals(node2.Point))
			{
				return true;
			}
			return false;
		}

		public static bool operator !=(Node node1, Node node2)
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
