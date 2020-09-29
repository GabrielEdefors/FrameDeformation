using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace FrameDeformation
{
	class LoadNode
	{
		// Properties	
		public Point3d Point { get; set; } = new Point3d();
		public double ForceX { get; set; } = 0.0;
		public double ForceY { get; set; } = 0.0;
		public double MomentR { get; set; } = 0.0;

		// Constructor
		public LoadNode(Point3d point, double forcex, double forcey, double momentr)
		{
			Point = point;
			ForceX = forcex;
			ForceY = forcey;
			MomentR = momentr;
		}

		public static bool operator ==(LoadNode node1, Node node2)
		{
			// If they have the same point they count as the same node
			if (node1.Point.Equals(node2.Point))
			{
				return true;
			}
			return false;
		}

		public static bool operator !=(LoadNode node1, Node node2)
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
