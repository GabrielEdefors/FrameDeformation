using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace FrameDeformation
{
	class LoadNode : AuxiliaryNode
	{
		// Properties	
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

	}
}
