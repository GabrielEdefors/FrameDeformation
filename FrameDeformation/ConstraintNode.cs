using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace FrameDeformation
{
	class ContstraintNode : AuxiliaryNode
	{
		// Properties	
		public double? ConstraintX { get; set; } = null;
		public double? ConstraintY { get; set; } = null;
		public double? ConstraintR { get; set; } = null;

		// Constructor
		public ContstraintNode(Point3d point,
					double? constraintx,
					double? constrainty,
					double? constraintr)
		{
			Point = point;
			ConstraintX = constraintx;
			ConstraintY = constrainty;
			ConstraintR = constraintr;
		}

	}
}
