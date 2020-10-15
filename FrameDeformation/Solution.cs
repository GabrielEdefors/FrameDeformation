﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FrameDeformation
{
	class Solution
	{
		public List<double> DisplacementField { get; set; } = new List<double>();
		public List<double> NodalDisplacements { get; set; } = new List<double>();
		public List<double> ShearForceField { get; set; } = new List<double>();
		public List<double> BendingMomentField { get; set; } = new List<double>();

		public Solution() { }
	}
}