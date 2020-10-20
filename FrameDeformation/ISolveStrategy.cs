using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;

namespace FrameDeformation
{
	interface ISolveStrategy
	{
		Vector<double> Solve(LinearStiffnessMatrix stiffnessMatrix, Vector<double> forceVector, List<int> boundaryDofs, List<double> boundaryConstraints);
		Vector<double> Solve(LinearStiffnessMatrix stiffnessMatrix, Vector<double> forceVector, List<int> boundaryDofs, List<double> boundaryConstraints, List<double> normalForces);
	
	}
}
