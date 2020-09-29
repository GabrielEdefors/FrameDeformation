using System;
using System.Collections.Generic;
using LinearAlgebra = MathNet.Numerics.LinearAlgebra;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel.Geometry;
using Rhino.Geometry;

namespace FrameDeformation
{
	class Bar
	{
		// Properties	
		public List<Node> Nodes = new List<Node>();
		public double Area { get; set; } = 0.0;
		public double StiffnessModulus { get; set; } = 0.0;
		public double ElemLength { get; set; } = 0.0;
		public double MomentArea { get; set; } = 0.0;

		private LinearAlgebra.Matrix<double> G = LinearAlgebra.Matrix<double>.Build.Dense(6, 6);

		// Constructor
		public Bar(Node node1,
				   Node node2,
				   double area,
				   double stiffnessModulus,
				   double momentArea)
		{
			Nodes.Add(node1);
			Nodes.Add(node2);
			StiffnessModulus = stiffnessModulus;
			Area = area;
			MomentArea = momentArea;
		}

		public LinearAlgebra.Matrix<double> ComputeStiffnessMatrix()
		{
			LinearAlgebra.Matrix<double> K = LinearAlgebra.Matrix<double>.Build.Dense(6, 6);

			double x1 = Nodes[0].Point.X;
			double y1 = Nodes[0].Point.Y;

			double x2 = Nodes[1].Point.X;
			double y2 = Nodes[1].Point.Y;

			ElemLength = Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

			K[0, 0] = Area * StiffnessModulus / ElemLength;
			K[0, 3] = -Area * StiffnessModulus / ElemLength;
			K[3, 1] = -Area * StiffnessModulus / ElemLength;
			K[3, 3] = Area * StiffnessModulus / ElemLength;

			K[1, 1] = 12 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 3);
			K[1, 2] = 6 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 2);
			K[1, 4] = -12 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 3);
			K[1, 5] = 6 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 2);

			K[2, 1] = 6 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 2);
			K[2, 2] = 4 * StiffnessModulus * MomentArea / ElemLength;
			K[2, 4] = - 6 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 2);
			K[2, 5] = 2 * StiffnessModulus * MomentArea / ElemLength;

			K[4, 1] = - 12 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 3);
			K[4, 2] = - 6 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 2);
			K[4, 4] = 12 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 3);
			K[4, 5] = - 6 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 2);

			K[2, 1] = 6 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 2);
			K[2, 2] = 2 * StiffnessModulus * MomentArea / ElemLength;
			K[2, 4] = - 6 * StiffnessModulus * MomentArea / Math.Pow(ElemLength, 2);
			K[2, 5] = 4 * StiffnessModulus * MomentArea / ElemLength;

			// Tranform to global element stiffness matrix using transformation matrix G

			double nxx = (x2 - x1) / ElemLength;
			double nyx = (y2 - y1) / ElemLength;

			G[0, 0] = nxx;
			G[0, 1] = nyx;
			G[1, 0] = -nyx;
			G[1, 1] = nxx;
			G[2, 2] = 1;

			G[3, 3] = nxx;
			G[3, 4] = nyx;
			G[4, 3] = -nyx;
			G[4, 4] = nxx;
			G[5, 5] = 1;

			LinearAlgebra.Matrix<double> GT = G.Transpose();
			LinearAlgebra.Matrix<double> KGlobal = (GT.Multiply(K)).Multiply(G);

			return KGlobal;
		}

	}
}
