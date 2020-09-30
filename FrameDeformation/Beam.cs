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
	class Beam
	{
		// Properties	
		public List<Node> Nodes = new List<Node>();
		public double Area { get; set; } = 0.0;
		public double StiffnessModulus { get; set; } = 0.0;
		public double ElemLength { get; set; } = 0.0;
		public double MomentArea { get; set; } = 0.0;
		public double NrEvalPoints { get; set; } = 100;
		public double TranverseLoad { get; set; } = 0;

		private LinearAlgebra.Matrix<double> G = LinearAlgebra.Matrix<double>.Build.Dense(6, 6);

		// Constructor
		public Beam(Node node1,
				   Node node2,
				   double area,
				   double stiffnessModulus,
				   double momentArea,
				   double transverseLoad)
		{
			Nodes.Add(node1);
			Nodes.Add(node2);
			StiffnessModulus = stiffnessModulus;
			Area = area;
			MomentArea = momentArea;
			TranverseLoad = transverseLoad;

			// Compute the element lenght and the transformation matrix G
			double x1 = Nodes[0].Point.X;
			double y1 = Nodes[0].Point.Y;

			double x2 = Nodes[1].Point.X;
			double y2 = Nodes[1].Point.Y;

			ElemLength = Math.Sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

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
		}

		public LinearAlgebra.Matrix<double> ComputeStiffnessMatrix()
		{
			LinearAlgebra.Matrix<double> K = LinearAlgebra.Matrix<double>.Build.Dense(6, 6);

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
			LinearAlgebra.Matrix<double> GT = G.Transpose();
			LinearAlgebra.Matrix<double> KGlobal = (GT.Multiply(K)).Multiply(G);

			return KGlobal;
		}

		public List<double> computeShearForce(List<double> globalElemDisp)
		{
			List<double> V = new List<double> { };

			for(int i = 0; i < NrEvalPoints; i++)
			{
				double xi = i / NrEvalPoints * ElemLength;
				LinearAlgebra.Vector<double> dBidx = LinearAlgebra.Vector<double>.Build.Dense(4);
				dBidx[0] = 12 / Math.Pow(ElemLength, 3);
				dBidx[1] = 6 / Math.Pow(ElemLength, 2);
				dBidx[2] = - 12 / Math.Pow(ElemLength, 4);
				dBidx[3] = 6 / Math.Pow(ElemLength, 2);

				// Transform from global to local coordinates
				LinearAlgebra.Vector<double> globalElemDispVector = LinearAlgebra.Vector<double>.Build.Dense(globalElemDisp.ToArray());
				LinearAlgebra.Vector<double> localElemDisp = G.Multiply(globalElemDispVector);

				// Extract displacements associated to beam dofs
				LinearAlgebra.Vector<double> localElemDispBeam = LinearAlgebra.Vector<double>.Build.Dense(4);
				localElemDispBeam[0] = localElemDisp[1];
				localElemDispBeam[1] = localElemDisp[2];
				localElemDispBeam[2] = localElemDisp[4];
				localElemDispBeam[3] = localElemDisp[5];

				// Particular solution for uniformly distributed load
				double Vpi = -TranverseLoad * (xi - ElemLength / 2);

				V.Add(-StiffnessModulus * MomentArea * (dBidx.DotProduct(localElemDispBeam) + Vpi));

				return V;

			}
		}

		public List<double> computebendingMoment(List<double> globalElemDisp)
		{
			List<double> M = new List<double> { };

			for (int i = 0; i < NrEvalPoints; i++)
			{
				double xi = i / NrEvalPoints * ElemLength;
				LinearAlgebra.Vector<double> Bi = LinearAlgebra.Vector<double>.Build.Dense(4);
				Bi[0] = -6 / Math.Pow(ElemLength, 2) + 12 * xi / Math.Pow(ElemLength, 3);
				Bi[1] = -4 / ElemLength + 6 * xi / Math.Pow(ElemLength, 2);
				Bi[2] = 6 / Math.Pow(ElemLength, 2) - 12 * xi / Math.Pow(ElemLength, 4);
				Bi[3] = -2 / ElemLength + 6 * xi / Math.Pow(ElemLength, 2);

				// Transform from global to local coordinates
				LinearAlgebra.Vector<double> globalElemDispVector = LinearAlgebra.Vector<double>.Build.Dense(globalElemDisp.ToArray());
				LinearAlgebra.Vector<double> localElemDisp = G.Multiply(globalElemDispVector);

				// Extract displacements associated to beam dofs
				LinearAlgebra.Vector<double> localElemDispBeam = LinearAlgebra.Vector<double>.Build.Dense(4);
				localElemDispBeam[0] = localElemDisp[1];
				localElemDispBeam[1] = localElemDisp[2];
				localElemDispBeam[2] = localElemDisp[4];
				localElemDispBeam[3] = localElemDisp[5];

				// Particular solution for uniformly distributed load
				double Mpi = TranverseLoad * (xi * xi / 2 - ElemLength * xi / 2 + ElemLength * ElemLength / 12);

				M.Add(StiffnessModulus * MomentArea * (Bi.DotProduct(localElemDispBeam) + Mpi));
			}
		}

	}
}
