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
		public double SecondMomentArea { get; set; } = 0.0;
		public double NrEvalPoints { get; set; } = 100;
		public double TransverseLoad { get; set; } = 0;
		public Solution Solution { get; set; }
		private LinearAlgebra.Matrix<double> G { get; set; } = LinearAlgebra.Matrix<double>.Build.Dense(6, 6);
		private LinearAlgebra.Matrix<double> K { get; set; } = LinearAlgebra.Matrix<double>.Build.Dense(6, 6);
		private LinearAlgebra.Matrix<double> Kg { get; set; } = LinearAlgebra.Matrix<double>.Build.Dense(6, 6);



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
			SecondMomentArea = momentArea;
			TransverseLoad = transverseLoad;
			Solution = new Solution();

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

			K[0, 0] = Area * StiffnessModulus / ElemLength;
			K[0, 3] = -Area * StiffnessModulus / ElemLength;
			K[3, 0] = -Area * StiffnessModulus / ElemLength;
			K[3, 3] = Area * StiffnessModulus / ElemLength;

			K[1, 1] = 12 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 3);
			K[1, 2] = 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2);
			K[1, 4] = -12 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 3);
			K[1, 5] = 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2);

			K[2, 1] = 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2);
			K[2, 2] = 4 * StiffnessModulus * SecondMomentArea / ElemLength;
			K[2, 4] = - 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2);
			K[2, 5] = 2 * StiffnessModulus * SecondMomentArea / ElemLength;

			K[4, 1] = - 12 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 3);
			K[4, 2] = - 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2);
			K[4, 4] = 12 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 3);
			K[4, 5] = - 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2);

			K[5, 1] = 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2);
			K[5, 2] = 2 * StiffnessModulus * SecondMomentArea / ElemLength;
			K[5, 4] = - 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2);
			K[5, 5] = 4 * StiffnessModulus * SecondMomentArea / ElemLength;

			// Tranform to global element stiffness matrix using transformation matrix G
			LinearAlgebra.Matrix<double> GT = G.Transpose();
			LinearAlgebra.Matrix<double> KGlobal = (GT.Multiply(K)).Multiply(G);

			return KGlobal;
		}

		public LinearAlgebra.Vector<double> ComputeLoadVector()
		{
			LinearAlgebra.Vector<double> fl = LinearAlgebra.Vector<double>.Build.Dense(6);

			fl[1] = TransverseLoad * ElemLength / 2;
			fl[2] = TransverseLoad * ElemLength * ElemLength / 12;
			fl[4] = TransverseLoad * ElemLength / 2;
			fl[5] = -TransverseLoad * ElemLength * ElemLength / 12;

			// Transform
			LinearAlgebra.Vector<double> flGlobal = G.Transpose().Multiply(fl);

			return flGlobal;
		}

		public List<double> ComputeShearForce(List<double> globalElemDisp)
		{
			List<double> V = new List<double> { };

			for(int i = 0; i < NrEvalPoints; i++)
			{
				double xi = i / (NrEvalPoints - 1) * ElemLength;
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
				double Vpi = -TransverseLoad * (xi - ElemLength / 2);

				V.Add(-StiffnessModulus * SecondMomentArea * (dBidx.DotProduct(localElemDispBeam)) + Vpi);
			}
			return V;
		}

		public List<double> ComputeBendingMoment(List<double> globalElemDisp)
		{
			List<double> M = new List<double> { };

			for (int i = 0; i < NrEvalPoints; i++)
			{
				double xi = i / (NrEvalPoints - 1) * ElemLength;
				LinearAlgebra.Vector<double> Bi = LinearAlgebra.Vector<double>.Build.Dense(4);
				Bi[0] = -6 / Math.Pow(ElemLength, 2) + 12 * xi / Math.Pow(ElemLength, 3);
				Bi[1] = -4 / ElemLength + 6 * xi / Math.Pow(ElemLength, 2);
				Bi[2] = 6 / Math.Pow(ElemLength, 2) - 12 * xi / Math.Pow(ElemLength, 3);
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
				double Mpi = TransverseLoad * (xi * xi / 2 - ElemLength * xi / 2 + ElemLength * ElemLength / 12);

				M.Add(StiffnessModulus * SecondMomentArea * (Bi.DotProduct(localElemDispBeam)) + Mpi);
			}

			return M;
		}

		public List<double> ComputeNormalForce(List<double> globalElemDisp)
		{
			List<double> N = new List<double> { };

			for (int i = 0; i < NrEvalPoints; i++)
			{
				LinearAlgebra.Vector<double> Bi = LinearAlgebra.Vector<double>.Build.Dense(2);
				Bi[0] = -1 / ElemLength;
				Bi[1] = 1 / ElemLength;

				// Transform from global to local coordinates
				LinearAlgebra.Vector<double> globalElemDispVector = LinearAlgebra.Vector<double>.Build.Dense(globalElemDisp.ToArray());
				LinearAlgebra.Vector<double> localElemDisp = G.Multiply(globalElemDispVector);

				// Extract displacements associated to bar dofs
				LinearAlgebra.Vector<double> localElemDispBar = LinearAlgebra.Vector<double>.Build.Dense(2);
				localElemDispBar[0] = localElemDisp[0];
				localElemDispBar[1] = localElemDisp[3];

				double Ni = StiffnessModulus * Area / ElemLength * Bi.DotProduct(localElemDispBar);

				N.Add(Ni);
			}
			return N;
		}

		public List<double> ComputeDisplacements(List<double> globalElemDisp)
		{
			List<double> V = new List<double>();

			// Add logic for transverse displacments

			return V;
		}

		public LinearAlgebra.Matrix<double> ComputeNonLinearStiffnessMatrix()
		{

			// Bar stiffness
			Kg[0, 0] = Area * StiffnessModulus / ElemLength;
			Kg[0, 3] = -Area * StiffnessModulus / ElemLength;
			Kg[3, 0] = -Area * StiffnessModulus / ElemLength;
			Kg[3, 3] = Area * StiffnessModulus / ElemLength;

			// Beam stiffness affected by normal force
			if (Solution.NormalForceField == null)
			{
				throw new InvalidOperationException("Normal forces must be computed before geometric stiffness matrix can be created!");
			}

			double N = Solution.NormalForceField[0];
			double rho = -N * ElemLength * ElemLength / 
				         (Math.PI * Math.PI * StiffnessModulus * SecondMomentArea);
			double k = Math.PI / ElemLength * Math.Sqrt(-rho);

			// Parameters derived from N
			double phi1; double phi2; double phi3; double phi4; double phi5;

			if (N < 0)
			{
				phi1 = k * ElemLength / 2 * Math.Cos(k * ElemLength / 2) / Math.Sin(k * ElemLength / 2);
				phi2 = k * k * ElemLength * ElemLength / (12 * (1 - phi1));
				phi3 = phi1 / 4 + 3 * phi2 / 4;
				phi4 = -phi1 / 2 + 3 * phi2 / 2;
				phi5 = phi1 * phi2;
			}
			else
			{
				phi1 = k * ElemLength / 2 * Math.Cos(k * ElemLength / 2) / Math.Sin(k * ElemLength / 2);
				phi2 = -k * k * ElemLength * ElemLength / (12 * (1 - phi1));
				phi3 = phi1 / 4 + 3 * phi2 / 4;
				phi4 = -phi1 / 2 + 3 * phi2 / 2;
				phi5 = phi1 * phi2;
			}


			Kg[1, 1] = 12 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 3) * phi5;
			Kg[1, 2] = 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2) * phi2;
			Kg[1, 4] = -12 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 3) * phi5;
			Kg[1, 5] = 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2) * phi2;

			Kg[2, 1] = 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2) * phi2;
			Kg[2, 2] = 4 * StiffnessModulus * SecondMomentArea / ElemLength * phi3;
			Kg[2, 4] = -6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2) * phi2;
			Kg[2, 5] = 2 * StiffnessModulus * SecondMomentArea / ElemLength * phi4;

			Kg[4, 1] = -12 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 3) * phi5;
			Kg[4, 2] = -6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2) * phi2;
			Kg[4, 4] = 12 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 3) * phi5;
			Kg[4, 5] = -6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2) * phi2;

			Kg[5, 1] = 6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2) * phi2;
			Kg[5, 2] = 2 * StiffnessModulus * SecondMomentArea / ElemLength * phi4;
			Kg[5, 4] = -6 * StiffnessModulus * SecondMomentArea / Math.Pow(ElemLength, 2) * phi2;
			Kg[5, 5] = 4 * StiffnessModulus * SecondMomentArea / ElemLength * phi3;

			// Tranform to global element stiffness matrix using transformation matrix G
			LinearAlgebra.Matrix<double> GT = G.Transpose();
			LinearAlgebra.Matrix<double> KgGlobal = (GT.Multiply(Kg)).Multiply(G);

			return KgGlobal;
		}
	}
}
