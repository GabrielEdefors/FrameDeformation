using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;

namespace FrameDeformation
{
	public abstract class StiffnessMatrix
	{

		int Size { get; set; }
		public Matrix<double> FullK { get; set; }
		public Matrix<double> ReducedK { get; set; }

		public StiffnessMatrix(int size)
		{
			Size = size;
			FullK = Matrix<double>.Build.Dense(size, size);
		}

		public Matrix<double> ComputeReducedMatrix(List<int> boundaryDofs)
		{
			List<int> allDofs = Enumerable.Range(0, Size).ToList();
			List<int> unknownDofs = allDofs.Except(boundaryDofs).ToList();

			// Pick out part of matrix corresponding to known forces
			int nrUnknownDofs = unknownDofs.Count;
			ReducedK = Matrix<double>.Build.Dense(nrUnknownDofs, nrUnknownDofs);

			for (int i = 0; i < nrUnknownDofs; i++)
			{
				for (int j = 0; j < nrUnknownDofs; j++)
				{
					ReducedK[i, j] = FullK[unknownDofs[i], unknownDofs[j]];
				}
			}
			return ReducedK;
		}
	}
}
