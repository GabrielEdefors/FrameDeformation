using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using LinearAlgebra = MathNet.Numerics.LinearAlgebra;

namespace FrameDeformation
{
	class Solver
	{
		public Solver()
		{
		}

		public LinearAlgebra.Vector<double> solveEquations(LinearAlgebra.Matrix<double> stiffnessMatrix, LinearAlgebra.Vector<double> forceVector, List<int> boundaryDofs, List<double> boundaryConstraints)
		{
			int nDof = forceVector.Count;

			// Find all dofs where force is known
			List<int> allDofs = Enumerable.Range(0, nDof).ToList();

			List<int> unknownDofs = allDofs.Except(boundaryDofs).ToList();

			// Add the know displacements to the result
			LinearAlgebra.Vector<double> displacementVector = LinearAlgebra.Vector<double>.Build.Dense(nDof);

			for (int i = 0; i < boundaryDofs.Count; i++)
			{
				displacementVector[boundaryDofs[i]] = boundaryConstraints[i];
			}

			// Pick out part of matrix corresponding to known forces
			int nrUnknownDofs = unknownDofs.Count;
			LinearAlgebra.Matrix<double> unknownK = LinearAlgebra.Matrix<double>.Build.Dense(nrUnknownDofs, nrUnknownDofs);
			LinearAlgebra.Vector<double> knownForces = LinearAlgebra.Vector<double>.Build.Dense(unknownDofs.Count);

			for (int i = 0; i < unknownDofs.Count; i++)
			{
				for (int j = 0; j < unknownDofs.Count; j++)
				{
					unknownK[i, j] = stiffnessMatrix[unknownDofs[i], unknownDofs[j]];
				}
				knownForces[i] = forceVector[unknownDofs[i]];
			}

			LinearAlgebra.Matrix<double> unkownKnownK = LinearAlgebra.Matrix<double>.Build.Dense(nrUnknownDofs, boundaryDofs.Count);


			for (int i = 0; i < unknownDofs.Count; i++)
			{
				for (int j = 0; j < boundaryDofs.Count; j++)
				{
					unkownKnownK[i, j] = stiffnessMatrix[unknownDofs[i], boundaryDofs[j]];
				}
				knownForces[i] = forceVector[unknownDofs[i]];
			}

			// Solve for the unknown displacements 
			LinearAlgebra.Vector<double> unknownDisplacements = unknownK.Inverse().Multiply(knownForces.Subtract(unkownKnownK.Multiply(LinearAlgebra.Vector<double>.Build.Dense(boundaryConstraints.ToArray()))));

			// Insert the calculated displacements
			for (int i = 0; i < unknownDofs.Count; i++)
			{
				displacementVector[unknownDofs[i]] = unknownDisplacements[i];
			}

			return displacementVector;

		}
	}
}
