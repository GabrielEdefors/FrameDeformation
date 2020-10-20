using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;

namespace FrameDeformation
{
	class LinearStrategy : ISolveStrategy
	{
		public Vector<double> Solve(LinearStiffnessMatrix stiffnessMatrix, Vector<double> forceVector, List<int> boundaryDofs, List<double> boundaryConstraints)
		{
			int nDof = forceVector.Count;

			// Find all dofs where force is known
			List<int> allDofs = Enumerable.Range(0, nDof).ToList();

			List<int> unknownDofs = allDofs.Except(boundaryDofs).ToList();

			// Add the know displacements to the result
			Vector<double> displacementVector = Vector<double>.Build.Dense(nDof);

			for (int i = 0; i < boundaryDofs.Count; i++)
			{
				displacementVector[boundaryDofs[i]] = boundaryConstraints[i];
			}

			// Pick out part known elements of force vector
			int nrUnknownDofs = unknownDofs.Count;
			Vector<double> knownForces = Vector<double>.Build.Dense(unknownDofs.Count);

			for (int i = 0; i < unknownDofs.Count; i++)
				knownForces[i] = forceVector[unknownDofs[i]];


			Matrix<double> unkownKnownK = Matrix<double>.Build.Dense(nrUnknownDofs, boundaryDofs.Count);


			for (int i = 0; i < unknownDofs.Count; i++)
			{
				for (int j = 0; j < boundaryDofs.Count; j++)
				{
					unkownKnownK[i, j] = stiffnessMatrix.ReducedK[unknownDofs[i], boundaryDofs[j]];
				}
				knownForces[i] = forceVector[unknownDofs[i]];
			}

			// Solve for the unknown displacements 
			Vector<double> unknownDisplacements = stiffnessMatrix.ReducedK.Inverse().Multiply(knownForces.Subtract(unkownKnownK.Multiply(Vector<double>.Build.Dense(boundaryConstraints.ToArray()))));

			// Insert the calculated displacements
			for (int i = 0; i < unknownDofs.Count; i++)
			{
				displacementVector[unknownDofs[i]] = unknownDisplacements[i];
			}

			return displacementVector;
		}

		public Vector<double> Solve(LinearStiffnessMatrix stiffnessMatrix, Vector<double> forceVector, List<int> boundaryDofs, List<double> boundaryConstraints, List<double> normalForces)
		{
			throw new NotImplementedException();
		}
	}
}
