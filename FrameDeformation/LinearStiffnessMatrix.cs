using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;

namespace FrameDeformation
{
	class LinearStiffnessMatrix : StiffnessMatrix
	{

		public LinearStiffnessMatrix(int size)
			: base(size) { }

		public void AddElementContribution(List<int> elemDof, Matrix<double> kElem)
		{
			// Assemble
			for (int k = 0; k < kElem.RowCount; k++)
			{

				for (int l = 0; l < kElem.ColumnCount; l++)
				{
					// Add contributions to the stiffness matrix
					FullK[elemDof[k], elemDof[l]] = FullK[elemDof[k], elemDof[l]] + kElem[k, l];
				}
			}
		}

	}
}
