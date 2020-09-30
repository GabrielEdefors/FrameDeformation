using System;
using System.Collections.Generic;
using LinearAlgebra = MathNet.Numerics.LinearAlgebra;
using System.Linq;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace FrameDeformation
{
	public class FrameDeformationComponent : GH_Component
	{
		public FrameDeformationComponent()
		  : base("FrameDeformation", "FrameDeformation",
			  "Solves the Equalibrium of the Frame Structure",
			  "Frame", "Solver")
		{
		}

		protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
		{
			pManager.AddGenericParameter("Line", "line", "Line of the beam", GH_ParamAccess.list);
			pManager.AddGenericParameter("Constraint Nodes", "constraint nodes", "Constraint nodes objects", GH_ParamAccess.list);
			pManager.AddGenericParameter("Load Nodes", "load nodes", "Load nodes objects", GH_ParamAccess.list);
			pManager.AddNumberParameter("Area", "A", "Cross sectional area of the bar", GH_ParamAccess.list);
			pManager.AddNumberParameter("Youngs Modulus", "E", "Stiffness of the bar", GH_ParamAccess.list);
			pManager.AddNumberParameter("Moment of Area", "I", "Second moment of area of the beam", GH_ParamAccess.list);
			pManager.AddNumberParameter("Scale Factor", "SF", "Scale Sectional Forces Diagrams With Scale Factor", GH_ParamAccess.item);
		}

		protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
		{
			pManager.AddLineParameter("Shear Diagram", "shear diagram", "shear diagram", GH_ParamAccess.list);
			pManager.AddLineParameter("Moment Diagram", "moment diagram", "moment diagram", GH_ParamAccess.list);
		}

		protected override void SolveInstance(IGH_DataAccess DA)
		{
			// Retrive data from component
			List<double> A = new List<double>();
			List<double> E = new List<double>();
			List<double> I = new List<double>();
			List<Line> lines = new List<Line>();
			List<ContstraintNode> rNodes = new List<ContstraintNode>();
			List<LoadNode> loadNodes = new List<LoadNode>();
			double scaleFactor = 1.0;

			DA.GetDataList("Line", lines);
			DA.GetDataList("Area", A);
			DA.GetDataList("Youngs Modulus", E);
			DA.GetDataList("Constraint Nodes", rNodes);
			DA.GetDataList("Load Nodes", loadNodes);
			DA.GetData("Scale Factor", ref scaleFactor);

			// The length of A and E must be the same as lines
			if ((A.Count != E.Count) | (E.Count != lines.Count) | (I.Count != lines.Count))
			{
				throw new ArgumentException("Length of A, I and E must equal length of Line");
			}

			// Create one list to store the nodes and one list to store the bars
			List<Node> frameNodes = new List<Node>();
			List<Beam> frameBeams = new List<Beam>();


			// Topology matrix to keep track of element dofs
			List<List<int>> eDof = new List<List<int>>();


			// Loop trough each line and create nodes at end points 
			for (int i = 0; i < lines.Count; i++)
			{

				Node node1 = new Node(lines[i].From);
				Node node2 = new Node(lines[i].To);

				// To keep track if the node is unique
				bool unique1 = true;
				bool unique2 = true;


				// Check if node is unique, if so give it an ID and degress of freedom
				foreach (Node existingNode in frameNodes)
				{

					// If not unique use an already identified node
					if (node1 == existingNode)
					{
						node1 = existingNode;
						unique1 = false;
					}

					if (node2 == existingNode)
					{
						node2 = existingNode;
						unique2 = false;
					}
				}

				// If unique give it an ID
				if (unique1)
				{
					int id_node_1 = frameNodes.Count;
					node1.ID = id_node_1;
					node1.Dofs = System.Linq.Enumerable.Range(id_node_1 * 3, 3).ToList();

					// Check if any boundary node or load node exist at current node
					foreach (ContstraintNode rNode in rNodes)
					{
						if (rNode == node1)
						{
							// Add restraint data
							node1.ConstraintX = rNode.ConstraintX;
							node1.ConstraintY = rNode.ConstraintY;
							node1.ConstraintR = rNode.ConstraintR;
						}
					}

					foreach (LoadNode loadNode in loadNodes)
					{
						if (loadNode == node1)
						{
							// Add force data
							node1.ForceX = loadNode.ForceX;
							node1.ForceY = loadNode.ForceY;
							node1.MomentR = loadNode.MomentR;
						}
					}

					// Finally add the node
					frameNodes.Add(node1);

				}

				if (unique2)
				{
					int id_node_2 = frameNodes.Count;
					node2.ID = id_node_2;
					node2.Dofs = System.Linq.Enumerable.Range(id_node_2 * 3, 3).ToList();

					// Check if any boundary node or load node exist at current node
					foreach (ContstraintNode rNode in rNodes)
					{
						if (rNode == node2)
						{
							// Add constraint data
							node2.ConstraintX = rNode.ConstraintX;
							node2.ConstraintY = rNode.ConstraintY;
							node2.ConstraintR = rNode.ConstraintR;
						}
					}


					foreach (LoadNode loadNode in loadNodes)
					{
						if (loadNode == node2)
						{
							// Add force data
							node2.ForceX = loadNode.ForceX;
							node2.ForceY = loadNode.ForceY;
							node2.MomentR = loadNode.MomentR;
						}
					}

					// Finally add the node
					frameNodes.Add(node2);
				}


				// Create a beam object between the nodes
				Beam beam = new Beam(node1, node2, A[i], E[i], I[i]);
				frameBeams.Add(beam);

				// Topology matrix
				List<int> dofs1 = beam.Nodes[0].Dofs;
				List<int> dofs2 = beam.Nodes[1].Dofs;

				List<int> eDofRow = new List<int>();
				eDofRow.AddRange(dofs1);
				eDofRow.AddRange(dofs2);
				eDof.Add(eDofRow);

			}

			int nDof = frameNodes.Count * 3;
			int nElem = eDof.Count;

			// Loop trough each node and construct a load vector and boundary vector
			LinearAlgebra.Vector<double> forceVector = LinearAlgebra.Vector<double>.Build.Dense(nDof);
			List<int> boundaryDofs = new List<int>();
			List<double?> boundaryConstraints = new List<double?>();

			for (int i = 0; i < frameNodes.Count; i++)
			{

				// Load vector
				forceVector[i * 3] = frameNodes[i].ForceX;
				forceVector[i * 3 + 1] = frameNodes[i].ForceY;
				forceVector[i * 3 + 2] = frameNodes[i].MomentR;

				// Boundary vector
				for (int j = 0; j < frameNodes[i].Dofs.Count; j++)
				{
					if (j == 0 && frameNodes[i].ConstraintX != null)
					{
						boundaryDofs.Add(frameNodes[i].Dofs[j]);
						boundaryConstraints.Add(frameNodes[i].ConstraintX);
					}
					else if (j == 1 && frameNodes[i].ConstraintY != null)
					{
						boundaryDofs.Add(frameNodes[i].Dofs[j]);
						boundaryConstraints.Add(frameNodes[i].ConstraintY);
					}

					else if (j == 2 && frameNodes[i].ConstraintR != null)
					{
						boundaryDofs.Add(frameNodes[i].Dofs[j]);
						boundaryConstraints.Add(frameNodes[i].ConstraintR);
					}

				}
			}

			// Loop trough each element, compute local stiffness matrix and assemble into global stiffness matrix
			LinearAlgebra.Matrix<double> K = LinearAlgebra.Matrix<double>.Build.Dense(nDof, nDof);

			for (int i = 0; i < frameBeams.Count; i++)
			{
				LinearAlgebra.Matrix<double> KElem = frameBeams[i].ComputeStiffnessMatrix();

				// Assemble
				for (int k = 0; k < 6; k++)
				{
					for (int l = 0; l < 6; l++)
					{
						K[eDof[i][k], eDof[i][l]] = K[eDof[i][k], eDof[i][l]] + KElem[k, l];
					}
				}
			}

			// Calculate the displacements
			Solver solver = new Solver();
			LinearAlgebra.Vector<double> displacements = solver.solveEquations(K, forceVector, boundaryDofs, boundaryConstraints.Cast<double>().ToList());

			// Loop trough each beam and compute the shear forces and bending moments and save in branches for each element
			// Save the displacement for each node and calculate the stress in each bar
			List<double> elemStress = new List<double> { };
			for (int i = 0; i < nElem; i++)
			{
				double disp1 = displacements[eDof[i][0]];
				double disp2 = displacements[eDof[i][1]];
				double disp3 = displacements[eDof[i][2]];
				double disp4 = displacements[eDof[i][3]];
				double disp5 = displacements[eDof[i][4]];
				double disp6 = displacements[eDof[i][5]];

				// Calculate element shear force
				frameBeams[i].computeShearForce(new List<double> { disp1, disp2, disp3, disp4, disp5, disp6 });

			}
		}


		protected override System.Drawing.Bitmap Icon
		{
			get
			{
				return null;
			}
		}

		public override Guid ComponentGuid
		{
			get { return new Guid("2e9e9f71-7c70-4a07-a149-a15dd30fc1b8"); }
		}
	}
}
