using System;
using System.Collections.Generic;
using System.Linq;
using LinearAlgebra = MathNet.Numerics.LinearAlgebra;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace FrameDeformation
{
	class Frame
	{
		public List<Line> Lines = new List<Line>();
		public List<Node> Nodes = new List<Node>();
		public List<ContstraintNode> RNodes = new List<ContstraintNode>();
		public List<LoadNode> LoadNodes = new List<LoadNode>();
		public List<HingeNode> HingeNodes = new List<HingeNode>();
		public List<Beam> Beams = new List<Beam>();

		public List<double> StiffnessModulus = new List<double>();
		public List<double> SecondMomentArea = new List<double>();
		public List<double> Area = new List<double>();
		public List<double> TransverseLoads = new List<double>();
		public List<double?> BoundaryConstraints = new List<double?>();

		public List<List<int>> eDof = new List<List<int>>();

		public int NDof = 0;
		public int NElem = 0;
		public List<int> BoundaryDofs = new List<int>();

		private LinearAlgebra.Vector<double> ForceVector = LinearAlgebra.Vector<double>.Build.Dense(0);
		private LinearAlgebra.Matrix<double> K = LinearAlgebra.Matrix<double>.Build.Dense(0, 0);

		public Frame(List<Line> lines,
					 List<ContstraintNode> rNodes,
					 List<LoadNode> loadNodes,
					 List<HingeNode> hingeNodes,
					 List<double> E,
					 List<double> I,
					 List<double> A,
					 List<double> transverseLoads)
		{
			Lines = lines;
			RNodes = rNodes;
			LoadNodes = loadNodes;
			HingeNodes = hingeNodes;
			StiffnessModulus = E;
			SecondMomentArea = I;
			Area = A;
			TransverseLoads = transverseLoads;

		}

		public void EstablishTopology()
		{

			// Loop trough each line and create nodes at end points 
			for (int i = 0; i < Lines.Count; i++)
			{

				Node node1 = new Node(Lines[i].From);
				Node node2 = new Node(Lines[i].To);

				// To keep track if the node is unique
				bool unique1 = true;
				bool unique2 = true;


				// Check if node is unique, if so give it an ID and degress of freedom
				foreach (Node existingNode in Nodes)
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
					int id_node_1 = Nodes.Count;
					node1.ID = id_node_1;

					// Use the dofs that come after the last nodes dofs
					if (Nodes.Count == 0)
						node1.Dofs = System.Linq.Enumerable.Range(0, 3).ToList();
					else
						node1.Dofs = System.Linq.Enumerable.Range(Nodes[(Nodes.Count - 1)].Dofs[2] + 1, 3).ToList();

					// Check if any boundary node or load node exist at current node
					foreach (ContstraintNode rNode in RNodes)
					{
						if (rNode == node1)
						{
							// Add restraint data
							node1.ConstraintX = rNode.ConstraintX;
							node1.ConstraintY = rNode.ConstraintY;
							node1.ConstraintR = rNode.ConstraintR;
						}
					}

					foreach (LoadNode loadNode in LoadNodes)
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
					Nodes.Add(node1);

				}
				else
				{
					foreach (HingeNode hingeNode in HingeNodes)
					{
						if (hingeNode == node1)
						{
							// If the node already exists and thus is connected to a beam a new fictive node is created
							// that has the same translational dofs but different rotational dofs
							Node existingNode = node1;
							node1 = new Node(Lines[i].From);
							int id_node1 = Nodes.Count;
							node1.ID = id_node1;
							node1.Dofs.Add(existingNode.Dofs[0]);
							node1.Dofs.Add(existingNode.Dofs[1]);
							node1.Dofs.Add(id_node1 * 3);

							// Add the hinge node
							Nodes.Add(node1);

						}
					}
				}

				if (unique2)
				{
					int id_node_2 = Nodes.Count;
					node2.ID = id_node_2;

					// Use the dofs that come after the last nodes dofs
					node2.Dofs = System.Linq.Enumerable.Range(Nodes[(Nodes.Count - 1)].Dofs[2] + 1, 3).ToList();

					// Check if any boundary node, load node or hinge node exist at current node
					foreach (ContstraintNode rNode in RNodes)
					{
						if (rNode == node2)
						{
							// Add constraint data
							node2.ConstraintX = rNode.ConstraintX;
							node2.ConstraintY = rNode.ConstraintY;
							node2.ConstraintR = rNode.ConstraintR;
						}
					}


					foreach (LoadNode loadNode in LoadNodes)
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
					Nodes.Add(node2);
				}
				else
				{
					foreach (HingeNode hingeNode in HingeNodes)
					{
						if (hingeNode == node2)
						{
							// If the node already exists and thus is connected to a beam a new fictive node is created
							// that has the same translational dofs but different rotational dofs
							Node existingNode = node2;
							node2 = new Node(Lines[i].From);
							int id_node2 = Nodes.Count;
							node2.ID = id_node2;
							node2.Dofs.Add(existingNode.Dofs[0]);
							node2.Dofs.Add(existingNode.Dofs[1]);
							node2.Dofs.Add(id_node2 * 3);

							// Add the new hinge node
							Nodes.Add(node2);
						}
					}
				}

				// Create a beam object between the nodes
				Beam beam = new Beam(node1, node2, Area[i], StiffnessModulus[i], SecondMomentArea[i], TransverseLoads[i]);
				Beams.Add(beam);

				// Topology matrix
				List<int> dofs1 = beam.Nodes[0].Dofs;
				List<int> dofs2 = beam.Nodes[1].Dofs;

				List<int> eDofRow = new List<int>();
				eDofRow.AddRange(dofs1);
				eDofRow.AddRange(dofs2);
				eDof.Add(eDofRow);

			}

			// Calculate number of dofs of the whole system
			NDof = Nodes[Nodes.Count - 1].Dofs[2] + 1;
			NElem = eDof.Count;
		}

		public void CreateForceVector()
		{

			for (int i = 0; i < Nodes.Count; i++)
			{

				LinearAlgebra.Vector<double> ForceVector = LinearAlgebra.Vector<double>.Build.Dense(NDof);
				ForceVector[Nodes[i].Dofs[0]] = Nodes[i].ForceX;
				ForceVector[Nodes[i].Dofs[1]] = Nodes[i].ForceY;
				ForceVector[Nodes[i].Dofs[2]] = Nodes[i].MomentR;

			}
		}

		public void CreateBoundaryVector()
		{
			for (int i = 0; i < Nodes.Count; i++)
			{
				// Boundary vector
				for (int j = 0; j < Nodes[i].Dofs.Count; j++)
				{
					if (j == 0 && Nodes[i].ConstraintX != null)
					{
						BoundaryDofs.Add(Nodes[i].Dofs[j]);
						BoundaryConstraints.Add(Nodes[i].ConstraintX);
					}
					else if (j == 1 && Nodes[i].ConstraintY != null)
					{
						BoundaryDofs.Add(Nodes[i].Dofs[j]);
						BoundaryConstraints.Add(Nodes[i].ConstraintY);
					}

					else if (j == 2 && Nodes[i].ConstraintR != null)
					{
						BoundaryDofs.Add(Nodes[i].Dofs[j]);
						BoundaryConstraints.Add(Nodes[i].ConstraintR);
					}

				}
			}
		}

		public void AssembleSystem()
		{

			// Loop trough each element, compute local stiffness matrix and assemble into global stiffness matrix
			LinearAlgebra.Matrix<double> K = LinearAlgebra.Matrix<double>.Build.Dense(NDof, NDof);

			for (int i = 0; i < Beams.Count; i++)
			{
				LinearAlgebra.Matrix<double> KElem = Beams[i].ComputeStiffnessMatrix();
				LinearAlgebra.Vector<double> fl = Beams[i].ComputeLoadVector();

				// Assemble
				for (int k = 0; k < 6; k++)
				{
					// Add contributions to the load vector
					ForceVector[eDof[i][k]] = ForceVector[eDof[i][k]] + fl[k];

					for (int l = 0; l < 6; l++)
					{
						// Add contributions to the stiffness matrix
						K[eDof[i][k], eDof[i][l]] = K[eDof[i][k], eDof[i][l]] + KElem[k, l];
					}
				}
			}
		}
		public void CalculateDisplacements()
		{
			LinearAlgebra.Vector<double> displacements = Solver.SolveLinearEquations(K, ForceVector, BoundaryDofs, BoundaryConstraints.Cast<double>().ToList());
		}


	}
}
