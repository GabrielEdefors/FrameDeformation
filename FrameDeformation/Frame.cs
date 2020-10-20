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
		public List<Line> Lines { get; set; } = new List<Line>();
		public List<Node> Nodes { get; set; } = new List<Node>();
		public List<ContstraintNode> RNodes { get; set; } = new List<ContstraintNode>();
		public List<LoadNode> LoadNodes { get; set; } = new List<LoadNode>();
		public List<HingeNode> HingeNodes { get; set; } = new List<HingeNode>();
		public List<Beam> Beams { get; set; } = new List<Beam>();

		public List<double> StiffnessModulus { get; set; } = new List<double>();
		public List<double> SecondMomentArea { get; set; } = new List<double>();
		public List<double> Area { get; set; } = new List<double>();
		public List<double> TransverseLoads { get; set; } = new List<double>();
		public List<double?> BoundaryConstraints { get; set; } = new List<double?>();

		public List<List<int>> EDof { get; set; } = new List<List<int>>();

		public int NDof { get; set; } = 0;
		public int NElem { get; set; } = 0;
		public List<int> BoundaryDofs { get; set; } = new List<int>();
		public ISolveStrategy SolveStrategy { get; set; }

		public LinearAlgebra.Vector<double> ForceVector { get; set; }
		public LinearStiffnessMatrix K { get; set; }
		public GeometricNonlinearStiffnessMatrix KNonLinear { get; set; }


		public Frame(List<Line> lines,
					 List<ContstraintNode> rNodes,
					 List<LoadNode> loadNodes,
					 List<HingeNode> hingeNodes,
					 List<double> E,
					 List<double> I,
					 List<double> A,
					 List<double> transverseLoads,
					 ISolveStrategy solveStrategy)
		{
			Lines = lines;
			RNodes = rNodes;
			LoadNodes = loadNodes;
			HingeNodes = hingeNodes;
			StiffnessModulus = E;
			SecondMomentArea = I;
			Area = A;
			TransverseLoads = transverseLoads;
			SolveStrategy = solveStrategy;

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
				EDof.Add(eDofRow);

			}

			// Calculate number of dofs of the whole system
			NDof = Nodes[Nodes.Count - 1].Dofs[2] + 1;
			NElem = EDof.Count;
		}

		public void CreateForceVector()
		{
			ForceVector = LinearAlgebra.Vector<double>.Build.Dense(NDof);
			for (int i = 0; i < Nodes.Count; i++)
			{

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
			K = new LinearStiffnessMatrix(NDof);

			for (int i = 0; i < Beams.Count; i++)
			{
				LinearAlgebra.Matrix<double> KElem = Beams[i].ComputeStiffnessMatrix();
				LinearAlgebra.Vector<double> fl = Beams[i].ComputeLoadVector();

				// Add element contributions to the load vector
				for (int k = 0; k < 6; k++)
				{
					// Add contributions to the load vector
					ForceVector[EDof[i][k]] = ForceVector[EDof[i][k]] + fl[k];
				}

				// Add element contributions to the stiffness matrix
				K.AddElementContribution(EDof[i], KElem);
			}
		}
		public void CalculateDisplacements()
		{
			K.ComputeReducedMatrix(BoundaryDofs);
			LinearAlgebra.Vector<double> displacements = SolveStrategy.Solve(K, ForceVector, BoundaryDofs, BoundaryConstraints.Cast<double>().ToList());

			// Save the displacements for each beam
			for (int i = 0; i < NElem; i++)
			{
				Beams[i].Solution.NodalDisplacements.Add(displacements[EDof[i][0]]);
				Beams[i].Solution.NodalDisplacements.Add(displacements[EDof[i][1]]);
				Beams[i].Solution.NodalDisplacements.Add(displacements[EDof[i][2]]);
				Beams[i].Solution.NodalDisplacements.Add(displacements[EDof[i][3]]);
				Beams[i].Solution.NodalDisplacements.Add(displacements[EDof[i][4]]);
				Beams[i].Solution.NodalDisplacements.Add(displacements[EDof[i][5]]);
			}
		}

		public void ComputeSectionalForces()
		{


			for (int i = 0; i < NElem; i++)
			{
				// Calculate element shear force
				Beams[i].Solution.ShearForceField = Beams[i].ComputeShearForce(Beams[i].Solution.NodalDisplacements);
				Beams[i].Solution.BendingMomentField = Beams[i].ComputeBendingMoment(Beams[i].Solution.NodalDisplacements);
				Beams[i].Solution.NormalForceField = Beams[i].ComputeNormalForce(Beams[i].Solution.NodalDisplacements);
			}

		}

		/// <summary>
		/// Computes the buckling modes and corresponding buckling factor. The normal forces in reference system are calculated using linear analysis, i.e. small displacements are assumed
		/// </summary>
		public void CalculateBucklingModes()
		{
			if (Beams[0].Solution.NormalForceField == null)
			{
				throw new InvalidOperationException("Normal forces must be computed before buckling analysis can be performed!");
			}

			KNonLinear = new GeometricNonlinearStiffnessMatrix(NDof);

			// Calcualte the gemetric non linear stiffness matrix for the reference load
			for (int i = 0; i < NElem; i++)
			{
				Beams[i].ComputeNonLinearStiffnessMatrix();

				// Next time, assemble in the non linear parts
			}
		}
		
	}
}
