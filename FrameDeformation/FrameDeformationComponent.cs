using System;
using System.Collections.Generic;
using System.Linq;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
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
			pManager.AddGenericParameter("Hinge Nodes", "hinge nodes", "Hinge nodes objects", GH_ParamAccess.list);
			pManager.AddNumberParameter("Area", "A", "Cross sectional area of the bar", GH_ParamAccess.list);
			pManager.AddNumberParameter("Youngs Modulus", "E", "Stiffness of the bar", GH_ParamAccess.list);
			pManager.AddNumberParameter("Moment of Area", "I", "Second moment of area of the beam", GH_ParamAccess.list);
			pManager.AddNumberParameter("Transverse Load", "Tranverse Load", "Magnitude of the uniformly distributed transverse load", GH_ParamAccess.list);
			pManager[2].Optional = true;
			pManager[3].Optional = true;
			pManager[7].Optional = true;
		}

		protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
		{
			pManager.AddGenericParameter("Shear Diagram", "shear diagram", "shear diagram", GH_ParamAccess.item);
			pManager.AddGenericParameter("Moment Diagram", "moment diagram", "moment diagram", GH_ParamAccess.item);
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
			List<HingeNode> hingeNodes = new List<HingeNode>();
			List<double> transverseLoad = new List<double>();

			DA.GetDataList(0, lines);
			DA.GetDataList(4, A);
			DA.GetDataList(5, E);
			DA.GetDataList(6, I);
			DA.GetDataList(1, rNodes);
			DA.GetDataList(2, loadNodes);
			DA.GetDataList(3, hingeNodes);
			DA.GetDataList(7, transverseLoad);

			// The length of A and E must be the same as lines
			if ((A.Count != E.Count) | (E.Count != lines.Count) | (I.Count != lines.Count))
			{
				throw new ArgumentException("Length of A, I and E must equal length of Line");
			}

			// Create a frame and create beam and node objects
			Frame frame = new Frame(lines, rNodes, loadNodes, hingeNodes, E, I, A, transverseLoad);
			frame.EstablishTopology();

			// Calculate force vector and displacement lists
			frame.CreateBoundaryVector();
			frame.CreateForceVector();

			// Calculate the displacements
			frame.CalculateDisplacements();

			// Loop trough each beam and compute the shear forces and bending moments and save in branches for each element
			var VTree = new Grasshopper.DataTree<double>();
			var MTree = new Grasshopper.DataTree<double>();

			for (int i = 0; i < nElem; i++)
			{
				double disp1 = displacements[eDof[i][0]];
				double disp2 = displacements[eDof[i][1]];
				double disp3 = displacements[eDof[i][2]];
				double disp4 = displacements[eDof[i][3]];
				double disp5 = displacements[eDof[i][4]];
				double disp6 = displacements[eDof[i][5]];

				// Calculate element shear force
				List<double> VBranch = frameBeams[i].ComputeShearForce(new List<double> { disp1, disp2, disp3, disp4, disp5, disp6 });
				List<double> MBranch = frameBeams[i].ComputeBendingMoment(new List<double> { disp1, disp2, disp3, disp4, disp5, disp6 });

				// Add the respective tree
				GH_Path pth = new GH_Path(i);
				VTree.AddRange(VBranch, pth);
				MTree.AddRange(MBranch, pth);
			}

			DA.SetDataTree(0, VTree);
			DA.SetDataTree(1, MTree);

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
