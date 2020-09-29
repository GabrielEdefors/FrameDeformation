using System;
using System.Collections.Generic;

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
			if ((A.Count != E.Count) | (E.Count != lines.Count))
			{
				throw new ArgumentException("Length of A and E must equal length of Line");
			}

			// Create one list to store the nodes and one list to store the bars
			List<Node> trussNodes = new List<Node>();
			List<Bar> trussBars = new List<Bar>();
		}


		protected override System.Drawing.Bitmap Icon
		{
			get
			{
				// You can add image files to your project resources and access them like this:
				//return Resources.IconForThisComponent;
				return null;
			}
		}

		/// <summary>
		/// Each component must have a unique Guid to identify it. 
		/// It is vital this Guid doesn't change otherwise old ghx files 
		/// that use the old ID will partially fail during loading.
		/// </summary>
		public override Guid ComponentGuid
		{
			get { return new Guid("2e9e9f71-7c70-4a07-a149-a15dd30fc1b8"); }
		}
	}
}
