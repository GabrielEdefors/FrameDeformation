using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace FrameDeformation
{
	public class plotDiagrams : GH_Component
	{

		public plotDiagrams()
		  : base("FrameDeformation", "FrameDeformation",
			  "Plots the provided values at each beam",
			  "Frame", "Post Processing")
		{
		}

		protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
		{
			pManager.AddGenericParameter("Values", "Values", "Values", GH_ParamAccess.item);
			pManager.AddCurveParameter("Line", "Line", "Lines representing the beams", GH_ParamAccess.list);
		}

		protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
		{
			pManager.AddCurveParameter("Diagram", "Diagram", "Curve representing the diagram", GH_ParamAccess.list);
		}

		protected override void SolveInstance(IGH_DataAccess DA)
		{
			List<Line> lines = new List<Line>();

			DA.GetDataTree("Values", out GH_Structure<IGH_Goo> values);
			DA.GetDataList("Line", lines);

			// The length of A and E must be the same as lines
			if ((lines.Count != values.Branches.Count))
			{
				throw new ArgumentException("The number of brances must be equal to the length of Line");
			}

			// 
			for (int i = 0; i < lines.Count; i++)
			{

			}

		}

		/// <summary>
		/// Provides an Icon for the component.
		/// </summary>
		protected override System.Drawing.Bitmap Icon
		{
			get
			{
				//You can add image files to your project resources and access them like this:
				// return Resources.IconForThisComponent;
				return null;
			}
		}

		/// <summary>
		/// Gets the unique ID for this component. Do not change this ID after release.
		/// </summary>
		public override Guid ComponentGuid
		{
			get { return new Guid("7b851093-947b-4039-87d0-6176936e933a"); }
		}
	}
}