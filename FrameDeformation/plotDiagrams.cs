using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace FrameDeformation
{
	public class PlotDiagrams : GH_Component
	{

		public PlotDiagrams()
		  : base("FrameDeformation", "FrameDeformation",
			  "Plots the provided values at each beam",
			  "Frame", "Post Processing")
		{
		}

		protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
		{
			pManager.AddNumberParameter("Values", "Values", "Values", GH_ParamAccess.tree);
			pManager.AddLineParameter("Line", "Line", "Lines representing the beams", GH_ParamAccess.list);
			pManager.AddNumberParameter("Scale Factor", "Scale Factor", "Scale factor for the diagram", GH_ParamAccess.item);
		}

		protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
		{
			pManager.AddCurveParameter("Diagram", "Diagram", "Curve representing the diagram", GH_ParamAccess.list);
		}

		protected override void SolveInstance(IGH_DataAccess DA)
		{
			List<Line> lines = new List<Line>();
			double scaleFactor = 0.0;

			DA.GetDataTree("Values", out GH_Structure<GH_Number> values);
			DA.GetDataList("Line", lines);
			DA.GetData("Scale Factor", ref scaleFactor);

			// The length of A and E must be the same as lines
			if ((lines.Count != values.Branches.Count))
			{
				throw new ArgumentException("The number of brances must be equal to the length of Line");
			}

			int nrValues = values[0].Count;

			List<Polyline> diagrams = new List<Polyline>();

			// If scale factor not provided normalize so that maximum value corresponds to half beam length
			double maxValue = 0.0;
			int imax = 0;
			if (scaleFactor == 0.0)
			{
				for (int i = 0; i < lines.Count; i++)
				{
					for (int j = 0; j < nrValues; j++)
					{
						if ((double)values[i][j].Value >= maxValue)
						{
							maxValue = (double)values[i][j].Value;
							imax = i;
						}
					}
				}
				scaleFactor = lines[imax].Length / maxValue / 2;
			}

			for (int i = 0; i < lines.Count; i++)
			{


				// Calculate normal vector
				Vector3d tangentVector = new Vector3d(lines[i].To - lines[i].From);
				tangentVector.Unitize();
				Vector3d normalVector = Vector3d.CrossProduct(tangentVector, new Vector3d(0, 0, 1));

				List<Point3d> points = new List<Point3d>();

				for (int j = 0; j < nrValues; j++)
				{
					Point3d pi = lines[i].PointAt((double)j / (nrValues - 1));

					if (j == 0)
						points.Add(lines[i].PointAt((double)j / (nrValues -1)));

					//Translate point in normal direction
					double scaledValue = (double)values[i][j].Value * scaleFactor;
					pi.Transform(Transform.Translation(Vector3d.Multiply(normalVector, scaledValue)));
					points.Add(pi);

					if (j == nrValues - 1)
						points.Add(lines[i].PointAt((double)j / (nrValues - 1)));

				}

				// Draw a polyline
				Polyline diagram = new Polyline();
				diagram.AddRange(points);
				diagrams.Add(diagram);

				
			}

			DA.SetDataList("Diagram", diagrams);

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