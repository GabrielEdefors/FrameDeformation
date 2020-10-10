using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using Rhino.Display;

namespace FrameDeformation
{
	public class PlotDiagrams : GH_Component
	{
		private List<Text3d> startValueText = new List<Text3d>();
		private List<Text3d> endValueText = new List<Text3d>();
		private List<Text3d> maxValueText = new List<Text3d>();
		List<Polyline> diagrams = new List<Polyline>();

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
			//pManager.AddCurveParameter("Diagram", "Diagram", "Curve representing the diagram", GH_ParamAccess.list);
		}

		protected override void SolveInstance(IGH_DataAccess DA)
		{
			startValueText.Clear();
			endValueText.Clear();
			maxValueText.Clear();
			diagrams.Clear();

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
				scaleFactor = lines[imax].Length / maxValue / 4;
			}

			for (int i = 0; i < lines.Count; i++)
			{


				// Calculate normal vector
				Vector3d tangentVector = new Vector3d(lines[i].To - lines[i].From);
				tangentVector.Unitize();
				Vector3d normalVector = Vector3d.CrossProduct(tangentVector, new Vector3d(0, 0, 1));

				List<Point3d> points = new List<Point3d>();
				double maxBeamValue = 0.0;
				Point3d maxBeamPoint = new Point3d(0, 0, 0);

				for (int j = 0; j < nrValues; j++)
				{
					Point3d pi = lines[i].PointAt((double)j / (nrValues - 1));

					if (j == 0)
						points.Add(lines[i].PointAt((double)j / (nrValues - 1)));

					//Translate point in normal direction
					double scaledValue = (double)values[i][j].Value * scaleFactor;
					pi.Transform(Transform.Translation(Vector3d.Multiply(normalVector, scaledValue)));
					points.Add(pi);

					if (Math.Abs((double)values[i][j].Value) > maxBeamValue)
					{
						maxBeamValue = Math.Abs((double)values[i][j].Value);
						maxBeamPoint = pi;
					}

					if (j == nrValues - 1)
					{
						points.Add(lines[i].PointAt((double)j / (nrValues - 1)));

						// Add text for end value
						endValueText.Add(new Text3d(Math.Round((double)values[i][j].Value, 1).ToString(), new Plane(pi, new Vector3d(0, 0, 1)), lines[imax].Length / 40));
					}

					// Add text for start value
					if (j == 0)
					{
						startValueText.Add(new Text3d(Math.Round((double)values[i][j].Value, 1).ToString(), new Plane(pi, new Vector3d(0, 0, 1)), lines[imax].Length / 40));
					}

					// Add lines between the diagram and the line for every eight point
					if (j % 8 == 0)
						diagrams.Add(new Polyline(new Point3d[] { pi, lines[i].PointAt((double)j / (nrValues - 1)) }));
				}

				// Add text for max value
				maxValueText.Add(new Text3d(Math.Round(maxBeamValue, 1).ToString(), new Plane(maxBeamPoint, new Vector3d(0, 0, 1)), lines[imax].Length / 40));

				// Draw a polyline
				Polyline diagram = new Polyline();
				diagram.AddRange(points);
				diagrams.Add(diagram);

			}

			//DA.SetDataList("Diagram", diagrams);
			//DA.SetDataList("Start Value", startValueTexts);
			//DA.SetDataList("End Value", endValueTexts);
			//DA.SetDataList("Max Value", maxValueTexts);

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
			get { return new Guid("7b851093-947b-4039-87d0-6176936e933a"); }
		}

		public override void DrawViewportWires(IGH_PreviewArgs args)
		{

			for (int j = 0; j < startValueText.Count; j++)
			{
				args.Display.Draw3dText(startValueText[j], System.Drawing.Color.Red);
				startValueText[j].Dispose();

				args.Display.Draw3dText(endValueText[j], System.Drawing.Color.Red);
				endValueText[j].Dispose();

				if (maxValueText[j].TextPlane != startValueText[j].TextPlane & maxValueText[j].TextPlane != endValueText[j].TextPlane)
				{
					args.Display.Draw3dText(maxValueText[j], System.Drawing.Color.Red);
					maxValueText[j].Dispose();
				}
			}

			foreach (Polyline line in diagrams)
			{
				args.Display.DrawPolyline(line, System.Drawing.Color.Blue);
			}

		}
	}
}