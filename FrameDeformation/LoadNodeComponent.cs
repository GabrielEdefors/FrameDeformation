using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace FrameDeformation
{
	public class LoadNodeComponent : GH_Component
	{
		public LoadNodeComponent()
		  : base("LoadNode", "LoadNode",
			  "Creates a node object that adds loads to a node",
			  "Frame", "Elements")
		{
		}

		protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
		{
			pManager.AddPointParameter("Point", "point", "The spatial representation of the node", GH_ParamAccess.list);
			pManager.AddNumberParameter("Force in x", "Force in x", "Force in x direction", GH_ParamAccess.list);
			pManager.AddNumberParameter("Force in y", "Force in y", "Force in y direction", GH_ParamAccess.list);
			pManager.AddNumberParameter("Moment in r", "Moment in r", "Moment in rotational degree of freedom", GH_ParamAccess.list);
		}

		protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
		{
			pManager.AddGenericParameter("loadNode", "loadNode", "Node", GH_ParamAccess.list);
		}

		protected override void SolveInstance(IGH_DataAccess DA)
		{
			List<Point3d> points = new List<Point3d>();
			List<double> forceX = new List<double>();
			List<double> forceY = new List<double>();
			List<double> momentR = new List<double>();

			DA.GetDataList("Point", points);
			DA.GetDataList("Force in x", forceX);
			DA.GetDataList("Force in y", forceY);
			DA.GetDataList("Moment in r", momentR);

			// Create a restraint node object and store it in a list
			List<LoadNode> nodes = new List<LoadNode>();

			for (int i = 0; i < points.Count; i++)
			{
				nodes.Add(new LoadNode(points[i], forceX[i], forceY[i], momentR[i]));
			}

			DA.SetDataList("loadNode", nodes);

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
			get { return new Guid("06c62fa7-be8b-49c4-acd1-3f37ae6169a3"); }
		}
	}
}