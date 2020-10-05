using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace FrameDeformation
{
	public class HingeComponent : GH_Component
	{
		public HingeComponent()
		  : base("HingeNode", "HingeNode",
			  "Creates a node object that adds a hinge to a node",
			  "Frame", "Elements")
		{
		}

		protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
		{
			pManager.AddPointParameter("Point", "point", "The spatial representation of the node", GH_ParamAccess.list);
		}

		protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
		{
			pManager.AddGenericParameter("hingeNode", "hingeNode", "Node", GH_ParamAccess.list);
		}

		protected override void SolveInstance(IGH_DataAccess DA)
		{
			List<Point3d> points = new List<Point3d>();

			DA.GetDataList("Point", points);

			// Create a hinge node object and store it in a list
			List<HingeNode> nodes = new List<HingeNode>();

			for (int i = 0; i < points.Count; i++)
			{
				nodes.Add(new HingeNode(points[i]));
			}

			DA.SetDataList("hingeNode", nodes);

		}

		public override Guid ComponentGuid
		{
			get { return new Guid("f40ed867-e6f7-40cd-9288-ab62859e2865"); }
		}
	}
}