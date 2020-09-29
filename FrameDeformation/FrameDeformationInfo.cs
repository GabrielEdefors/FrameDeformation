using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace FrameDeformation
{
	public class FrameDeformationInfo : GH_AssemblyInfo
	{
		public override string Name
		{
			get
			{
				return "FrameDeformation";
			}
		}
		public override Bitmap Icon
		{
			get
			{
				//Return a 24x24 pixel bitmap to represent this GHA library.
				return null;
			}
		}
		public override string Description
		{
			get
			{
				//Return a short string describing the purpose of this GHA library.
				return "";
			}
		}
		public override Guid Id
		{
			get
			{
				return new Guid("7e6ea7bf-39dc-481d-b97b-5854ed48f947");
			}
		}

		public override string AuthorName
		{
			get
			{
				//Return a string identifying you or your company.
				return "";
			}
		}
		public override string AuthorContact
		{
			get
			{
				//Return a string representing your preferred contact details.
				return "";
			}
		}
	}
}
