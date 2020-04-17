using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace ContinuousLinearMovingByPress
{
    public class ContinuousLinearMovingByPressInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "ContinuousLinearMovingByPress";
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
                return new Guid("7163e5d4-9098-4b9c-86b6-b08440417278");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "Xia Su";
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
