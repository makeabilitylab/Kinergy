using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace ContinuousLinearMovingPress
{
    public class ContinuousLinearMovingPressInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "ContinuousLinearMovingPress";
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
                return new Guid("028fe3a5-2d6e-475e-a53a-c534de92265e");
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
