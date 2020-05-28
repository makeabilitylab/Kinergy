using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace InstExtension
{
    public class InstantExtensionInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "InstantExtension";
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
                return new Guid("35c6cf1b-1fe6-47af-b1b7-88b2d4f45445");
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
                return "su_xia1997@foxmail.com";
            }
        }
    }
}
