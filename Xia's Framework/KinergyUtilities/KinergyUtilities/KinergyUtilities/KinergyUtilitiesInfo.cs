using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace KinergyUtilities
{
    public class KinergyUtilitiesInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "KinergyUtilities";
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
                return new Guid("64a5b011-54a1-4ef2-922b-bc4839332a22");
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
