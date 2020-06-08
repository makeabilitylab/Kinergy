using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace MS_Throwing
{
    public class MS_ThrowingInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "MSThrowing";
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
                return new Guid("9cf57984-29e5-45ae-8e11-68db595dfe05");
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
