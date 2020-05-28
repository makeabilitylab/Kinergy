using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace MotionSimulator
{
    public class MotionSimulatorInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "MotionSimulator";
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
                return "This cell take established motion instance and conduct kinetic simulation.";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("48a7cff3-f746-43b3-a0a8-328e73a544ba");
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
