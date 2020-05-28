using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace MotionSolver
{
    public class MotionSolverInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "MotionSolver";
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
                return "Solve motion with given base model and other parameter";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("38b186d2-b6db-4825-8853-b84ceebb1ae8");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "XiaSu";
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
