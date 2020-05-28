using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace Simulation
{
    public class SimulationInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "Simulation";
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
                return new Guid("22fe14be-52e9-49c0-b1fd-4b2514072b2b");
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
