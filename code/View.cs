using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace EnergyPlugin
{
    /// <summary>
    /// Rhino plugin required class: set up the controller
    /// </summary>
    public interface View
    {
        void setController(Controller cont);
    }
}
