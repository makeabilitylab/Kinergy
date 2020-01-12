using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Timers;
using Rhino.DocObjects;

namespace EnergyPlugin
{
    public interface IControllerModelObserver
    {

    }


    public interface Controller
    {
        #region default functions
        void AddRandomGeometry();
        void ExportModel();
        void Voxelize();
        #endregion

    }

    public class IncController : Controller
    {
        RhinoModel rhinoModel;
        View view;

        public IncController(View v,  RhinoModel rm)
        {
            view = v;
            rhinoModel = rm;
            view.setController(this);
        }

        public void AddRandomGeometry()
        {
            rhinoModel.AddRandomGeometry();
        }

        public void ExportModel()
        {
            rhinoModel.ExportModel();
        }
        public void Voxelize()
        {
            rhinoModel.Voxelize();
        }
    }
}
