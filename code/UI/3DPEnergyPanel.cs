using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using Rhino.UI;
using System.Xml;
using Rhino.DocObjects;
using Rhino.Geometry;
using System.Drawing.Drawing2D;
using System.IO;
using Rhino;
using Rhino.Commands;
using Rhino.Input;
using Rhino.Geometry;
using Rhino.DocObjects;

namespace EnergyPlugin
{
    public partial class EnergyPluginControl : MetroFramework.Controls.MetroUserControl, View,  IControllerModelObserver
    {

        #region Initialization & Construction
        Controller controller;
        public void setController(Controller cont)
        {
            controller = cont;
            
            if (EventWatcherHandlers.Instance.IsEnabled == false)
            {
                EventWatcherHandlers.Instance.Enable(true);
                EventWatcherHandlers.Instance.setRhinoModel(ref controller);
            }
        }


        public EnergyPluginControl()
        {

            InitializeComponent();
            SetStyle(ControlStyles.SupportsTransparentBackColor, true);
            this.BackColor = Color.FromArgb(255, 255, 255, 255);

        }
        #endregion

        private void ExportBtn_Click(object sender, EventArgs e)
        {
            controller.ExportModel();
        }

        private void AddShapeBtn_Click(object sender, EventArgs e)
        {
            controller.AddRandomGeometry();
        }
    }
}
