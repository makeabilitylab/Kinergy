using System;
using System.Drawing;
using System.Windows.Forms;

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

        private void VoxelizeBtn_Click(object sender, EventArgs e) {
            controller.Voxelize();
        }
    }
}
