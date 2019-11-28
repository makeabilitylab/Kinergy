using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.IO;
using System.Drawing;

namespace EnergyPlugin
{
    partial class EnergyPluginControl
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;


        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code
        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.ExportBtn = new System.Windows.Forms.Button();
            this.AddShapeBtn = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // ExportBtn
            // 
            this.ExportBtn.Location = new System.Drawing.Point(152, 173);
            this.ExportBtn.Name = "ExportBtn";
            this.ExportBtn.Size = new System.Drawing.Size(195, 59);
            this.ExportBtn.TabIndex = 0;
            this.ExportBtn.Text = "Export as STL";
            this.ExportBtn.UseVisualStyleBackColor = true;
            this.ExportBtn.Click += new System.EventHandler(this.ExportBtn_Click);
            // 
            // AddShapeBtn
            // 
            this.AddShapeBtn.Location = new System.Drawing.Point(152, 89);
            this.AddShapeBtn.Name = "AddShapeBtn";
            this.AddShapeBtn.Size = new System.Drawing.Size(195, 58);
            this.AddShapeBtn.TabIndex = 1;
            this.AddShapeBtn.Text = "Add a shape...";
            this.AddShapeBtn.UseVisualStyleBackColor = true;
            this.AddShapeBtn.Click += new System.EventHandler(this.AddShapeBtn_Click);
            // 
            // WearablePluginControl
            // 
            this.Controls.Add(this.AddShapeBtn);
            this.Controls.Add(this.ExportBtn);
            this.Name = "WearablePluginControl";
            this.Size = new System.Drawing.Size(500, 812);
            this.ResumeLayout(false);

        }
        #endregion

        private Button ExportBtn;
        private Button AddShapeBtn;
    }
}
