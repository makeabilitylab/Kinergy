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
            this.button1 = new System.Windows.Forms.Button();
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
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(152, 274);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(195, 51);
            this.button1.TabIndex = 2;
            this.button1.Text = "Voxelize a model";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click_1);
            // 
            // EnergyPluginControl
            // 
            this.Controls.Add(this.button1);
            this.Controls.Add(this.AddShapeBtn);
            this.Controls.Add(this.ExportBtn);
            this.Name = "EnergyPluginControl";
            this.Size = new System.Drawing.Size(500, 812);
            this.Load += new System.EventHandler(this.EnergyPluginControl_Load);
            this.ResumeLayout(false);

        }
        #endregion

        private Button ExportBtn;
        private Button AddShapeBtn;
        private Button button1;
    }
}
