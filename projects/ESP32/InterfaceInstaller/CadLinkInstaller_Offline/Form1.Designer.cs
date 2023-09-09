
namespace CadLinkHL7InterfaceSetup
{
    partial class formMain
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

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(formMain));
            this.rtbOutputWindow = new System.Windows.Forms.RichTextBox();
            this.btnInstall = new System.Windows.Forms.Button();
            this.btnDownload = new System.Windows.Forms.Button();
            this.BtnClrSelections = new System.Windows.Forms.Button();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.RdoCssSp2 = new System.Windows.Forms.RadioButton();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.RdoArcSp2 = new System.Windows.Forms.RadioButton();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.RdoSmt315 = new System.Windows.Forms.RadioButton();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.RdoCln5319 = new System.Windows.Forms.RadioButton();
            this.RdoThn5319 = new System.Windows.Forms.RadioButton();
            this.RdoCln5222 = new System.Windows.Forms.RadioButton();
            this.RdoThn5222 = new System.Windows.Forms.RadioButton();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.RdoHL7 = new System.Windows.Forms.RadioButton();
            this.RdoSrv5319 = new System.Windows.Forms.RadioButton();
            this.RdoSrv5222 = new System.Windows.Forms.RadioButton();
            this.PnlRdoSelections = new System.Windows.Forms.Panel();
            this.pBarDownload = new System.Windows.Forms.ProgressBar();
            this.lblDownloadProgress = new System.Windows.Forms.Label();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.btnExportLog = new System.Windows.Forms.Button();
            this.groupBox4.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.PnlRdoSelections.SuspendLayout();
            this.groupBox6.SuspendLayout();
            this.SuspendLayout();
            // 
            // rtbOutputWindow
            // 
            this.rtbOutputWindow.Location = new System.Drawing.Point(218, 67);
            this.rtbOutputWindow.Name = "rtbOutputWindow";
            this.rtbOutputWindow.ReadOnly = true;
            this.rtbOutputWindow.ScrollBars = System.Windows.Forms.RichTextBoxScrollBars.Vertical;
            this.rtbOutputWindow.Size = new System.Drawing.Size(406, 237);
            this.rtbOutputWindow.TabIndex = 12;
            this.rtbOutputWindow.Text = "Please select your software from the left and click install.\n\nTo perform a manual" +
    " installation, please click the Download Software button.\n";
            // 
            // btnInstall
            // 
            this.btnInstall.BackColor = System.Drawing.Color.Lime;
            this.btnInstall.Location = new System.Drawing.Point(424, 6);
            this.btnInstall.Name = "btnInstall";
            this.btnInstall.Size = new System.Drawing.Size(200, 23);
            this.btnInstall.TabIndex = 13;
            this.btnInstall.Text = "Install Selected";
            this.btnInstall.UseVisualStyleBackColor = false;
            this.btnInstall.Click += new System.EventHandler(this.BtnInstall_Click);
            // 
            // btnDownload
            // 
            this.btnDownload.BackColor = System.Drawing.Color.DarkOrange;
            this.btnDownload.Location = new System.Drawing.Point(218, 36);
            this.btnDownload.Name = "btnDownload";
            this.btnDownload.Size = new System.Drawing.Size(200, 23);
            this.btnDownload.TabIndex = 14;
            this.btnDownload.Text = "Download Software";
            this.btnDownload.UseVisualStyleBackColor = false;
            this.btnDownload.Click += new System.EventHandler(this.BtnDownload_Click);
            // 
            // BtnClrSelections
            // 
            this.BtnClrSelections.BackColor = System.Drawing.Color.Red;
            this.BtnClrSelections.Location = new System.Drawing.Point(218, 7);
            this.BtnClrSelections.Name = "BtnClrSelections";
            this.BtnClrSelections.Size = new System.Drawing.Size(200, 23);
            this.BtnClrSelections.TabIndex = 32;
            this.BtnClrSelections.Text = "Clear Selections";
            this.BtnClrSelections.UseVisualStyleBackColor = false;
            this.BtnClrSelections.Click += new System.EventHandler(this.BtnClrSelections_Click);
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.RdoCssSp2);
            this.groupBox4.Location = new System.Drawing.Point(3, 265);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(200, 43);
            this.groupBox4.TabIndex = 30;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Cascade Surgical Studio";
            // 
            // RdoCssSp2
            // 
            this.RdoCssSp2.AutoSize = true;
            this.RdoCssSp2.Location = new System.Drawing.Point(7, 19);
            this.RdoCssSp2.Name = "RdoCssSp2";
            this.RdoCssSp2.Size = new System.Drawing.Size(62, 17);
            this.RdoCssSp2.TabIndex = 24;
            this.RdoCssSp2.TabStop = true;
            this.RdoCssSp2.Text = "3.5 Sp2";
            this.RdoCssSp2.UseVisualStyleBackColor = true;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.RdoArcSp2);
            this.groupBox3.Location = new System.Drawing.Point(3, 215);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(200, 44);
            this.groupBox3.TabIndex = 29;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Arc";
            // 
            // RdoArcSp2
            // 
            this.RdoArcSp2.AutoSize = true;
            this.RdoArcSp2.Location = new System.Drawing.Point(7, 19);
            this.RdoArcSp2.Name = "RdoArcSp2";
            this.RdoArcSp2.Size = new System.Drawing.Size(63, 17);
            this.RdoArcSp2.TabIndex = 23;
            this.RdoArcSp2.TabStop = true;
            this.RdoArcSp2.Text = "3.0 SP2";
            this.RdoArcSp2.UseVisualStyleBackColor = true;
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.RdoSmt315);
            this.groupBox5.Location = new System.Drawing.Point(3, 314);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(200, 39);
            this.groupBox5.TabIndex = 31;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "Sierra Summit";
            // 
            // RdoSmt315
            // 
            this.RdoSmt315.AutoSize = true;
            this.RdoSmt315.Location = new System.Drawing.Point(13, 19);
            this.RdoSmt315.Name = "RdoSmt315";
            this.RdoSmt315.Size = new System.Drawing.Size(61, 17);
            this.RdoSmt315.TabIndex = 25;
            this.RdoSmt315.TabStop = true;
            this.RdoSmt315.Text = "3.1.537";
            this.RdoSmt315.UseVisualStyleBackColor = true;
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.RdoCln5319);
            this.groupBox2.Controls.Add(this.RdoThn5319);
            this.groupBox2.Controls.Add(this.RdoCln5222);
            this.groupBox2.Controls.Add(this.RdoThn5222);
            this.groupBox2.Location = new System.Drawing.Point(3, 107);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(200, 106);
            this.groupBox2.TabIndex = 28;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "CadLink Client";
            // 
            // RdoCln5319
            // 
            this.RdoCln5319.AutoSize = true;
            this.RdoCln5319.Location = new System.Drawing.Point(13, 19);
            this.RdoCln5319.Name = "RdoCln5319";
            this.RdoCln5319.Size = new System.Drawing.Size(55, 17);
            this.RdoCln5319.TabIndex = 17;
            this.RdoCln5319.Text = "5.3.19";
            this.RdoCln5319.UseVisualStyleBackColor = true;
            // 
            // RdoThn5319
            // 
            this.RdoThn5319.AutoSize = true;
            this.RdoThn5319.Location = new System.Drawing.Point(13, 42);
            this.RdoThn5319.Name = "RdoThn5319";
            this.RdoThn5319.Size = new System.Drawing.Size(156, 17);
            this.RdoThn5319.TabIndex = 20;
            this.RdoThn5319.TabStop = true;
            this.RdoThn5319.Text = "5.3.19 - CadLink Thin Client";
            this.RdoThn5319.UseVisualStyleBackColor = true;
            // 
            // RdoCln5222
            // 
            this.RdoCln5222.AutoSize = true;
            this.RdoCln5222.Location = new System.Drawing.Point(13, 65);
            this.RdoCln5222.Name = "RdoCln5222";
            this.RdoCln5222.Size = new System.Drawing.Size(55, 17);
            this.RdoCln5222.TabIndex = 21;
            this.RdoCln5222.TabStop = true;
            this.RdoCln5222.Text = "5.2.22";
            this.RdoCln5222.UseVisualStyleBackColor = true;
            // 
            // RdoThn5222
            // 
            this.RdoThn5222.AutoSize = true;
            this.RdoThn5222.Location = new System.Drawing.Point(13, 85);
            this.RdoThn5222.Name = "RdoThn5222";
            this.RdoThn5222.Size = new System.Drawing.Size(156, 17);
            this.RdoThn5222.TabIndex = 22;
            this.RdoThn5222.TabStop = true;
            this.RdoThn5222.Text = "5.2.22 - CadLink Thin Client";
            this.RdoThn5222.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.RdoSrv5319);
            this.groupBox1.Controls.Add(this.RdoSrv5222);
            this.groupBox1.Location = new System.Drawing.Point(3, 40);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(200, 61);
            this.groupBox1.TabIndex = 27;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "CadLink Server";
            // 
            // RdoHL7
            // 
            this.RdoHL7.AutoSize = true;
            this.RdoHL7.Location = new System.Drawing.Point(13, 14);
            this.RdoHL7.Name = "RdoHL7";
            this.RdoHL7.Size = new System.Drawing.Size(90, 17);
            this.RdoHL7.TabIndex = 17;
            this.RdoHL7.TabStop = true;
            this.RdoHL7.Text = "HL7 Interface";
            this.RdoHL7.UseVisualStyleBackColor = true;
            // 
            // RdoSrv5319
            // 
            this.RdoSrv5319.AutoSize = true;
            this.RdoSrv5319.Location = new System.Drawing.Point(13, 19);
            this.RdoSrv5319.Name = "RdoSrv5319";
            this.RdoSrv5319.Size = new System.Drawing.Size(55, 17);
            this.RdoSrv5319.TabIndex = 15;
            this.RdoSrv5319.Text = "5.3.19";
            this.RdoSrv5319.UseVisualStyleBackColor = true;
            // 
            // RdoSrv5222
            // 
            this.RdoSrv5222.AutoSize = true;
            this.RdoSrv5222.Location = new System.Drawing.Point(13, 42);
            this.RdoSrv5222.Name = "RdoSrv5222";
            this.RdoSrv5222.Size = new System.Drawing.Size(55, 17);
            this.RdoSrv5222.TabIndex = 16;
            this.RdoSrv5222.Text = "5.2.22";
            this.RdoSrv5222.UseVisualStyleBackColor = true;
            // 
            // PnlRdoSelections
            // 
            this.PnlRdoSelections.Controls.Add(this.groupBox6);
            this.PnlRdoSelections.Controls.Add(this.groupBox1);
            this.PnlRdoSelections.Controls.Add(this.groupBox2);
            this.PnlRdoSelections.Controls.Add(this.groupBox5);
            this.PnlRdoSelections.Controls.Add(this.groupBox3);
            this.PnlRdoSelections.Controls.Add(this.groupBox4);
            this.PnlRdoSelections.Location = new System.Drawing.Point(5, 3);
            this.PnlRdoSelections.Name = "PnlRdoSelections";
            this.PnlRdoSelections.Size = new System.Drawing.Size(207, 353);
            this.PnlRdoSelections.TabIndex = 33;
            // 
            // pBarDownload
            // 
            this.pBarDownload.Location = new System.Drawing.Point(218, 336);
            this.pBarDownload.Name = "pBarDownload";
            this.pBarDownload.Size = new System.Drawing.Size(406, 20);
            this.pBarDownload.Step = 1;
            this.pBarDownload.TabIndex = 34;
            // 
            // lblDownloadProgress
            // 
            this.lblDownloadProgress.AutoSize = true;
            this.lblDownloadProgress.Location = new System.Drawing.Point(215, 317);
            this.lblDownloadProgress.Name = "lblDownloadProgress";
            this.lblDownloadProgress.Size = new System.Drawing.Size(102, 13);
            this.lblDownloadProgress.TabIndex = 35;
            this.lblDownloadProgress.Text = "Download Progress:";
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.RdoHL7);
            this.groupBox6.Location = new System.Drawing.Point(3, 3);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Size = new System.Drawing.Size(200, 31);
            this.groupBox6.TabIndex = 36;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "CadLink HL7 Interface";
            // 
            // btnExportLog
            // 
            this.btnExportLog.BackColor = System.Drawing.Color.DarkOrange;
            this.btnExportLog.Location = new System.Drawing.Point(425, 36);
            this.btnExportLog.Name = "btnExportLog";
            this.btnExportLog.Size = new System.Drawing.Size(199, 23);
            this.btnExportLog.TabIndex = 36;
            this.btnExportLog.Text = "Export Log";
            this.btnExportLog.UseVisualStyleBackColor = false;
            this.btnExportLog.Click += new System.EventHandler(this.btnExportLog_Click);
            // 
            // formMain
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(631, 362);
            this.Controls.Add(this.btnExportLog);
            this.Controls.Add(this.lblDownloadProgress);
            this.Controls.Add(this.pBarDownload);
            this.Controls.Add(this.PnlRdoSelections);
            this.Controls.Add(this.BtnClrSelections);
            this.Controls.Add(this.btnDownload);
            this.Controls.Add(this.btnInstall);
            this.Controls.Add(this.rtbOutputWindow);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "formMain";
            this.Text = "CadLink HL7 Interface Setup";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox5.ResumeLayout(false);
            this.groupBox5.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.PnlRdoSelections.ResumeLayout(false);
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion
        private System.Windows.Forms.RichTextBox rtbOutputWindow;
        private System.Windows.Forms.Button btnInstall;
        private System.Windows.Forms.Button btnDownload;
        private System.Windows.Forms.Button BtnClrSelections;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.RadioButton RdoCssSp2;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.RadioButton RdoArcSp2;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.RadioButton RdoSmt315;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.RadioButton RdoCln5319;
        private System.Windows.Forms.RadioButton RdoThn5319;
        private System.Windows.Forms.RadioButton RdoCln5222;
        private System.Windows.Forms.RadioButton RdoThn5222;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.RadioButton RdoSrv5319;
        private System.Windows.Forms.RadioButton RdoSrv5222;
        private System.Windows.Forms.Panel PnlRdoSelections;
        private System.Windows.Forms.RadioButton RdoHL7;
        private System.Windows.Forms.ProgressBar pBarDownload;
        private System.Windows.Forms.Label lblDownloadProgress;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.Button btnExportLog;
    }
}

