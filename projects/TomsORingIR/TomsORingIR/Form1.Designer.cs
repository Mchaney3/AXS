namespace TomsORingIR
{
    partial class Form1
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
            this.components = new System.ComponentModel.Container();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.buttonLED_BUILTIN = new System.Windows.Forms.Button();
            this.panel1 = new System.Windows.Forms.Panel();
            this.counterCurrentCount = new System.Windows.Forms.Label();
            this.panel2 = new System.Windows.Forms.Panel();
            this.counterLastCount = new System.Windows.Forms.Label();
            this.buttonConnect = new System.Windows.Forms.Button();
            this.buttonResetCount = new System.Windows.Forms.Button();
            this.panel3 = new System.Windows.Forms.Panel();
            this.boxComPort = new System.Windows.Forms.ComboBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.ComPort = new System.IO.Ports.SerialPort(this.components);
            this.panel1.SuspendLayout();
            this.panel2.SuspendLayout();
            this.panel3.SuspendLayout();
            this.SuspendLayout();
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(12, 113);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(171, 20);
            this.label1.TabIndex = 0;
            this.label1.Text = "Total O-Rings Counted";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(189, 113);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(186, 20);
            this.label2.TabIndex = 1;
            this.label2.Text = "Last Count Before Reset";
            // 
            // buttonLED_BUILTIN
            // 
            this.buttonLED_BUILTIN.Location = new System.Drawing.Point(7, 40);
            this.buttonLED_BUILTIN.Name = "buttonLED_BUILTIN";
            this.buttonLED_BUILTIN.Size = new System.Drawing.Size(75, 23);
            this.buttonLED_BUILTIN.TabIndex = 2;
            this.buttonLED_BUILTIN.Text = "LED: OFF";
            this.buttonLED_BUILTIN.UseVisualStyleBackColor = true;
            this.buttonLED_BUILTIN.Click += new System.EventHandler(this.buttonLED_BUILTIN_Click);
            // 
            // panel1
            // 
            this.panel1.BackColor = System.Drawing.Color.White;
            this.panel1.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.panel1.Controls.Add(this.counterCurrentCount);
            this.panel1.Location = new System.Drawing.Point(22, 136);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(148, 156);
            this.panel1.TabIndex = 3;
            // 
            // counterCurrentCount
            // 
            this.counterCurrentCount.AutoSize = true;
            this.counterCurrentCount.Font = new System.Drawing.Font("Microsoft Sans Serif", 48F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.counterCurrentCount.Location = new System.Drawing.Point(41, 39);
            this.counterCurrentCount.Name = "counterCurrentCount";
            this.counterCurrentCount.RightToLeft = System.Windows.Forms.RightToLeft.No;
            this.counterCurrentCount.Size = new System.Drawing.Size(68, 73);
            this.counterCurrentCount.TabIndex = 0;
            this.counterCurrentCount.Text = "0";
            // 
            // panel2
            // 
            this.panel2.BackColor = System.Drawing.Color.White;
            this.panel2.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.panel2.Controls.Add(this.counterLastCount);
            this.panel2.Location = new System.Drawing.Point(207, 136);
            this.panel2.Name = "panel2";
            this.panel2.Size = new System.Drawing.Size(148, 156);
            this.panel2.TabIndex = 4;
            // 
            // counterLastCount
            // 
            this.counterLastCount.AutoSize = true;
            this.counterLastCount.Font = new System.Drawing.Font("Microsoft Sans Serif", 48F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.counterLastCount.Location = new System.Drawing.Point(42, 39);
            this.counterLastCount.Name = "counterLastCount";
            this.counterLastCount.Size = new System.Drawing.Size(68, 73);
            this.counterLastCount.TabIndex = 1;
            this.counterLastCount.Text = "0";
            // 
            // buttonConnect
            // 
            this.buttonConnect.Location = new System.Drawing.Point(238, 27);
            this.buttonConnect.Name = "buttonConnect";
            this.buttonConnect.Size = new System.Drawing.Size(75, 23);
            this.buttonConnect.TabIndex = 5;
            this.buttonConnect.Text = "Connect";
            this.buttonConnect.UseVisualStyleBackColor = true;
            this.buttonConnect.Click += new System.EventHandler(this.buttonConnect_Click);
            // 
            // buttonResetCount
            // 
            this.buttonResetCount.Location = new System.Drawing.Point(7, 69);
            this.buttonResetCount.Name = "buttonResetCount";
            this.buttonResetCount.Size = new System.Drawing.Size(75, 23);
            this.buttonResetCount.TabIndex = 6;
            this.buttonResetCount.Text = "Reset Count";
            this.buttonResetCount.UseVisualStyleBackColor = true;
            this.buttonResetCount.Click += new System.EventHandler(this.buttonResetCount_Click);
            // 
            // panel3
            // 
            this.panel3.BackColor = System.Drawing.SystemColors.Control;
            this.panel3.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.panel3.Controls.Add(this.boxComPort);
            this.panel3.Controls.Add(this.label4);
            this.panel3.Controls.Add(this.label3);
            this.panel3.Controls.Add(this.buttonResetCount);
            this.panel3.Controls.Add(this.buttonConnect);
            this.panel3.Controls.Add(this.buttonLED_BUILTIN);
            this.panel3.Location = new System.Drawing.Point(12, 12);
            this.panel3.Name = "panel3";
            this.panel3.Size = new System.Drawing.Size(363, 98);
            this.panel3.TabIndex = 7;
            // 
            // boxComPort
            // 
            this.boxComPort.FormattingEnabled = true;
            this.boxComPort.Location = new System.Drawing.Point(237, 3);
            this.boxComPort.Name = "boxComPort";
            this.boxComPort.Size = new System.Drawing.Size(121, 21);
            this.boxComPort.TabIndex = 10;
            this.boxComPort.SelectedIndexChanged += new System.EventHandler(this.comboBox1_SelectedIndexChanged);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(146, 8);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(86, 13);
            this.label4.TabIndex = 9;
            this.label4.Text = "Select COM Port";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.Location = new System.Drawing.Point(3, 4);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(104, 20);
            this.label3.TabIndex = 7;
            this.label3.Text = "Control Panel";
            // 
            // ComPort
            // 
            this.ComPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.ComPort_DataReceived);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(384, 304);
            this.Controls.Add(this.panel2);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.panel3);
            this.Name = "Form1";
            this.Text = "O Ring Counter";
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            this.panel2.ResumeLayout(false);
            this.panel2.PerformLayout();
            this.panel3.ResumeLayout(false);
            this.panel3.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button buttonLED_BUILTIN;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Panel panel2;
        private System.Windows.Forms.Label counterLastCount;
        private System.Windows.Forms.Button buttonConnect;
        private System.Windows.Forms.Button buttonResetCount;
        private System.Windows.Forms.Panel panel3;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.IO.Ports.SerialPort ComPort;
        private System.Windows.Forms.ComboBox boxComPort;
        public System.Windows.Forms.Label counterCurrentCount;
    }
}

