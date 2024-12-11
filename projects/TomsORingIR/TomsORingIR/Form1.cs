using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using Tulpep.NotificationWindow;
using System.Threading;

namespace TomsORingIR
{
    public partial class Form1 : Form
    {
        private SerialPort myport;
        private bool myport_connected = false;
        public delegate void d1(string indata); //  Delegate to bridge data between gui and serial port
        string currentCount = "0";
        private static Mutex mut = new Mutex();

        public Form1()
        {
            InitializeComponent();
            counterCurrentCount.Text = "123";   //  Remove after testing. Dummy counter number

            string[] ports = SerialPort.GetPortNames();

            Console.WriteLine("The following serial ports were found:");

            // Display each port name to the console.
            foreach (string port in ports)
            {
                boxComPort.Items.Add(port);
                Console.WriteLine(boxComPort.Items);
            }
            myport = new SerialPort();
            myport.BaudRate = 9600;
            Console.ReadLine();
        }

        private void buttonConnect_Click(object sender, EventArgs e)
        {
            if (buttonConnect.Text == "Connect")
            {
                connect();
            }
            else
            {
                disconnect();
            }
        }

        private void buttonResetCount_Click(object sender, EventArgs e)
        {
            myport.DtrEnable= false;
            counterLastCount.Text = counterCurrentCount.Text;
            myport.WriteLine("RC");
            counterCurrentCount.Text = "0";
            myport.DtrEnable = true;
        }

        public void connect()
        {
            try
            {
                if (!(myport_connected))
                {
                    myport.PortName = boxComPort.Text;   //  Set this to a selection list from the control panel
                    myport.DataReceived += new SerialDataReceivedEventHandler(ComPort_DataReceived);
                    myport.Open();                    
                    myport_connected = true;
                    buttonConnect.BackColor = Color.Green;
                    buttonConnect.Text = "Disconnect";
                }
                else
                {

                }
            }
            catch (Exception exc)
            {
                MessageBox.Show(exc.Message, "Could not connect on " + myport.PortName);
            }
        }

        private void disconnect()
        {
            try
            {
                if (myport_connected)
                {
                    myport.Close();
                    myport_connected = false;
                    buttonConnect.BackColor = Color.Red;
                    buttonConnect.Text = "Connect";
                }
                else
                {

                }
            }
            catch (Exception exc)
            {
                MessageBox.Show(exc.Message, "Error");
            }
        }

        private void buttonLED_BUILTIN_Click(object sender, EventArgs e)
        {
            if (buttonLED_BUILTIN.Text == "LED: OFF")
            {
                try
                {
                    myport.WriteLine("BLEDON");
                    buttonLED_BUILTIN.Text = "LED: ON";
                }
                catch (Exception exc)
                {
                    MessageBox.Show(exc.Message, "Error sending command to turn on LED");
                }

            }
            else
            {
                try
                {
                    myport.WriteLine("BLEDOFF");
                    buttonLED_BUILTIN.Text = "LED: OFF";
                }
                catch (Exception exc)
                {
                    MessageBox.Show(exc.Message, "Error sending command to turn off LED");
                }

            }
        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {
            Console.WriteLine(boxComPort.Text);
            myport.PortName = boxComPort.Text;
        }

        private void ComPort_DataReceived(object sender, EventArgs e)
        {            
            string indata = myport.ReadLine();
            //Console.WriteLine(indata);
            char firstchar = indata[0];
            switch (firstchar)
            {
                case 'c':
                    currentCount = indata.Substring(1);
                    Console.WriteLine("Current count = " + currentCount);
                    this.Invoke((MethodInvoker)delegate { counterCurrentCount.Text = currentCount; });
                    break;
            }
        }
            
    }
}
