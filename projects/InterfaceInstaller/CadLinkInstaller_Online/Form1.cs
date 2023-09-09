/*******    TO DO's
        
        1.  Thomas has auto installers for most of these files - Link: 
        2.  First, auto download and manual install, than ask engineering to help automate
        3.  Compile this into an exe and find some tools to test against AV false positives
        4.  Iterate through and get values of selected radio's and store as variables
        5.  Print versions as variables from radials
        6.  Is it better to use a switch statement or an if statment for going through selected radial buttons

*/

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.IO.Compression;

namespace CadLinkHL7InterfaceSetup
{
    public partial class formMain : Form
    {

        public formMain()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        string DownloadPathRoot;
        string PackageName;
        readonly string pkgCadLinkServer5319 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/standardized-packages/CadLink_Server_5.3_and_Tools.zip";
        readonly string pkgCadLinkThickClient5319 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/software/cadlink/5.3/CadLinkClientInstaller.zip";
        readonly string pkgCadLinkThinClient5319 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/software/cadlink/5.3/CadLinkThinClientInstaller.zip";
        readonly string pkgCadLinkServer5223 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/software/cadlink/5.2/5.2.23/install.zip";
        readonly string pkgCadLinkThickClient5223 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/software/cadlink/5.2/5.2.23/CadLinkClientInstaller.zip";
        readonly string pkgCadLinkThinClient5223 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/software/cadlink/5.3/CadLinkThinClientInstaller.zip";
        readonly string pkgHL7 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/standardized-packages/CadwellInterface.zip";
        readonly string pkgArc30SP2 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/software/arc/3.0.234(SP2)/ArcSetup.exe";
        readonly string pkgArc30SP2DTS4576 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/software/dts/arc/4576/Installer.zip";
        readonly string pkgCSSSP2 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/standardized-packages/Cascade_3.5.1520.zip";
        readonly string pkgSierraSummit31537 = "https://cadwell-sgtpr8yrulvhzdpxsdr.netdna-ssl.com/enterprise/standardized-packages/Summit_3.1.537_Package_v01.zip";


        public async void BtnDownload_Click(object sender, EventArgs e)
        {
            //  Prompt for download location
            FolderBrowserDialog folderBrowserDialog1 = new FolderBrowserDialog();
            if (folderBrowserDialog1.ShowDialog() == DialogResult.OK)
            {
                DownloadPathRoot = folderBrowserDialog1.SelectedPath + "\\";
            }
            var cntls = GetAll(this, typeof(RadioButton));
            foreach (Control cntrl in cntls)
            {
                RadioButton _rb = (RadioButton)cntrl;
                String DownloadFileName = "";                
                if (_rb.Checked)
                {

                    string selectedRdo = _rb.Name;
                    PackageName = null;
                    switch (selectedRdo)
                    {
                        case "RdoSrv5319":
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);                        
                            PackageName = "CadLink_Server_5.3.19.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkServer5319, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoSrv5222":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLink_Server_5.2.23.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkServer5223, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoHL7":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLinkHL7InterfaceInstaller.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgHL7, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoCln5319":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "";
                            PackageName = "CadLink_Client_5.3.19.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkThickClient5319, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoThn5319":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLink_ThinClient_5.3.19.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkThinClient5319, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoCln5222":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLink_Client5.2.22.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkThickClient5223, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoThn5222":
                            
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLink_ThinClient5.2.22.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkThinClient5223, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoArcSp2":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "Arc_3.0_SP2.exe";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgArc30SP2, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoCssSp2":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CascadeSurgicalStudio_SP2.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCSSSP2, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoSmt315":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "SierraSummit_3.1.537.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgSierraSummit31537, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        default:
                            //  no clicky, me on breaky
                            break;
                    }
                }
            }
        }

//  *****************           INSTALL TIME        **************************************

        private async void BtnInstall_Click(object sender, EventArgs e)
        {
            //  Prompt for download location
            FolderBrowserDialog folderBrowserDialog1 = new FolderBrowserDialog();
            if (folderBrowserDialog1.ShowDialog() == DialogResult.OK)
            {
                DownloadPathRoot = folderBrowserDialog1.SelectedPath + "\\";
            }
            var cntls = GetAll(this, typeof(RadioButton));
            foreach (Control cntrl in cntls)
            {
                RadioButton _rb = (RadioButton)cntrl;
                String DownloadFileName = "";
                if (_rb.Checked)
                {

                    string selectedRdo = _rb.Name;
                    PackageName = null;
                    switch (selectedRdo)
                    {
                        case "RdoHL7":
                            //System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);                            
                            PackageName = "CadLinkHL7InterfaceInstaller";
                            appendOutput("You selected " + PackageName);
                            DownloadFileName = DownloadPathRoot + PackageName + ".zip";
                            appendOutput("Starting " + PackageName + " download");
                            //await getSoftwarePackage(pkgHL7, DownloadFileName);
                            string ExtractDirectory = DownloadPathRoot + "CadLink HL7 Interface Resources";
                            appendOutput("Extracting " + DownloadFileName + " to " + ExtractDirectory);
                            //ZipFile.ExtractToDirectory(DownloadFileName, ExtractDirectory);
                            appendOutput(PackageName + " successfully extracted");

                            string HL7ResourcesDir = ExtractDirectory + "\\Installers\\";                            
                            string EdgeInstaller = HL7ResourcesDir + "MicrosoftEdgeEnterpriseX64.msi";
                            string OpenJDKInstaller = HL7ResourcesDir + "OpenJDK11U-jdk_x64_windows_hotspot_11.0.8_10.msi";
                            string jdkInstallArgs = "ADDLOCAL=FeatureMain,FeatureEnvironment,FeatureJarFileRunWith,FeatureJavaHome,FeatureOracleJavaSoft INSTALLDIR=" + "\"C:\\Program Files\\AdoptOpenJDK\"" + " /quiet";
                            string instMirth = HL7ResourcesDir + "mirthconnect-3.9.1.b263-windows-x64.exe";
                            string instMirthArguments = "-q -console -varfile .\\3.9_unattended.var";
                            string instMirthAL = HL7ResourcesDir + "mirth-administrator-launcher-1.2.0-windows-x64.exe";
                            string instMirthALArguments = "INSTALLLEVEL=1 -q";
                            string instPGSQL = HL7ResourcesDir + "postgresql-12.5-1-windows-x64.exe";
                            string instPGSQLArguments = "--mode unattended --unattendedmodeui minimalWithDialogs --datadir \"C:\\Program Files\\PostgreSQL\\12\\data\" --enable-components server,pgAdmin,commandlinetools --superpassword C@dw3ll";
                            await installSoftwarePackage(EdgeInstaller, "/q");
                            await installSoftwarePackage(OpenJDKInstaller, jdkInstallArgs);
                            await installSoftwarePackage(instMirth, instMirthArguments);
                            await installSoftwarePackage(instMirthAL, instMirthALArguments);
                            await installSoftwarePackage(instPGSQL, instPGSQLArguments);

                            DownloadFileName = null;
                            break;

                        case "RdoSrv5319":
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLink_Server_5.3.19.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkServer5319, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoSrv5222":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLink_Server_5.2.23.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkServer5223, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoCln5319":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "";
                            PackageName = "CadLink_Client_5.3.19.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkThickClient5319, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoThn5319":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLink_ThinClient_5.3.19.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkThinClient5319, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoCln5222":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLink_Client5.2.22.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkThickClient5223, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoThn5222":

                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CadLink_ThinClient5.2.22.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCadLinkThinClient5223, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoArcSp2":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "Arc_3.0_SP2.exe";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgArc30SP2, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoCssSp2":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "CascadeSurgicalStudio_SP2.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgCSSSP2, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        case "RdoSmt315":
                            //isDownloaded = false;
                            System.Windows.Forms.MessageBox.Show("You selected " + selectedRdo);
                            PackageName = "SierraSummit_3.1.537.zip";
                            DownloadFileName = DownloadPathRoot + PackageName;
                            await getSoftwarePackage(pkgSierraSummit31537, DownloadFileName);
                            DownloadFileName = null;
                            break;

                        default:
                            //  no clicky, me on breaky
                            break;
                    }
                }
            }
        }

        private void BtnClrSelections_Click(object sender, EventArgs e)
        {
            var cntls = GetAll(this, typeof(RadioButton));
            foreach (Control cntrl in cntls)
            {
                RadioButton _rb = (RadioButton)cntrl;
                if (_rb.Checked)
                {
                    _rb.Checked = false;
                }
            }
        }

        public IEnumerable<Control> GetAll(Control control, Type type)
        {
            var controls = control.Controls.Cast<Control>();
            return controls.SelectMany(ctrls => GetAll(ctrls, type)).Concat(controls).Where(c => c.GetType() == type);
        }

        public async Task getSoftwarePackage(string packageUrl, string packagePath)
        {
            pBarDownload.Value = 0;
            WebClient webClient = new WebClient();
            Uri uri = new Uri(packageUrl);
            webClient.DownloadProgressChanged += new DownloadProgressChangedEventHandler(pBarDownload_Callback);
            await webClient.DownloadFileTaskAsync(uri, packagePath);
        
        }

        public Task installSoftwarePackage(string PackageFile, string arguments)
        {
            appendOutput("Installing " + PackageFile);    //  Generate function for HL7 Installation process in order to create callbacks for progress bars
            System.Diagnostics.Process InstallerProcess = System.Diagnostics.Process.Start(PackageFile, arguments);
            InstallerProcess.WaitForExit(300);

            /*
             *      *********** My 1st Attempt at a better way to wait until thie first process finishes   ************
                  
            InstallerProcess.EnableRaisingEvents = true;
            InstallerProcess.StartInfo.FileName = PackageFile;
            InstallerProcess.StartInfo.Arguments = arguments;
            InstallerProcess.StartInfo.UseShellExecute = true;
            InstallerProcess.StartInfo.RedirectStandardOutput = true;            
            InstallerProcess.WaitForExit(300);
            while (!InstallerProcess.StandardOutput.EndOfStream)
            {
                appendOutput(InstallerProcess.StandardOutput.ReadLine());
            }            
            */
            appendOutput(PackageFile + " installed");
            return Task.CompletedTask;   
        }

        private void pBarDownload_Callback(object sender, DownloadProgressChangedEventArgs e)
        {
            pBarDownload.Value = e.ProgressPercentage;
            lblDownloadProgress.Text = PackageName + " " + e.ProgressPercentage + "% complete";
            lblDownloadProgress.Refresh();
        }

        private void appendOutput(String msg)
        {
            rtbOutputWindow.AppendText("\r\n" + msg + "\r\n");
        }

        private void btnExportLog_Click(object sender, EventArgs e)
        {
            SaveFileDialog saveFile1 = new SaveFileDialog();

            // Initialize the SaveFileDialog to specify the RTF extension for the file.
            saveFile1.DefaultExt = "*.rtf";
            saveFile1.Filter = "RTF Files|*.rtf";

            // Determine if the user selected a file name from the saveFileDialog.
            if (saveFile1.ShowDialog() == System.Windows.Forms.DialogResult.OK &&
               saveFile1.FileName.Length > 0)
            {
                // Save the contents of the RichTextBox into the file.
                rtbOutputWindow.SaveFile(saveFile1.FileName, RichTextBoxStreamType.PlainText);
            }
        }
    }
#pragma warning disable CS1022 // Type or namespace definition, or end-of-file expected
}
#pragma warning restore CS1022 // Type or namespace definition, or end-of-file expected
