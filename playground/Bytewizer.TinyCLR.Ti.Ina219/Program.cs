using System.Text;
using System.Threading;
using System.Diagnostics;

using GHIElectronics.TinyCLR.Pins;
using GHIElectronics.TinyCLR.Devices.I2c;
using Bytewizer.TinyCLR.Drivers.Ti.Ina219;

namespace Bytewizer.TinyCLR.Ina219
{
    public class Program
    {
        static void Main()
        {
            var controller = I2cController.FromName(SC20260.I2cBus.I2c2);
            var device = new Ina219Controller(controller);

            device.Reset();

            device.Sensitivity = Ina219Sensitivity.PlusOrMinus40mv;
            device.BusVoltageRange = Ina219BusVoltageRange.Range16v;
            device.SetCalibration(33574, (float)12.2e-6);

            Debug.WriteLine($"Sensitivity        :{device.Sensitivity}");
            Debug.WriteLine($"Bus Voltage Range  :{device.BusVoltageRange}");
            Debug.WriteLine($"Operating Mode     :{device.OperatingMode}");
            Debug.WriteLine($"Bus Resolution     :{device.BusResolution}");
            Debug.WriteLine($"Shunt Resolution   :{device.ShuntResolution}");

            while (true)
            {
                var sb = new StringBuilder();
                
                sb.Append($"Current {device.ReadCurrent():F3} mA :: ");
                sb.Append($"Bus Voltage {device.ReadVoltage():F3} V :: ");
                sb.Append($"Shunt Voltage {device.ReadShuntVoltage():F3} mV :: ");
                sb.Append($"Power {device.ReadPower():F3} mW");
                
                Debug.WriteLine(sb.ToString());
                Thread.Sleep(1);
            }
        }
    }
}