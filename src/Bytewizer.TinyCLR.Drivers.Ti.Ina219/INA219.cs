using System;
using System.Threading;

using GHIElectronics.TinyCLR.Devices.I2c;

namespace Bytewizer.TinyCLR.Drivers.Ti.Ina219
{
    /// <summary>
    /// Configures the <see cref="Ina219Controller"/> to use I2C bus for communication with the a host.
    /// </summary>
    public class Ina219Controller : IDisposable
    {
        /// <summary>
        /// INA219 I2c device address.
        /// </summary>
        private const int I2cAddress = 0x40;

        /// <summary>
        /// I2C device that represent the INA219 connection.
        /// </summary>
        private readonly I2cDevice i2cDevice;

        private uint CalibrationRegister { get; set; }
        private float CurrentLsb { get; set; }

        /// <summary>
        /// Initializes a new instance of the <see cref="Ina219Controller"/> class.
        /// </summary>
        /// <param name="i2cController">The i2c controller to use.</param>
        public Ina219Controller(I2cController i2cController)
            : this(i2cController, new I2cConnectionSettings(I2cAddress)
            {
                BusSpeed = 100000,
                AddressFormat = I2cAddressFormat.SevenBit
            })
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Ina219Controller"/> class.
        /// </summary>
        /// <param name="i2cController">The i2c controller to use.</param>
        /// <param name="i2cSettings">The i2c controller settings to use.</param>
        public Ina219Controller(I2cController i2cController, I2cConnectionSettings i2cSettings)
        {
            if (i2cController == null)
            {
                throw new ArgumentNullException(nameof(i2cController));
            }

            if (i2cSettings == null)
            {
                throw new ArgumentNullException(nameof(i2cSettings));
            }

            CurrentLsb = 1F;
            byte[] buffer = { 0 };

            try
            {
                this.i2cDevice = i2cController.GetDevice(i2cSettings);
                this.i2cDevice.WriteRead(buffer, buffer);

                if (buffer[0] != 0x39)
                {
                    Reset();
                }
            }
            catch (Exception ex)
            {
                throw new Exception("INA219 is not responding", ex);
            }
        }

        /// <summary>
        /// Pro-actively frees resources owned by this instance.
        /// </summary>
        public void Dispose()
            => this.i2cDevice.Dispose();

        /// <summary>
        /// Reset the INA219 to default values.
        /// </summary>
        public void Reset()
        {
            this.i2cDevice.WriteRegister(Ina219Register.Configuration, (ushort)Ina219ConfigurationFlags.Rst);

            CalibrationRegister = 0;

        }

        /// <summary>
        /// Set the INA219 calibration value used to scale the Shunt voltage into a Current reading.
        /// </summary>
        /// <remarks>
        /// This method allows the user to manually specify the value written to the INA219 calibration register which determines how the shunt voltage
        /// reading is scaled into the current register by the INA219. To allow finer control of the scaling the current register does not contain the actual
        /// current value and the currentLsb is used to specify how much current in Amperes is represented by the least significant bit of the current register.
        /// This will allow the ReadCurrent method to return the corrent current value and by implication ReadPower to return the correct power value as it is derived froom
        /// the current value.
        /// <see cref="ReadPower"/><see cref="ReadCurrent"/><seealso href="http://www.ti.com/lit/ds/symlink/ina219.pdf"/>
        /// </remarks>
        /// <param name="register">The number of Amperes represented by the LSB of the INA219 current register.</param>
        /// <param name="lsb">The current value in Amperes of the least significan bit of the calibration register. Defaults to unity so that the register can be read directly.</param>
        public void SetCalibration(ushort register, float currentLsb = 1.0F)
        {
            // cache the values for later use
            CalibrationRegister = register;
            CurrentLsb = currentLsb;

            // set the INA219 calibration value
            this.i2cDevice.WriteRegister(Ina219Register.Calibration, register);
        }

        /// <summary>
        /// Property representing the Operating mode of the INA219
        /// </summary>
        /// <remarks>
        /// This allows the user to selects continuous, triggered, or power-down mode of operation along with which of the shunt and bus voltage measurements are made.
        /// </remarks>
        public Ina219OperatingMode OperatingMode
        {
            get => (Ina219OperatingMode)(this.i2cDevice.ReadRegister(Ina219Register.Configuration)
                & (ushort)Ina219ConfigurationFlags.ModeMask);
            set
            {
                ushort regValue = this.i2cDevice.ReadRegister(Ina219Register.Configuration);

                regValue &= (ushort)~Ina219ConfigurationFlags.ModeMask;
                regValue |= (ushort)value;

                this.i2cDevice.WriteRegister(Ina219Register.Configuration, regValue);
            }
        }

        /// <summary>
        /// Property representing the Bus voltage range of the INA219
        /// </summary>
        /// <remarks>
        /// This allows the user to selects eiter a 16V range or a 32V range for the ADC reading the bus voltage.
        /// In general the lowest range compatible with the application parameters should be selected.
        /// </remarks>
        public Ina219BusVoltageRange BusVoltageRange
        {
            get => (Ina219BusVoltageRange)(this.i2cDevice.ReadRegister(Ina219Register.Configuration)
                & (ushort)Ina219ConfigurationFlags.BrngMask);
            set
            {
                ushort regValue = this.i2cDevice.ReadRegister(Ina219Register.Configuration);

                regValue &= (ushort)~Ina219ConfigurationFlags.BrngMask;
                regValue |= (ushort)value;

                this.i2cDevice.WriteRegister(Ina219Register.Configuration, regValue);
            }
        }

        /// <summary>
        /// Property representing the voltage range of the Programable Gain Amplifier used in the INA219 to measure Shunt Voltage.
        /// </summary>
        /// <remarks>
        /// This allows the user to selects a gain for the amplifier reading the shunt voltage before it is applied to the ADC. It can be one of +/-40mV, +/-80mV, +/-160mV or +/-320mV.
        /// In general the lowest range compatible with the application parameters should be selected.
        /// </remarks>
        public Ina219Sensitivity Sensitivity
        {
            get => (Ina219Sensitivity)(this.i2cDevice.ReadRegister(Ina219Register.Configuration)
                & (ushort)Ina219ConfigurationFlags.PgaMask);
            set
            {
                ushort regValue = this.i2cDevice.ReadRegister(Ina219Register.Configuration);

                regValue &= (ushort)~Ina219ConfigurationFlags.PgaMask;
                regValue |= (ushort)value;

                this.i2cDevice.WriteRegister(Ina219Register.Configuration, regValue);
            }
        }

        /// <summary>
        /// Set the Ina219 ADC resolution or samples to be used when reading the Bus voltage.
        /// </summary>
        /// <remarks>
        /// This can either by the number of bits used for the ADC conversion (9-12 bits) or the number of samples at 12 bits to be averaged for the result.
        /// </remarks>
        public Ina219AdcResolution BusResolution
        {
            get => (Ina219AdcResolution)((this.i2cDevice.ReadRegister(Ina219Register.Configuration)
                & (ushort)Ina219ConfigurationFlags.BadcMask) >> 4);
            set
            {
                ushort regValue = this.i2cDevice.ReadRegister(Ina219Register.Configuration);

                regValue &= (ushort)~Ina219ConfigurationFlags.BadcMask;
                regValue |= (ushort)((ushort)value << 4);

                this.i2cDevice.WriteRegister(Ina219Register.Configuration, regValue);
            }
        }

        /// <summary>
        /// Set the INA219 ADC resolution or samples to be used when reading the Shunt voltage.
        /// </summary>
        /// <remarks>
        /// This can either by the number of bits used for the ADC conversion (9-12 bits) or the number of samples at 12 bits to be averaged for the result.
        /// </remarks>
        public Ina219AdcResolution ShuntResolution
        {
            get => (Ina219AdcResolution)(this.i2cDevice.ReadRegister(Ina219Register.Configuration)
                & (ushort)Ina219ConfigurationFlags.SadcMask);
            set
            {
                ushort regValue = this.i2cDevice.ReadRegister(Ina219Register.Configuration);

                regValue &= (ushort)~Ina219ConfigurationFlags.SadcMask;
                regValue |= (ushort)((ushort)value << 4);

                this.i2cDevice.WriteRegister(Ina219Register.Configuration, regValue);

                ShuntResolution = value;
            }
        }

        /// <summary>
        /// Reads the measured current in milliamps (mA).
        /// </summary>
        public float ReadCurrent() // 27.9 mA
        {
            var regValue = this.i2cDevice.ReadRegister(Ina219Register.Current, 1);

            return (regValue * CurrentLsb) * 1000f;
        }

        /// <summary>
        /// Reads the measured bus voltage in volts (V).
        /// </summary>
        public float ReadVoltage() // 5.03 V
        {
            var regValue = this.i2cDevice.ReadRegister(Ina219Register.BusVoltage, 1);

            return ((short)regValue >> 3) * 4 / 1000f;
        }

        /// <summary>
        /// Reads the measured shunt voltage millivolts (mV).
        /// </summary>
        /// <returns>The shunt potential difference.</returns>
        public float ReadShuntVoltage() //2.83 mV
        {
            var regValue = this.i2cDevice.ReadRegister(Ina219Register.ShuntVoltage, 1);

            return (float)(regValue * 10.0 / 1000000.0) * 1000f;
        }

        /// <summary>
        /// Reads the calculated power in milliwatts (mW).
        /// </summary>
        /// <remarks>
        /// This value is determined by an internal calculation using the calulated current and the bus voltage and then scaled.
        /// </remarks>
        public float ReadPower()
        {
            var regValue = this.i2cDevice.ReadRegister(Ina219Register.Power, 1);

            return (regValue * CurrentLsb * 20) * 1000f;
        }
    }

    /// <summary>
    /// Extensions for work with I2C devices.
    /// </summary>
    public static class I2cExtensions
    {
        /// <summary>
        /// Reads bytes from a register.
        /// </summary>
        /// <param name="device">Device to use.</param>
        /// <param name="register" > The register to read.</param>
        /// <param name="delay">A delay between write and read to wait for any sampling, average or conversion. Defaults to 0</param>
        /// <returns>An unsiged short integer representing the regsiter contents.</returns>
        public static ushort ReadRegister(this I2cDevice device, Ina219Register register, int delay = 0)
        {
            byte[] writeBuffer = new byte[1] { (byte)register };
            byte[] readBuffer = new byte[2];

            device.Write(writeBuffer);

            if (delay > 0)
            {
                Thread.Sleep(delay / 1000); // this can be improved to use ticks
            }

            device.Read(readBuffer);

            return (ushort)(readBuffer[0] << 8 | readBuffer[1]);
        }

        /// <summary>
        /// Writes bytes to a register.
        /// </summary>
        /// <param name="device">Device to use.</param>
        /// <param name="register">The register to be written to.</param>
        /// <param name="value">The value to be writtent to the register.</param>
        public static void WriteRegister(this I2cDevice device, Ina219Register register, ushort value)
        {
            byte[] buffer = new byte[3];

            buffer[0] = (byte)register;
            buffer[1] = (byte)(value >> 8);
            buffer[2] = (byte)value;

            device.Write(buffer);
        }
    }

    /// <summary>
    /// Holds correct register locations for all registers in a INA219 device.
    /// </summary>
    public enum Ina219Register : byte
    {
        /// <summary>
        /// Location of configuration register (Read/Write).
        /// </summary>
        Configuration = 0x00,

        /// <summary>
        /// Location of Shunt Voltage register (Read). 
        /// </summary>
        ShuntVoltage = 0x01,

        /// <summary>
        /// Location of Bus Voltage register (Read). 
        /// </summary>
        BusVoltage = 0x02,

        /// <summary>
        /// Location of Power register (Read). 
        /// </summary>
        Power = 0x03,

        /// <summary>
        /// Location of Current register (Read).
        /// </summary>
        Current = 0x04,

        /// <summary>
        /// Location of Calibration register (Read/Write).
        /// </summary>
        Calibration = 0x05
    }

    /// <summary>
    /// An enumeration representing flags and masks using in the configuration register on the INA219 device.
    /// </summary>
    [Flags]
    public enum Ina219ConfigurationFlags : ushort
    {
        Rst = 0b10000000_00000000,
        BrngMask = 0b00100000_00000000,
        ModeMask = 0b00000000_00000111,
        PgaMask = 0b00011000_00000000,
        BadcMask = 0b00000111_10000000,
        SadcMask = 0b00000000_01111000,
    }

    /// <summary>
    /// An enumeration representing the operating modes available on the INA219 device.
    /// </summary>
    public enum Ina219OperatingMode : ushort
    {
        /// <summary>
        /// Power Down mode
        /// </summary>
        PowerDown = 0b00000000_00000000,

        /// <summary>
        /// Mode to read the shunt voltage on demand
        /// </summary>
        ShuntVoltageTriggered = 0b00000000_00000001,

        /// <summary>
        /// Mode to read the bus voltage on demand
        /// </summary>
        BusVoltageTriggered = 0b00000000_00000010,

        /// <summary>
        /// Mode to read the shunt and bus voltage on demand
        /// </summary>
        ShuntAndBusTriggered = 0b00000000_00000011,

        /// <summary>
        /// Mode to disable the ADC
        /// </summary>
        AdcOff = 0b00000000_00000100,

        /// <summary>
        /// Mode to read the shunt voltage on continuously
        /// </summary>
        ShuntVoltageContinuous = 0b00000000_00000101,

        /// <summary>
        /// Mode to read the bus voltage on continuously
        /// </summary>
        BusVoltageContinuous = 0b00000000_00000110,

        /// <summary>
        /// Mode to read the shunt and bus voltage on continuously
        /// </summary>
        ShuntAndBusContinuous = 0b00000000_00000111
    }

    /// <summary>
    /// An enumeration representing possible bus voltage measurment ranges available on the INA219 device.
    /// </summary>
    public enum Ina219BusVoltageRange : ushort
    {
        /// <summary>
        /// Bus voltage range of 0 - 16V
        /// </summary>
        Range16v = 0b00000000_00000000,

        /// <summary>
        /// Bus voltage range of 0 - 32V
        /// </summary>
        Range32v = 0b00100000_00000000,
    }

    /// <summary>
    /// An enumeration representing the shunt Programable Gain Amplifier sensitivities available on the INA219 device.
    /// </summary>
    public enum Ina219Sensitivity : ushort
    {
        /// <summary>
        /// Pga range of +/- 40mV
        /// </summary>
        PlusOrMinus40mv = 0b00000000_00000000,

        /// <summary>
        /// Pga range of +/- 80mV
        /// </summary>
        PlusOrMinus80mv = 0b00001000_00000000,

        /// <summary>
        /// Pga range of +/- 160mV
        /// </summary>
        PlusOrMinus160mv = 0b00010000_00000000,

        /// <summary>
        /// Pga range of +/- 320mV
        /// </summary>
        PlusOrMinus320mv = 0b00011000_00000000,
    }

    /// <summary>
    /// An enumeration representing ADC resolution and samples available on the INA219 device for reading the shunt and bus voltages.
    /// </summary>
    public enum Ina219AdcResolution
    {
        /// <summary>
        /// 9 bit single Sample
        /// </summary>
        Adc9Bit = 0b00000000_00000000,

        /// <summary>
        /// 10 bit single Sample
        /// </summary>
        Adc10Bit = 0b00000000_00001000,

        /// <summary>
        /// 11 bit single Sample
        /// </summary>
        Adc11Bit = 0b00000000_00010000,

        /// <summary>
        /// 12 bit single Sample
        /// </summary>
        Adc12Bit = 0b00000000_00011000,

        /// <summary>
        /// 12 bit 2 samples averaged
        /// </summary>
        Adc2Sample = 0b00000000_01001000,

        /// <summary>
        /// 12 bit 4 samples averaged
        /// </summary>
        Adc4Sample = 0b00000000_01010000,

        /// <summary>
        /// 12 bit 8 samples averaged
        /// </summary>
        Adc8Sample = 0b00000000_01011000,

        /// <summary>
        /// 12 bit 16 samples averaged
        /// </summary>
        Adc16Sample = 0b00000000_01100000,

        /// <summary>
        /// 12 bit 32 samples averaged
        /// </summary>
        Adc32Sample = 0b00000000_01101000,

        /// <summary>
        /// 12 bit 64 samples averaged
        /// </summary>
        Adc64Sample = 0b00000000_01110000,

        /// <summary>
        /// 12 bit 128 samples averaged
        /// </summary>
        Adc128Sample = 0b00000000_01111000,
    }
}