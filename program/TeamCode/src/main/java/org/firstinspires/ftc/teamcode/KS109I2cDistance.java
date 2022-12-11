package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.nio.ByteOrder;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;




// @I2cSensor(name = "KS 109", description = "a KS 109", xmlTag = "KS109I2cDistance")  // KS109I2cDistance is built-in

public class KS109I2cDistance extends I2cDeviceSynchDevice<I2cDeviceSynch>
        implements   I2cAddrConfig
{
    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(0xe8);

    public enum Register
    {
     //   READ_WINDOW_FIRST(0x00),
        MANUFACTURE_CODE(0x01),
        COMMAND(0x02),
        DATA_LOW8BIT(0x03),
        BAUD(0x04),
        I2C_ADDRESS_LIST(0x05),
        NOISE_REDUCTION_LEVEL(0x06),
        ANGLE_LEVEL(0x06),
        FIRMWARE_REV(0x08),
        MODULE_CODE(0x09),
        SETTINGS_SLEEP_AND_DB(0x10),
        UNKNOWN(-1);

        public byte bVal;
        Register(int value) { this.bVal = (byte)value; }
        public static KS109I2cDistance.Register fromByte(byte bVal) {
            for (KS109I2cDistance.Register register : values()) {
                if (register.bVal == bVal) return register;
            }
            return UNKNOWN;
        }
    }

    public enum Command
    {
        NORMAL(0x00),
        CALIBRATE(0x4E),
        RESET_Z_AXIS(0x52),
        WRITE_EEPROM(0x57),
        UNKNOWN(-1);

        public byte bVal;
        Command(int value) { this.bVal = (byte)value; }
        public static KS109I2cDistance.Command fromByte(byte bVal) {
            for (KS109I2cDistance.Command command : values()) {
                if (command.bVal == bVal) return command;
            }
            return UNKNOWN;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public KS109I2cDistance(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        setOptimalReadWindow();

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow()
    {
/*        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register.READ_WINDOW_FIRST.bVal,
                com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register.READ_WINDOW_LAST.bVal - com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register.READ_WINDOW_FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);*/
    }

    @Override
    protected synchronized boolean doInitialize()
    {
    /*    this.writeCommand(com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Command.NORMAL);
        this.resetZAxisIntegrator();
        this.setZAxisScalingCoefficient(1<<8);
        this.headingMode = com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN;
*/        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override public String getDeviceName()
    {
         return String.format(Locale.getDefault(), "KS 109 %s", "");
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    public byte read8(com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register reg)
    {
        return this.deviceClient.read8(reg.bVal);
    }
    public void write8(com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register reg, byte value)
    {
        this.deviceClient.write8(reg.bVal, value);
    }

    public short readShort(com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register reg)
    {
        return TypeConversion.byteArrayToShort(this.deviceClient.read(reg.bVal, 2), ByteOrder.LITTLE_ENDIAN);
    }
    public void writeShort(com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register reg, short value)
    {
        this.deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value, ByteOrder.LITTLE_ENDIAN));
    }

    public void writeCommand(com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Command command)
    {
        // Wait for any previous command write to finish so we don't clobber it
        // before the USB controller gets a chance to see it and pass it on to the sensor
        this.deviceClient.waitForWriteCompletions(I2cWaitControl.ATOMIC);

        this.write8(com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register.COMMAND, command.bVal);
    }

    public com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Command readCommand()
    {
        return com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Command.fromByte(this.read8(com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register.COMMAND));
    }



    //----------------------------------------------------------------------------------------------
    // I2cAddrConfig interface
    //----------------------------------------------------------------------------------------------

    @Override public void setI2cAddress(I2cAddr newAddress)
    {
        // In light of the existence of I2C multiplexers, we don't *require* a valid Modern Robotics I2cAddr
        this.deviceClient.setI2cAddress(newAddress);
    }

    @Override public I2cAddr getI2cAddress()
    {
        return this.deviceClient.getI2cAddress();
    }



    protected void notSupported()
    {
        throw new UnsupportedOperationException("This method is not supported for " + getDeviceName());
    }
}
