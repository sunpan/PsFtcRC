package org.firstinspires.ftc.teamcode.driver;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.util.Hashtable;
import java.util.Locale;

@I2cDeviceType
@DeviceProperties(name = "KS 109", description = "a KS 109", xmlTag = "KS109I2cDistance")  // KS109I2cDistance is built-in
public class KS109I2cDistance extends I2cDeviceSynchDevice<I2cDeviceSynch> implements   I2cAddrConfig
{
    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    public final static int DRIVER_VERSION = 100;
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(0xe8);

    public enum Register
    {
        VERSION(0x00),
        MANUFACTURE_CODE(0x01),
        COMMAND(0x02),
        SETTING_BAUD_MODE(0x04),
        SETTING_NOISE_REDUCTION_MODE(0x06),
        SETTING_ANGLE_MODE(0x07),
        MODEL_CODE(0x09),
        SETTING_SLEEP_AND_DB_MODE(0x10),
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
        SET_SCL_LOW_ON(0x02),
        SET_SCL_LOW_OFF(0x03),
        SET_NOISE_REDUCTION_MODE_71(0x71),
        SET_NOISE_REDUCTION_MODE_72(0x72),
        SET_NOISE_REDUCTION_MODE_73(0x73),
        SET_NOISE_REDUCTION_MODE_74(0x74),
        SET_BAUD_MODE_77(0x77),
        SET_BAUD_MODE_79(0x79),
        SET_SLEEP_AND_DB_MODE_5S_55DB_TO_65DB_C4(0xc4),
        SET_SLEEP_AND_DB_MODE_1S_55DB_TO_65DB_C5(0xc5),
        SET_SLEEP_AND_DB_MODE_5S_45DB_TO_50DB_C6(0xc6),
        SET_SLEEP_AND_DB_MODE_5S_50DB_TO_55DB_C7(0xc7),
        GET_DISTANCE_B0(0xb0),
        GET_DISTANCE_B2(0xb2),
        GET_DISTANCE_B4(0xb4),
        GET_DISTANCE_B8(0xb8),
        GET_DISTANCE_BA(0xba),
        GET_DISTANCE_BC(0xbc),
        UNKNOWN(-1);

        public byte bVal;
        Command(int value) { this.bVal = (byte)value; }
        public static Command fromByte(byte bVal) {
            for (Command command : values()) {
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

      //  setOptimalReadWindow();

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

/*    protected void setOptimalReadWindow()
    {
*//*        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register.READ_WINDOW_FIRST.bVal,
                com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register.READ_WINDOW_LAST.bVal - com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Register.READ_WINDOW_FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);*//*
    }*/

    @Override
    protected synchronized boolean doInitialize()
    {
    /*    this.writeCommand(com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.Command.NORMAL);
        this.resetZAxisIntegrator();
        this.setZAxisScalingCoefficient(1<<8);
        this.headingMode = com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN;
*/

        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override public String getDeviceName()
    {
         return "KS 109";
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    public byte read8(Register reg)
    {
        return this.deviceClient.read8(reg.bVal);
    }
    public void write8(Register reg, byte value)
    {
        this.deviceClient.write8(reg.bVal, value);
    }

    public short readShortBE(Register reg)
    {
        return TypeConversion.byteArrayToShort(this.deviceClient.read(reg.bVal, 2), ByteOrder.BIG_ENDIAN);
    }
    public void writeShortBE(Register reg, short value)
    {
        this.deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value, ByteOrder.BIG_ENDIAN));
    }
    public short readShortLE(Register reg)
    {
        return TypeConversion.byteArrayToShort(this.deviceClient.read(reg.bVal, 2), ByteOrder.LITTLE_ENDIAN);
    }
    public void writeShortLE(Register reg, short value)
    {
        this.deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value, ByteOrder.LITTLE_ENDIAN));
    }

    public void writeCommand(Command command)
    {
        // Wait for any previous command write to finish so we don't clobber it
        // before the USB controller gets a chance to see it and pass it on to the sensor
        this.deviceClient.waitForWriteCompletions(I2cWaitControl.ATOMIC);
        this.write8(Register.COMMAND, command.bVal);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public double getDistance()
    {
        writeCommand(Command.GET_DISTANCE_BC);
        this.deviceClient.waitForWriteCompletions(I2cWaitControl.ATOMIC);
        safeSleep(100);
        return readShortBE(Register.COMMAND)&0xFFFF;
    }

    public Hashtable<String, Object> getDeviceInfo()
    {
        Hashtable<String, Object> info= new Hashtable<String, Object>();
        info.put("Model Code", read8(Register.MODEL_CODE));
        info.put("Hardware Version", read8(Register.VERSION)&0xFF);
        info.put("Driver Version", DRIVER_VERSION);
        int ManufactureCode= read8(Register.MANUFACTURE_CODE);;
        info.put("Year", ((ManufactureCode &0xf0)>>4)+10);
        info.put("Month", ManufactureCode&0xF);
        return info;
    }

    public Hashtable<String, Object> getSettingInfo()
    {
        Hashtable<String, Object> info = new Hashtable<String, Object>();
        info.put("BaudMode", String.format("0x%02x", read8(Register.SETTING_BAUD_MODE)));
        info.put("SleepAndDBMode", String.format("0x%02x", read8(Register.SETTING_SLEEP_AND_DB_MODE)));
        info.put("NoiseReductionMode", String.format("0x%02x", read8(Register.SETTING_NOISE_REDUCTION_MODE)));
        info.put("AngleModeForB0B2B4", String.format("0x%02x", read8(Register.SETTING_ANGLE_MODE)));
        return info;
    }

    public void slowConfig()
    {
        writeCommand(Command.SET_SCL_LOW_OFF);
        writeCommand(Command.SET_BAUD_MODE_79);
        writeCommand(Command.SET_NOISE_REDUCTION_MODE_71);
        writeCommand(Command.SET_SLEEP_AND_DB_MODE_1S_55DB_TO_65DB_C5);
        safeSleep(2000);
    }

    public void safeSleep(int millis)
    {
        try {
            Thread.sleep(millis);
        }
        catch (Exception e)
        {

        }
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


}
