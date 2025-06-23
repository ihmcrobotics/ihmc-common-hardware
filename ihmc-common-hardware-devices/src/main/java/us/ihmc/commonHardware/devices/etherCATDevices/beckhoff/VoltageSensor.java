package us.ihmc.commonHardware.devices.etherCATDevices.beckhoff;

public interface VoltageSensor
{
   public double getVoltageForChannel(int channel);
}
