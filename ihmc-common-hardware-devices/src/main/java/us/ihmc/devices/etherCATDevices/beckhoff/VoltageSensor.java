package us.ihmc.devices.etherCATDevices.beckhoff;

public interface VoltageSensor
{
   public double getVoltageForChannel(int channel);
}
