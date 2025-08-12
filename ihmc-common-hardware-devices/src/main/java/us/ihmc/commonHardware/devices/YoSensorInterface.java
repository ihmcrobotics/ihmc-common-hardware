package us.ihmc.commonHardware.devices;

/**
 * This interface is to be used with any sensor used, so that the update function can be called without needing to know
 * the type of device
 */
public interface YoSensorInterface
{
   /**
    * Update the sensor with the relevant information
    */
   void update();
}
