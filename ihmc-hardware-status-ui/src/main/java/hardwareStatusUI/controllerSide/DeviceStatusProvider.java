package hardwareStatusUI.controllerSide;

/**
 * Interface for any class that wishes to provide status info of a generic hardware device.
 * If registered with {@code HardwareStatusManager}, this information can be displayed in
 * the {@code HardwareStatusUI}.
 */
public interface DeviceStatusProvider
{
   /**
    * @return True if the sensor is responding properly, false otherwise
    */
   boolean isResponding();
}
