package us.ihmc.commonHardware.devices.genericSensor;

import org.ejml.data.DMatrixRMaj;
import java.util.Map;

public interface ForceSensorManagerInterface
{
   /**
    * Reads the data from the force sensor and applies the necessary filtering, transforms, and any other needed operations.
    * The sensor name is needed to access the correct sensor from the map
    * @param forceSensorMeasurement Map of the matrices that hold the forces and torques from the force sensor
    */
   void read(Map<String, DMatrixRMaj> forceSensorMeasurement);

   /**
    * Write any necessary commands to the force sensor. Defaulted to do nothing
    */
   default void write()
   {
      //do nothing
   }

   /**
    * Calibrate/Zero/Tare the force sensor using this method
    */
   void calibrate();

   /**
    * @return the name of the force sensor
    */
   String getName();
}
