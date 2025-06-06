package us.ihmc.devices;

import org.ejml.data.DMatrixRMaj;
import java.util.Map;

public interface ForceSensorManagerInterface
{
   void read(Map<String, DMatrixRMaj> forceSensorMeasurement);

   void write();

   void calibrate();

   String getName();
}
