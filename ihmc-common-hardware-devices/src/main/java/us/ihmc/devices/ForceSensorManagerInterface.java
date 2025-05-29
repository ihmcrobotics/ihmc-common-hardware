package us.ihmc.devices;

import org.ejml.data.DMatrixRMaj;

public interface ForceSensorManagerInterface
{
   void read(DMatrixRMaj forceSensorMeasurement);

   void write();

   void calibrate();

   String getName();
}
