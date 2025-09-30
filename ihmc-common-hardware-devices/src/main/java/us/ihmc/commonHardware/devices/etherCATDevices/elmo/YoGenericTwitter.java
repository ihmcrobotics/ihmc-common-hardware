package us.ihmc.commonHardware.devices.etherCATDevices.elmo;

import us.ihmc.etherCAT.master.Slave;

/**
 * This interface provides a skeleton for a yovariable wrapper around any twitter for motor control purposes
 */
public interface YoGenericTwitter
{
   /**
    * Reads the current status of the twitter, including operational status, sensor data, and any other info
    */
   void read();

   /**
    * Writes the desired commands to the twitter
    */
   void write();

   /**
    * Clears all possible faults from the twitter
    */
   void clearFaults();

   /**
    * @param enableDrive If true, enable drive control, else disable drive control
    */
   void enableDrive(boolean enableDrive);

   void setDesiredMotorPosition(double motorPosition);

   void setDesiredMotorVelocity(double motorVelocity);

   void setDesiredFeedForwardCurrent(double desiredFeedForwardCurrent);

   void setDesiredMotorTorque(double desiredMotorTorque);

   void setDesiredOutputTorque(double desiredTorque);

   double getMeasuredMotorPosition();

   double getMeasuredOutputPosition();

   double getFilteredMotorPosition();

   double getFilteredOutputPosition();

   double getMeasuredMotorVelocity();

   double getMeasuredOutputVelocity();

   double getFilteredMotorVelocity();

   double getFilteredOutputVelocity();

   double getMeasuredMotorTorque();

   Slave.State getEtherCATState();

   double getMeasuredOutputTorque();

   default double getGearRatio()
   {
      return 1.0;
   }

   double getKt();

   boolean isMotorFaulted();

   void setDesiredMotorStiffness(double desiredMotorStiffness);

   void setDesiredMotorDamping(double desiredMotorDamping);

   void setMaxPositionFeedbackError(double maxPositionFeedbackError);

   void setMaxVelocityFeedbackError(double maxVelocityFeedbackError);
}