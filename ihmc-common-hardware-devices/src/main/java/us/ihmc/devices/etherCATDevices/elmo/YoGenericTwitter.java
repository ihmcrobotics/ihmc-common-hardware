package us.ihmc.devices.etherCATDevices.elmo;

import us.ihmc.etherCAT.master.Slave;

public interface YoGenericTwitter
{
   void setDesiredMotorPosition(double motorPosition);

   void setDesiredMotorVelocity(double motorVelocity);

   void setDesiredFeedForwardCurrent(double desiredFeedForwardCurrent);

   void setDesiredMotorTorque(double desiredMotorTorque);

   void setDesiredOutputTorque(double desiredTorque);

   void enableDrive(boolean enableDrive);

   void read();

   void write();

   double getMeasuredMotorPosition();

   double getMeasuredOutputPosition();

   double getMeasuredMotorVelocity();

   double getMeasuredMotorTorque();

   double getFilteredMotorVelocity();

   double getFilteredOutputVelocity();

   Slave.State getEtherCATState();

   double getMeasuredOutputVelocity();

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