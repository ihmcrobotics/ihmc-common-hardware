package us.ihmc.commonHardware.devices.etherCATDevices.elmo.motorControl;

import us.ihmc.commonHardware.devices.etherCATDevices.elmo.YoGenericTwitter;
import us.ihmc.robotics.stateMachine.core.State;

public class TwitterNoneControlState implements State
{
   private final TwitterControlDesireds controlDesireds;
   private final YoGenericTwitter twitter;

   public TwitterNoneControlState(TwitterControlDesireds controlDesireds, YoGenericTwitter twitter)
   {
      this.controlDesireds = controlDesireds;
      this.twitter = twitter;
   }

   @Override
   public void onEntry()
   {
//      controlDesireds.setDesiredTwitterControlMode(ElmoModeOfOperation.NO_MODE);
      controlDesireds.setDesiredMotorPosition(twitter.getMeasuredMotorPosition());
      controlDesireds.setDesiredMotorVelocity(twitter.getMeasuredMotorVelocity());
      controlDesireds.setDesiredFeedForwardCurrent(0.0);
      controlDesireds.setDesiredMotorTorque(0.0);
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public void onExit(double timeInState)
   {
   }
}
