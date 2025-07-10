package us.ihmc.commonHardware.devices.etherCATDevices.elmo.motorControl;

import us.ihmc.commonHardware.devices.etherCATDevices.elmo.YoGenericTwitter;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TwitterCurrentControlState implements State
{  
   private static final double SMOOTH_IN_TIME = 1.0;
   private final TwitterControlDesireds controlDesireds;
   private final YoGenericTwitter twitter;
   
   private final YoDouble smoothInTime;
   private final YoDouble smoothInAlpha;
   private final YoDouble unsmoothedFeedForwardCurrent;
   private final YoDouble unsmoothedDesiredTorque;
  
   public TwitterCurrentControlState(String name, TwitterControlDesireds controlDesireds, YoGenericTwitter actuator, YoRegistry registry)
   {
      String prefix = name + "_";
      
      smoothInTime = new YoDouble(prefix + "CurrentSmoothInTime", registry);
      smoothInAlpha = new YoDouble(prefix + "CurrentSmoothInAlpha", registry);
      unsmoothedFeedForwardCurrent = new YoDouble(prefix + "unsmoothedFeedForwardCurrent", registry);
      unsmoothedDesiredTorque = new YoDouble(prefix + "unsmoothedDesiredTorque", registry);
      
      smoothInTime.set(SMOOTH_IN_TIME);
      
      this.controlDesireds = controlDesireds;
      this.twitter = actuator;
   }
   

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      smoothInAlpha.set(MathTools.clamp(timeInState / smoothInTime.getDoubleValue(), 0.0, 1.0));
//      controlDesireds.setDesiredTwitterControlMode(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_TORQUE);
      
      //torque and ff current both contribute to the desired current
      unsmoothedFeedForwardCurrent.set(controlDesireds.getDesiredFeedForwardCurrent());
      unsmoothedDesiredTorque.set(controlDesireds.getDesiredMotorTorque());
      
      controlDesireds.setDesiredFeedForwardCurrent(smoothInAlpha.getDoubleValue() * unsmoothedFeedForwardCurrent.getDoubleValue());
      controlDesireds.setDesiredMotorTorque(smoothInAlpha.getDoubleValue() * unsmoothedDesiredTorque.getDoubleValue());
      
      //set actuals to desireds and set feedback limit to 0
      controlDesireds.setDesiredMotorPosition(twitter.getMeasuredMotorPosition());
      controlDesireds.setDesiredMotorVelocity(twitter.getMeasuredMotorVelocity());
   }

   @Override
   public void onExit(double timeInState)
   {      
      smoothInAlpha.setToNaN();
      unsmoothedFeedForwardCurrent.setToNaN();
      unsmoothedDesiredTorque.setToNaN();
   }
}
