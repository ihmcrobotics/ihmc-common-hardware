package us.ihmc.commonHardware.devices.etherCATDevices.elmo.motorControl;

import us.ihmc.commonHardware.devices.etherCATDevices.elmo.YoGenericTwitter;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TwitterOnBoardPositionControlState implements State
{       
   private static final double SMOOTH_IN_TIME = 1.0;
   private final TwitterControlDesireds controlDesireds;
   private final YoGenericTwitter actuator;

  
   public TwitterOnBoardPositionControlState(String name, TwitterControlDesireds controlDesireds, YoGenericTwitter actuator, YoRegistry registry)
   {
      
      this.controlDesireds = controlDesireds;
      this.actuator = actuator;
   }

   //just forward everything to the twitter.....
   //good place to put limits
   @Override
   public void onEntry()
   {
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
