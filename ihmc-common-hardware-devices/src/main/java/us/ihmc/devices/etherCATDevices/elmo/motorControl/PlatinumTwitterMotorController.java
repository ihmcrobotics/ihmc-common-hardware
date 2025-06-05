package us.ihmc.devices.etherCATDevices.elmo.motorControl;

import us.ihmc.devices.cycloids.CycloidPlatinumTwitter;
import us.ihmc.devices.cycloids.YoCycloidPlatinumTwitter;
import us.ihmc.devices.etherCATDevices.elmo.YoGenericTwitter;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.robotics.outputData.JointDesiredControlMode;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class PlatinumTwitterMotorController implements RobotController
{
   private static final String SIMPLE_NAME = PlatinumTwitterMotorController.class.getSimpleName();

   private static final boolean CREATE_FUNCTION_GENERATORS = false;

   private final YoRegistry registry;
   private final YoGenericTwitter twitter;
   
   private final TwitterControlDesireds desireds;

   private final YoEnum<TwitterControlMode> requestedControlMode;
   private final StateMachine<TwitterControlMode, State> stateMachine;

   public PlatinumTwitterMotorController(String name, YoGenericTwitter twitter, YoDouble yoTime, double dt, YoRegistry parentRegistry)
   {
      this.twitter = twitter;
      String prefix = name + "_";

      // registries
      registry = new YoRegistry(prefix + "Controller");
      desireds = new TwitterControlDesireds(prefix, registry);
      
      requestedControlMode = new YoEnum<>(prefix + "requestedControlMode", registry, TwitterControlMode.class, true);
      requestedControlMode.set(TwitterControlMode.POSITION);

      StateMachineFactory<TwitterControlMode, State> stateMachineFactory = new StateMachineFactory<>(TwitterControlMode.class);
      stateMachineFactory.setNamePrefix(prefix).setRegistry(registry).buildYoClock(yoTime);
      
      TwitterNoneControlState noneState = new TwitterNoneControlState(desireds, twitter);
      TwitterOnBoardPositionControlState positionControlState = new TwitterOnBoardPositionControlState(prefix, desireds, twitter, registry);
      TwitterCurrentControlState currentControlState = new TwitterCurrentControlState(prefix, desireds, twitter, registry);
      
      stateMachineFactory.addState(TwitterControlMode.NONE, noneState);
      stateMachineFactory.addState(TwitterControlMode.CURRENT, currentControlState);
      stateMachineFactory.addState(TwitterControlMode.POSITION, positionControlState);
      
      stateMachineFactory.addRequestedTransition(TwitterControlMode.NONE, TwitterControlMode.CURRENT, requestedControlMode);
      stateMachineFactory.addRequestedTransition(TwitterControlMode.NONE, TwitterControlMode.POSITION, requestedControlMode);
      
      stateMachineFactory.addRequestedTransition(TwitterControlMode.CURRENT, TwitterControlMode.NONE, requestedControlMode);
      stateMachineFactory.addRequestedTransition(TwitterControlMode.CURRENT, TwitterControlMode.POSITION, requestedControlMode);
      stateMachineFactory.addRequestedTransition(TwitterControlMode.POSITION, TwitterControlMode.NONE, requestedControlMode);
      stateMachineFactory.addRequestedTransition(TwitterControlMode.POSITION, TwitterControlMode.CURRENT, requestedControlMode);

      if (CREATE_FUNCTION_GENERATORS)
      {
         TwitterYoFunctionControlState yoFunctionControlState = new TwitterYoFunctionControlState(prefix, desireds, twitter, yoTime, dt, registry);
         stateMachineFactory.addState(TwitterControlMode.YOFUNCTION, yoFunctionControlState);
         stateMachineFactory.addRequestedTransition(TwitterControlMode.NONE, TwitterControlMode.YOFUNCTION, requestedControlMode);
         stateMachineFactory.addRequestedTransition(TwitterControlMode.CURRENT, TwitterControlMode.YOFUNCTION, requestedControlMode);
         stateMachineFactory.addRequestedTransition(TwitterControlMode.POSITION, TwitterControlMode.YOFUNCTION, requestedControlMode);
         stateMachineFactory.addRequestedTransition(TwitterControlMode.YOFUNCTION, TwitterControlMode.NONE, requestedControlMode);
      }
      
      stateMachine = stateMachineFactory.build(TwitterControlMode.NONE);
      
      parentRegistry.addChild(registry);
   }

   public void read()
   {
      twitter.read();
   }
   
   @Override
   public void doControl()
   {
      stateMachine.doTransitions();
      stateMachine.doAction();

      // sets the desired position of the motor, in rad
      twitter.setDesiredMotorPosition(desireds.getDesiredMotorPosition());
      
      // sets the desired velocity of the motor, in rad/s
      twitter.setDesiredMotorVelocity(desireds.getDesiredMotorVelocity());
      
      // sets a desired feedforward current
      twitter.setDesiredFeedForwardCurrent(desireds.getDesiredFeedForwardCurrent());
      
      // sets the desired torque
      twitter.setDesiredMotorTorque(desireds.getDesiredMotorTorque());
      
      // sets the motor stiffness
      twitter.setDesiredMotorStiffness(desireds.getDesiredMotorStiffness());
      
      // sets the motor damping
      twitter.setDesiredMotorDamping(desireds.getDesiredMotorDamping());
      
      // sets the max position feedback error
      twitter.setMaxPositionFeedbackError(desireds.getMaxPositionFeedbackError());
      
      // sets the max velocity feedback error
      twitter.setMaxVelocityFeedbackError(desireds.getMaxVelocityFeedbackError());
      
   }

//   public void setDesiredControlMode(JointDesiredControlMode controlMode)
//   {
//      switch (controlMode)
//      {
//         case POSITION -> twi
//      }
//   }
   
   public void write()
   {
      twitter.write();
   }

   public void setRequestedControlMode(TwitterControlMode mode)
   {
      requestedControlMode.set(mode);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return SIMPLE_NAME;
   }

   @Override
   public String getDescription()
   {
      return "twitter controller";
   }

   public Slave.State getEtherCATState()
   {
      return twitter.getEtherCATState();
   }

   public double getMeasuredMotorPosition()
   {
      return twitter.getMeasuredMotorPosition();
   }
   
   public double getMeasuredOutputPosition()
   {
      return twitter.getMeasuredOutputPosition();
   }
   
   public double getMeasuredMotorVelocity()
   {
      return twitter.getMeasuredMotorVelocity();
   }

   public double getMeasuredMotorTorque()
   {
      return twitter.getMeasuredMotorTorque();
   }
   
   public double getKt()
   {
      return twitter.getKt();
   }

   public boolean isMotorFaulted()
   {
      return twitter.isMotorFaulted();
   }

   /**
    * Sets the desired motor position in rads for this actuator
    */
   public void setDesiredMotorPosition(double position)
   {
      desireds.setDesiredMotorPosition(position);
   }

   /**
    * Sets the desired motor velocity in rads / s for this actuator
    */
   public void setDesiredMotorVelocity(double velocity)
   {
      desireds.setDesiredMotorVelocity(velocity);
   }

   /**
    * Sets the desired motor torque in Nm for this actuator
    */
   public void setDesiredMotorTorque(double torque)
   {
      desireds.setDesiredMotorTorque(torque);
   }

   /**
    * Sets whether or not the motor current is enabled.
    */
   public void enableDrive(boolean enableDrive)
   {
      twitter.enableDrive(enableDrive);
   }

   public double getMeasuredOutputVelocity()
   {
      return twitter.getMeasuredOutputVelocity();
   }

   public double getFilteredOutputVelocity()
   {
      return twitter.getFilteredOutputVelocity();
   }

   public double getMeasuredOutputTorque()
   {
      return twitter.getMeasuredOutputTorque();
   }

   public double getGearRatio()
   {
      return twitter.getGearRatio();
   }

   public void setMotorStiffness(double stiffness)
   {
      desireds.setDesiredMotorStiffness(stiffness);
   }

   public void setMotorDamping(double damping)
   {
      desireds.setDesiredMotorDamping(damping);
   }
   
   public void setMaxPositionFeedbackError(double maxFeedbackError)
   {
      desireds.setMaxPositionFeedbackError(maxFeedbackError);
   }

   public void setMaxVelocityFeedbackError(double maxFeedbackError)
   {
      desireds.setMaxVelocityFeedbackError(maxFeedbackError);
   }

   public void setDesiredMotorAcceleration(double acceleration)
   {
      //TODO: implement
//      desireds.setDesiredMotorAcceleration(acceleration);
   }

   public double getStatorTemperature()
   {
      return ((YoCycloidPlatinumTwitter)twitter).getStatorTemperature();
   }

   public int getMaximumAllowableStatorTemperature()
   {
      return ((YoCycloidPlatinumTwitter)twitter).getMaxAllowableStatorTemperature();
   }
   public int getMaximumRecommendedStatorTemperature()
   {
      return ((YoCycloidPlatinumTwitter)twitter).getMaxRecommendedStatorTemperature();
   }

}
