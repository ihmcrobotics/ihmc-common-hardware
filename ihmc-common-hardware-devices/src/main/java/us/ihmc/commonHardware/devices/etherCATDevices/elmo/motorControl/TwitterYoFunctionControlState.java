package us.ihmc.commonHardware.devices.etherCATDevices.elmo.motorControl;

import us.ihmc.commonHardware.devices.etherCATDevices.elmo.YoGenericTwitter;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class TwitterYoFunctionControlState implements State
{       
   private enum FunctionGenOutputMode {ONBOARD_POSITION, OFFBOARD_POSITION, TORQUE, CURRENT};
   private enum VelocitySource {MOTOR_ENCODER, OUTPUT_ENCODER, FUSED, CONDITIONAL};

   private final TwitterControlDesireds controlDesireds;

   private final YoFunctionGenerator yoFunctionGenerator;
   private final YoEnum<FunctionGenOutputMode> functionGenOutputMode;
   private final YoEnum<VelocitySource> velocitySource;
   private final YoDouble velocityFuseAlpha;
   private final YoDouble velocityErrorThreshold;
   private final YoDouble measuredVelocity;
   
   private final PIDController positionController;
   private final YoGenericTwitter actuator;
   private final double DT;

   public TwitterYoFunctionControlState(String name, TwitterControlDesireds controlDesireds, YoGenericTwitter actuator, YoDouble yoTime, double dt, YoRegistry registry)
   {
      this.actuator = actuator;
      this.DT = dt;
      String prefix = name + "_";
      
      yoFunctionGenerator = new YoFunctionGenerator(prefix + "YoFunc", yoTime, registry);
      functionGenOutputMode = new YoEnum<FunctionGenOutputMode>(prefix + "functionGenOutputMode", registry, FunctionGenOutputMode.class);
      velocitySource = new YoEnum<VelocitySource>(prefix + "velocitySource", registry, VelocitySource.class);
      velocityFuseAlpha = new YoDouble(prefix + "velocityFuseAlpha", registry);
      velocityFuseAlpha.set(1.0);

      velocityErrorThreshold = new YoDouble(prefix + "velocityErrorThreshold", registry);
      velocityErrorThreshold.set(0.0);
      
      measuredVelocity = new YoDouble(prefix + "measuredVelocity", registry);
      
      this.controlDesireds = controlDesireds;

      YoRegistry pdControllerRegistry = new YoRegistry(prefix);
      positionController = new PIDController(prefix, pdControllerRegistry);
      positionController.setMaxIntegralError(0.1);
      positionController.setIntegralLeakRatio(0.99);
      registry.addChild(pdControllerRegistry);
   }

   @Override
   public void onEntry()
   {
      yoFunctionGenerator.setMode(YoFunctionGeneratorMode.SINE);
      yoFunctionGenerator.setAmplitude(0.0);
      yoFunctionGenerator.setFrequency(0.1);
      yoFunctionGenerator.setOffset(0.0);
      yoFunctionGenerator.setAlphaForSmoothing(0.9);
      functionGenOutputMode.set(FunctionGenOutputMode.CURRENT);
   }

   @Override
   public void doAction(double timeInState)
   {
      
      double value = yoFunctionGenerator.getValue();
      double valueDot = yoFunctionGenerator.getValueDot();
      
      switch(functionGenOutputMode.getEnumValue())
      {
         case ONBOARD_POSITION:
            /** Function generator output is in rad **/
            controlDesireds.setDesiredMotorTorque(0.0);
            controlDesireds.setDesiredMotorPosition(value);
            controlDesireds.setDesiredMotorVelocity(valueDot);
            
//            controlDesireds.setDesiredTwitterControlMode(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_POSITION);
            controlDesireds.setDesiredFeedForwardCurrent(0.0);
            break;

         case OFFBOARD_POSITION:
            /** Function generator output is in rad **/
            controlDesireds.setDesiredMotorPosition(value);
            controlDesireds.setDesiredMotorVelocity(valueDot);
            
			double filteredMotorVelocity = actuator.getFilteredMotorVelocity();
			double filteredOutputVelocity = actuator.getFilteredOutputVelocity() * 6.0;
			 
			switch(velocitySource.getEnumValue())
			{
			case MOTOR_ENCODER:
				measuredVelocity.set(filteredMotorVelocity);
				break;
			case OUTPUT_ENCODER:
				measuredVelocity.set(filteredOutputVelocity);
				break;
			case CONDITIONAL:
				 measuredVelocity.set(filteredMotorVelocity);
				if(Math.abs(filteredOutputVelocity) > velocityErrorThreshold.getDoubleValue())
				{
					measuredVelocity.set(filteredOutputVelocity);
				}
				break;
			case FUSED:
				double vel = (velocityFuseAlpha.getDoubleValue() * filteredMotorVelocity) + ((1.0 - velocityFuseAlpha.getDoubleValue()) * filteredOutputVelocity);
				measuredVelocity.set(vel);
			}
		
			double tauDesired = positionController.compute(actuator.getMeasuredMotorPosition(), value, measuredVelocity.getDoubleValue(), valueDot, DT);
            controlDesireds.setDesiredMotorTorque(tauDesired);
            
//            controlDesireds.setDesiredTwitterControlMode(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_POSITION);
            controlDesireds.setDesiredFeedForwardCurrent(0.0);
            break;
         
         case TORQUE:
            /** Function generator output is in Nm **/
            controlDesireds.setDesiredMotorTorque(value);
            controlDesireds.setDesiredMotorPosition(actuator.getMeasuredMotorPosition());
            controlDesireds.setDesiredMotorVelocity(0.0);
            
//            controlDesireds.setDesiredTwitterControlMode(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_TORQUE);
            controlDesireds.setDesiredFeedForwardCurrent(0.0);
            break;

         case CURRENT:
            /** Function generator output is in amps **/
            controlDesireds.setDesiredMotorTorque(0.0);
            controlDesireds.setDesiredMotorPosition(actuator.getMeasuredMotorPosition());
            controlDesireds.setDesiredMotorVelocity(0.0);
            
//            controlDesireds.setDesiredTwitterControlMode(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_TORQUE);
            controlDesireds.setDesiredFeedForwardCurrent(value);
            break;
      }
   }
   
   @Override
   public void onExit(double timeInState)
   {    
   }
}
