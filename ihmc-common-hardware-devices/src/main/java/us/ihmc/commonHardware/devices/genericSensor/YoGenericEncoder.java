package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.commonHardware.devices.EncoderInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksEncoder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class YoGenericEncoder implements YoSensorInterface
{
   private final YoRegistry registry;
   private final EncoderInterface input;
   private final YoDouble previousPosition;
   private final YoDouble position;
   private final YoLong positionRaw;
   private final YoLong previousPositionRaw;
   public final YoDouble encoderOffset;
   private final YoBoolean invertDirection;

   private YoDouble alphaFilter;
   private AlphaFilteredYoVariable filteredPosition;
   private final YoDouble velocity;

   private final double CONTROLLER_DT;

   private boolean previousPositionRawBeenSet = false;

   /**
    * * YoWrapper class for the encoder component of an Ethersnacks board. Reads sensor measurements
    * and updates corresponding yovariables
    *
    * @param prefix          the name prefix for named yovariables in this class
    * @param input           the daughter board object the load cell is located on
    * @param invertDirection whether or not to negate the position and velocity values sent by the encoder
    * @param controllerDt    the period of the controller
    * @param parentRegistry  initial parent registry for this object
    */
   public YoGenericEncoder(String prefix, EncoderInterface input, boolean invertDirection, double controllerDt, YoRegistry parentRegistry)
   {
      this.input = input;
      registry = new YoRegistry("JointEncoder_" + prefix);

      position = new YoDouble(prefix + "position", registry);
      positionRaw = new YoLong(prefix + "positionRaw", registry);
      encoderOffset = new YoDouble(prefix + "encoderOffset", registry);
      previousPosition = new YoDouble(prefix + "previousPosition", registry);
      previousPositionRaw = new YoLong(prefix + "previousPositionRaw", registry);
      velocity = new YoDouble(prefix + "velocity", registry);

      this.alphaFilter = new YoDouble(prefix + "_alphaFilter", registry);
      //alphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1000.0, controllerDt));
      alphaFilter.set(0.8);
      this.filteredPosition = new AlphaFilteredYoVariable(prefix + "filtered", registry, alphaFilter, position);

      this.invertDirection = new YoBoolean(prefix + "invertDirection", registry);
      this.invertDirection.set(invertDirection);

      this.CONTROLLER_DT = controllerDt;

      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {
      previousPositionRaw.set(positionRaw.getLongValue());
      positionRaw.set(input.getRawPosition());

      if (!((previousPositionRaw.getLongValue() < 50) && (positionRaw.getLongValue() > 500) && previousPositionRawBeenSet))
      {
         if (!invertDirection.getBooleanValue())
         {
            position.set(input.getPosition() + encoderOffset.getValueAsDouble());
         }
         else
         {
            position.set(-input.getPosition() + encoderOffset.getValueAsDouble());
         }
      }

      position.set(AngleTools.trimAngleMinusPiToPi(position.getDoubleValue()));

      filteredPosition.update();

      velocity.set((filteredPosition.getDoubleValue() - previousPosition.getDoubleValue()) / CONTROLLER_DT);
      previousPosition.set(filteredPosition.getDoubleValue());

      previousPositionRawBeenSet = true;
   }

   /**
    * @return the current position of the encoder
    */
   public double getPosition()
   {
      return filteredPosition.getDoubleValue();
   }

   /**
    * @return the current velocity of the encoder
    */
   public double getVelocity()
   {
      return velocity.getDoubleValue();
   }

   //TODO: ADD DATA VALID STATUS TO PDO ON DAUGHTER BOARD

   /**
    * @return true if the encoder data is valid, false otherwise
    */
   public boolean isDataValid()
   {
      return true;
   }

   /**
    * adds a position bias to the encoder readings
    *
    * @param offset bias to add to subsequent position measurements
    */
   public void setEncoderOffset(double offset)
   {
      encoderOffset.set(offset);
   }

   /**
    * sets the offset on the encoder such that the current position is zero
    */
   public void zeroEncoder()
   {
      encoderOffset.set(-filteredPosition.getDoubleValue());
   }
}
