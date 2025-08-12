package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.commonHardware.devices.LoadCellInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoGenericLoadCell implements YoSensorInterface
{
   private final YoRegistry registry;

   private final LoadCellInterface loadCell;

   private final YoInteger rawVoltage;
   private final YoDouble voltage;
   private final YoDouble force;

   private final YoDouble yoNominalSensitivity;
   private final YoDouble yoZeroBalance;
   private final YoDouble yoNominalLoad;
   private final YoDouble yoExcitationVoltage;

   /**
    * Construct the yovariable wrapper for a generic load cell
    *
    * @param prefix             Prefix used for all yovariable names
    * @param loadCell           Load cell to be yovariable-ized
    * @param nominalSensitivity Nominal sensitivity of the load cell
    * @param zeroBalance        Zero balance of the load cell
    * @param nominalLoad        Nominal load of the load cell
    * @param excitationVoltage  Excitation voltage of the load cell
    * @param parentRegistry     Parent {@code YoRegistry} of the load cell
    */
   public YoGenericLoadCell(String prefix,
                            LoadCellInterface loadCell,
                            double nominalSensitivity,
                            double zeroBalance,
                            double nominalLoad,
                            double excitationVoltage,
                            YoRegistry parentRegistry)
   {
      this.loadCell = loadCell;
      registry = new YoRegistry(prefix + getClass().getSimpleName());

      yoNominalSensitivity = new YoDouble(prefix + "NominalSensitivity", registry);
      yoZeroBalance = new YoDouble(prefix + "ZeroBalance", registry);
      yoNominalLoad = new YoDouble(prefix + "NominalLoad", registry);
      yoExcitationVoltage = new YoDouble(prefix + "ExcitationVoltage", registry);

      yoNominalSensitivity.set(nominalSensitivity);
      yoZeroBalance.set(zeroBalance);
      yoNominalLoad.set(nominalLoad);
      yoExcitationVoltage.set(excitationVoltage);

      voltage = new YoDouble(prefix + "Voltage", registry);
      rawVoltage = new YoInteger(prefix + "RawVoltage", registry);
      force = new YoDouble(prefix + "Force", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {
      rawVoltage.set(loadCell.getRawVoltage());
      voltage.set(loadCell.getVoltage());

      double calculatedForce =
            -yoNominalLoad.getDoubleValue() * voltage.getDoubleValue() / (yoExcitationVoltage.getDoubleValue() * yoNominalSensitivity.getDoubleValue() * 0.001)
            + yoZeroBalance.getDoubleValue();
      force.set(calculatedForce);
   }

   public double getForce()
   {
      return force.getDoubleValue();
   }
}
