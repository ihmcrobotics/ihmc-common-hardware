package us.ihmc.commonHardware.thermal;

import us.ihmc.hardwareXMLToolkit.devices.parameters.XmlMotorThermalParameters;

/**
 * Thermal model parameters for a two-node (winding + housing) actuator thermal network.
 *
 * <p>The model consists of:
 * <ul>
 *   <li>A winding node with thermal capacitance {@link #getWindingThermalCapacitance()} and
 *       Joule heating from current flowing through the winding resistance.</li>
 *   <li>A housing node with thermal capacitance {@link #getHousingThermalCapacitance()}, coupled
 *       to the winding via {@link #getWindingToHousingThermalResistance()} and to ambient via
 *       {@link #getHousingToAmbientThermalResistance()}.</li>
 * </ul>
 *
 * <p>Winding resistance varies with temperature via the copper temperature coefficient:
 * {@code R(T) = R_0 * (1 + alpha_Cu * (T - T_0))}
 */
public class MotorThermalParameters
{
   private final double housingThermalCapacitance; // J/K
   private final double windingThermalCapacitance; // J/K
   private final double windingResistanceAtReferenceTemperature; // Ω
   private final double referenceTemperature; // °C
   private final double maxWindingTemperature; // °C
   private final double housingToAmbientThermalResistance; // K/W
   private final double windingToHousingThermalResistance;

   public MotorThermalParameters()
   {
      this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   public MotorThermalParameters(XmlMotorThermalParameters xmlMotorThermalParameters)
   {
      this(xmlMotorThermalParameters.getHousingThermalCapacitance(),
           xmlMotorThermalParameters.getWindingThermalCapacitance(),
           xmlMotorThermalParameters.getWindingResistanceAtReferenceTemperature(),
           xmlMotorThermalParameters.getReferenceTemperature(),
           xmlMotorThermalParameters.getMaxWindingTemperature(),
           xmlMotorThermalParameters.getHousingToAmbientThermalResistance(),
           xmlMotorThermalParameters.getWindingToHousingThermalResistance());
   }

   public MotorThermalParameters(double housingThermalCapacitance,
                                 double windingThermalCapacitance,
                                 double windingResistanceAtReferenceTemperature,
                                 double referenceTemperature,
                                 double maxWindingTemperature,
                                 double housingToAmbientThermalResistance,
                                 double windingToHousingThermalResistance)
   {
      this.housingThermalCapacitance = housingThermalCapacitance;
      this.windingThermalCapacitance = windingThermalCapacitance;
      this.windingResistanceAtReferenceTemperature = windingResistanceAtReferenceTemperature;
      this.referenceTemperature = referenceTemperature;
      this.maxWindingTemperature = maxWindingTemperature;
      this.housingToAmbientThermalResistance = housingToAmbientThermalResistance;
      this.windingToHousingThermalResistance = windingToHousingThermalResistance;
   }

   /**
    * Housing thermal capacitance C_h [J/K].
    * Thermal energy storage capacity of the motor housing/stator.
    */
   double getHousingThermalCapacitance()
   {
      return housingThermalCapacitance;
   }

   /**
    * Winding thermal capacitance C_w [J/K].
    * Thermal energy storage capacity of the copper windings.
    */
   double getWindingThermalCapacitance()
   {
      return windingThermalCapacitance;
   }

   /**
    * Winding electrical resistance at the reference temperature R_0 [Ω].
    */
   double getWindingResistanceAtReferenceTemperature()
   {
      return windingResistanceAtReferenceTemperature;
   }

   /**
    * Reference temperature T_0 [°C] at which {@link #getWindingResistanceAtReferenceTemperature()}
    * was measured.
    */
   double getReferenceTemperature()
   {
      return referenceTemperature;
   }

   /**
    * Maximum allowable winding temperature T_max [°C].
    * The fuse curve gives the time-to-failure defined as when the winding reaches this temperature.
    */
   double getMaxWindingTemperature()
   {
      return maxWindingTemperature;
   }

   /**
    * Housing-to-ambient thermal resistance R_ha [K/W].
    */
   double getHousingToAmbientThermalResistance()
   {
      return housingToAmbientThermalResistance;
   }

   /**
    * Winding-to-housing thermal resistance R_wh [K/W].
    */
   double getWindingToHousingThermalResistance()
   {
      return windingToHousingThermalResistance;
   }
}
