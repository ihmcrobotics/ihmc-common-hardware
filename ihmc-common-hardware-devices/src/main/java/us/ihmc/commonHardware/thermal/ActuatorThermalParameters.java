package us.ihmc.commonHardware.thermal;

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
public interface ActuatorThermalParameters
{
   /**
    * Housing thermal capacitance C_h [J/K].
    * Thermal energy storage capacity of the motor housing/stator.
    */
   double getHousingThermalCapacitance();

   /**
    * Winding thermal capacitance C_w [J/K].
    * Thermal energy storage capacity of the copper windings.
    */
   double getWindingThermalCapacitance();

   /**
    * Winding electrical resistance at the reference temperature R_0 [Ω].
    */
   double getWindingResistanceAtReferenceTemperature();

   /**
    * Reference temperature T_0 [°C] at which {@link #getWindingResistanceAtReferenceTemperature()}
    * was measured.
    */
   double getReferenceTemperature();

   /**
    * Maximum allowable winding temperature T_max [°C].
    * The fuse curve gives the time-to-failure defined as when the winding reaches this temperature.
    */
   double getMaxWindingTemperature();

   /**
    * Housing-to-ambient thermal resistance R_ha [K/W].
    */
   double getHousingToAmbientThermalResistance();

   /**
    * Winding-to-housing thermal resistance R_wh [K/W].
    */
   double getWindingToHousingThermalResistance();
}
