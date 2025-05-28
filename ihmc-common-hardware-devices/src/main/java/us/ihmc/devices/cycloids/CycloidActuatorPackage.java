package us.ihmc.devices.cycloids;

/**
 * Holds the identifier for each cycloid package. Any time a new one is created, it needs to be added here, with the settings updated in SILParameters
 * and CycloidActuatorParameters
 */
public enum CycloidActuatorPackage
{
   A, B, C, D;

   public static final CycloidActuatorPackage[] values = CycloidActuatorPackage.values();
}
