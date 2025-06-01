package us.ihmc.devices.cycloids;

/**
 * Holds the identifier for each cycloid package. Any time a new one is created, it needs to be added here, with the settings updated in SILParameters
 * and CycloidActuatorParameters
 */
public enum CycloidActuatorPackage
{
   A, B, C, D,
   A001,
   A002,
   A009,
   A013,
   A005,
   A011,
   A010,
   A004,
   A003,
   A012,
   A008,
   A006,
   A007;

   public static final CycloidActuatorPackage[] values = CycloidActuatorPackage.values();
}
