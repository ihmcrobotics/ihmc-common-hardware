package us.ihmc.devices.cycloids;

public class CycloidActuatorParameters
{
   private int countsPerMotorRevolution;
   private int countsPerOutputRevolution;
   private double gearRatio;
   private double kt;

   public CycloidActuatorParameters()
   {
   }

   public CycloidActuatorParameters(int countsPerMotorRevolution, int countsPerOutputRevolution, double gearRatio, double kt)
   {
      setCountsPerMotorRevolution(countsPerMotorRevolution);
      setCountsPerOutputRevolution(countsPerOutputRevolution);
      setGearRatio(gearRatio);
      setKt(kt);
   }

   public int getCountsPerMotorRevolution()
   {
      return countsPerMotorRevolution;
   }

   public void setCountsPerMotorRevolution(int countsPerMotorRevolution)
   {
      this.countsPerMotorRevolution = countsPerMotorRevolution;
   }

   public int getCountsPerOutputRevolution()
   {
      return countsPerOutputRevolution;
   }

   public void setCountsPerOutputRevolution(int countsPerOutputRevolution)
   {
      this.countsPerOutputRevolution = countsPerOutputRevolution;
   }

   public double getGearRatio()
   {
      return gearRatio;
   }

   public void setGearRatio(double gearRatio)
   {
      this.gearRatio = gearRatio;
   }

   public double getKt()
   {
      return kt;
   }

   public void setKt(double kt)
   {
      this.kt = kt;
   }

   public static CycloidActuatorParameters createCycloidParameters(CycloidActuatorPackage actuatorPackage)
   {
      switch (actuatorPackage)
      {
         case A:
         case B:
         case C:
         case D:
            return createCycloidParameters();
         case A001: //ALIAS 7008
            return createA001CycloidParameters();
         case A002: //ALIAS 7007
            return createA002CycloidParameters();
         case A009: //ALIAS 5004
            return createA009CycloidParameters();
         case A013: //ALIAS 5001
            return createA013CycloidParameters();
         case A005: //ALIAS 5001
            return createA005CycloidParameters();
         case A011: //ALIAS 7004
            return createA011CycloidParameters();
         case A010: //ALIAS 7005
            return createA010CycloidParameters();
         case A004: //ALIAS 6004
            return createA004CycloidParameters();
         case A003: //ALIAS 6005
            return createA003CycloidParameters();
         case A012: //ALIAS 7002
            return createA012CycloidParameters();
         case A008: //ALIAS 6002
            return createA008CycloidParameters();
         case A006: //ALIAS 6008
            return createA006CycloidParameters();
         case A007: //ALIAS 6007
            return createA007CycloidParameters();
         default:
            throw new IllegalArgumentException("Unexpected actuator package for cycloid parameters: " + actuatorPackage);
      }
   }

   public static CycloidActuatorParameters createCycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(17.0);
      parameters.setKt(0.073);
      return parameters;
   }

   public static CycloidActuatorParameters createA001CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.2);
      return parameters;
   }

   public static CycloidActuatorParameters createA002CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.2);
      return parameters;
   }

   public static CycloidActuatorParameters createA009CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.253);
      return parameters;
   }

   public static CycloidActuatorParameters createA013CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.253);
      return parameters;
   }

   public static CycloidActuatorParameters createA005CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.253);
      return parameters;
   }

   public static CycloidActuatorParameters createA011CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.281);
      return parameters;
   }

   public static CycloidActuatorParameters createA010CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.281);
      return parameters;
   }

   public static CycloidActuatorParameters createA004CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.281);
      return parameters;
   }

   public static CycloidActuatorParameters createA003CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.281);
      return parameters;
   }

   public static CycloidActuatorParameters createA012CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.175);
      return parameters;
   }

   public static CycloidActuatorParameters createA008CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.155);
      return parameters;
   }

   public static CycloidActuatorParameters createA006CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.205);
      return parameters;
   }

   public static CycloidActuatorParameters createA007CycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(19.0);
      parameters.setKt(0.21);
      return parameters;
   }
}