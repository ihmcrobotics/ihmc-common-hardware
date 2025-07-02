package us.ihmc.commonHardware.devices.etherCATDevices.elmo;

import us.ihmc.etherCAT.slaves.DSP402Slave;
import us.ihmc.etherCAT.slaves.elmo.ElmoErrorCodes;

/**
 * Elmo twitter slave code. Requires firmware version 1.1.12.b01 or above
 * 
 * @author jesper
 */
public abstract class PlatinumTwitter extends DSP402Slave
{

   public enum TWITTER_PRODUCT_CODE
   {
      X00100002, X01100002, LATEST;

      public int getID()
      {
         switch (this)
         {
            case X00100002:
               return oldProductCode;
            case X01100002:
            case LATEST:
            default:
               return latestProductCode;
         }
      }

      public static TWITTER_PRODUCT_CODE getCode(int productCode)
      {
         if (productCode == 0x00100002)
         {
            return X00100002;
         }
         if (productCode == 0x01100002)
         {
            return X01100002;
         }
         throw new RuntimeException(productCode + " Does not exist. Please update the PlatinumTwitter product codes or check the supplied code for typos");
      }
   };

   static final int vendorID = 0x0000009a;
   static final int oldProductCode = 0x00100002;
   static final int latestProductCode = 0x01100002;

   //Status Register Event Locations
   private final int UNDER_VOLTAGE = 3;
   private final int OVER_VOLTAGE = 5;
   private final int STO_DISABLED = 7;
   private final int CURRENT_SHORT = 11;
   private final int OVER_TEMPERATURE = 13;

   public PlatinumTwitter(int alias, int position, TWITTER_PRODUCT_CODE productCode)
   {
      super(vendorID, productCode.getID(), alias, position);
   }

   public abstract int getElmoStatusRegister();

   public boolean isFaulted()
   {
      return isUnderVoltage() || isOverVoltage() || isSTODisabled() || isCurrentShorted() || isOverTemperature();
   }

   @Override
   public void doStateControl()
   {
      super.doStateControl();
   }

   private boolean getStatusValue(int maskValue)
   {
//      System.out.println(getElmoStatusRegister());

      //Bitwise AND to check bit-string for status events 
      return (getElmoStatusRegister() & 0xF) == maskValue;
   }

   //Status Register Events
   public boolean isUnderVoltage()
   {
      return getStatusValue(UNDER_VOLTAGE);
   }

   public boolean isOverVoltage()
   {
      return getStatusValue(OVER_VOLTAGE);
   }

   public boolean isSTODisabled()
   {
      return getStatusValue(STO_DISABLED);
   }

   public boolean isCurrentShorted()
   {
      return getStatusValue(CURRENT_SHORT);
   }

   public boolean isOverTemperature()
   {
      return getStatusValue(OVER_TEMPERATURE);
   }

   @Override
   protected boolean hasShutdown()
   {
      if (super.hasShutdown())
      {
         int error = readSDOInt(0x306A, 0x1);
         int temperature = readSDOUnsignedShort(0x22A3, 0x1);
         System.out.println(toString() + " Last elmo error code: " + error + ": " + ElmoErrorCodes.errorCodeToString(error) + ". Drive temperature: "
                            + temperature + "�C");
         return true;
      }
      return false;
   }

   @Override
   public final boolean supportsCA()
   {
      return false;
   }

   @Override
   protected boolean blockLRW()
   {
      return true;
   }
}