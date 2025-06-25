package us.ihmc.commonHardware.xmlDescription.devices;

import jakarta.xml.bind.annotation.XmlElement;

public class XmlLoadCell extends AbstractXmlDevice
{
   @XmlElement(defaultValue = "0.002") //in volts not mV
   protected double nominalSensitivity;
   @XmlElement(defaultValue = "0.0")
   protected double zeroBalance;
   @XmlElement(defaultValue = "444.822")
   protected double nominalLoad;
   @XmlElement(defaultValue = "5.0")
   protected double excitationVoltage;

   public double getNominalSensitivity()
   {
      return nominalSensitivity;
   }

   public double getZeroBalance()
   {
      return zeroBalance;
   }

   public double getNominalLoad()
   {
      return nominalLoad;
   }

   public double getExcitationVoltage()
   {
      return excitationVoltage;
   }
}
