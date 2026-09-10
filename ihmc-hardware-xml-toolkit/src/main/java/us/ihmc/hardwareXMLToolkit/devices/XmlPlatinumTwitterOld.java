package us.ihmc.hardwareXMLToolkit.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlPlatinumTwitter")
public class XmlPlatinumTwitterOld extends AbstractXmlEtherCATDevice
{
   @XmlElement(required = true)
   protected String actuatorPackage;
   @XmlElement(defaultValue = "false")
   protected boolean useLatestCode;
   @XmlElement(required = true)
   protected boolean reversePositiveMotorDirection;
   @XmlElement(required = true)
   protected int inputOffset;
   @XmlElement(required = true)
   protected int outputOffset;

   public XmlPlatinumTwitterOld()
   {

   }

   public XmlPlatinumTwitterOld(String name, int alias, int position, String actuatorPackage)
   {
      this.name = name;
      this.alias = alias;
      this.position = position;
      this.useLatestCode = false;
      this.present = true;
      this.actuatorPackage = actuatorPackage;
   }

   public String getActuatorPackage()
   {
      return actuatorPackage;
   }

   public boolean useLatestCode()
   {
      return useLatestCode;
   }

   public int getInputOffset()
   {
      return inputOffset;
   }

   public int getOutputOffset()
   {
      return outputOffset;
   }

   public boolean isMotorDirectionReversed()
   {
      return reversePositiveMotorDirection;
   }

   public void setReversePositiveMotorDirection(boolean reversePositiveMotorDirection)
   {
      this.reversePositiveMotorDirection = reversePositiveMotorDirection;
   }

   public void setInputOffset(int inputOffset)
   {
      this.inputOffset = inputOffset;
   }

   public void setOutputOffset(int outputOffset)
   {
      this.outputOffset = outputOffset;
   }
}