package us.ihmc.commonHardware.xmlDescription.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlPlatinumTwitter")
public class XmlPlatinumTwitter extends AbstractXmlEtherCATDevice
{
   @XmlElement(required = true)
   protected String actuatorPackage;
   @XmlElement(required = true)
   protected boolean reversePositiveMotorDirection;
   @XmlElement(required = true)
   protected int inputOffset;
   @XmlElement(required = true)
   protected int outputOffset;

   public XmlPlatinumTwitter()
   {

   }

   public XmlPlatinumTwitter(String name, int alias, int position, String actuatorPackage)
   {
      this.name = name;
      this.alias = alias;
      this.position = position;
      this.present = true;
      this.actuatorPackage = actuatorPackage;
   }

   public String getActuatorPackage()
   {
      return actuatorPackage;
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