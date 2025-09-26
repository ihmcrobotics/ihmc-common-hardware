package us.ihmc.hardwareXMLToolkit.devices;

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
   @XmlElement(defaultValue = "false")
   protected boolean useLatestCode;
   @XmlElement(required = true)
   protected boolean reversePositiveMotorDirection;
   @XmlElement(required = true)
   protected double motorOffset;
   @XmlElement(required = true)
   protected double outputOffset;

   public XmlPlatinumTwitter()
   {

   }

   public XmlPlatinumTwitter(String name, int alias, int position, String actuatorPackage)
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

   public double getMotorOffset()
   {
      return motorOffset;
   }

   public double getOutputOffset()
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

   public void setMotorOffset(int motorOffset)
   {
      this.motorOffset = motorOffset;
   }

   public void setOutputOffset(int outputOffset)
   {
      this.outputOffset = outputOffset;
   }
}