package us.ihmc.commonHardware.xmlDescription.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlIMU")
public class XmlIMU extends AbstractXmlEtherCATDevice
{
   @XmlElement(required = true)
   protected XmlIMUType imuType;

   @XmlElement(defaultValue = "9.81")
   protected double gravity;

   @XmlElement(defaultValue = "0.0")
   protected double linearBiasX;
   @XmlElement(defaultValue = "0.0")
   protected double linearBiasY;
   @XmlElement(defaultValue = "0.0")
   protected double linearBiasZ;

   @XmlElement(defaultValue = "0.0")
   protected double angularBiasX;
   @XmlElement(defaultValue = "0.0")
   protected double angularBiasY;
   @XmlElement(defaultValue = "0.0")
   protected double angularBiasZ;

   public XmlIMUType getIMUType()
   {
      return imuType;
   }

   public void setIMUType(XmlIMUType type)
   {
      this.imuType = type;
   }

   public double getGravity()
   {
      return gravity;
   }

   public void setGravity(double gravity)
   {
      this.gravity = gravity;
   }

   public double getLinearBiasX()
   {
      return linearBiasX;
   }

   public void setLinearBiasX(double linearBiasX)
   {
      this.linearBiasX = linearBiasX;
   }

   public double getLinearBiasY()
   {
      return linearBiasY;
   }

   public void setLinearBiasY(double linearBiasY)
   {
      this.linearBiasY = linearBiasY;
   }

   public double getLinearBiasZ()
   {
      return linearBiasZ;
   }

   public void setLinearBiasZ(double linearBiasZ)
   {
      this.linearBiasZ = linearBiasZ;
   }

   public double getAngularBiasX()
   {
      return angularBiasX;
   }

   public void setAngularBiasX(double angularBiasX)
   {
      this.angularBiasX = angularBiasX;
   }

   public double getAngularBiasY()
   {
      return angularBiasY;
   }

   public void setAngularBiasY(double angularBiasY)
   {
      this.angularBiasY = angularBiasY;
   }

   public double getAngularBiasZ()
   {
      return angularBiasZ;
   }

   public void setAngularBiasZ(double angularBiasZ)
   {
      this.angularBiasZ = angularBiasZ;
   }

   public void setPresent(boolean present)
   {
      this.present = present;
   }
}
