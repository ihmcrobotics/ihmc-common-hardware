package us.ihmc.hardwareXMLToolkit.devices;

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
   protected double linearAccelerationBiasX;
   @XmlElement(defaultValue = "0.0")
   protected double linearAccelerationBiasY;
   @XmlElement(defaultValue = "0.0")
   protected double linearAccelerationBiasZ;

   @XmlElement(defaultValue = "0.0")
   protected double angularVelocityBiasX;
   @XmlElement(defaultValue = "0.0")
   protected double angularVelocityBiasY;
   @XmlElement(defaultValue = "0.0")
   protected double angularVelocityBiasZ;

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

   public double getLinearAccelerationBiasX()
   {
      return linearAccelerationBiasX;
   }

   public void setLinearAccelerationBiasX(double linearAccelerationBiasX)
   {
      this.linearAccelerationBiasX = linearAccelerationBiasX;
   }

   public double getLinearAccelerationBiasY()
   {
      return linearAccelerationBiasY;
   }

   public void setLinearAccelerationBiasY(double linearAccelerationBiasY)
   {
      this.linearAccelerationBiasY = linearAccelerationBiasY;
   }

   public double getLinearAccelerationBiasZ()
   {
      return linearAccelerationBiasZ;
   }

   public void setLinearAccelerationBiasZ(double linearAccelerationBiasZ)
   {
      this.linearAccelerationBiasZ = linearAccelerationBiasZ;
   }

   public double getAngularVelocityBiasX()
   {
      return angularVelocityBiasX;
   }

   public void setAngularVelocityBiasX(double angularVelocityBiasX)
   {
      this.angularVelocityBiasX = angularVelocityBiasX;
   }

   public double getAngularVelocityBiasY()
   {
      return angularVelocityBiasY;
   }

   public void setAngularVelocityBiasY(double angularVelocityBiasY)
   {
      this.angularVelocityBiasY = angularVelocityBiasY;
   }

   public double getAngularVelocityBiasZ()
   {
      return angularVelocityBiasZ;
   }

   public void setAngularVelocityBiasZ(double angularVelocityBiasZ)
   {
      this.angularVelocityBiasZ = angularVelocityBiasZ;
   }

   public void setPresent(boolean present)
   {
      this.present = present;
   }
}
