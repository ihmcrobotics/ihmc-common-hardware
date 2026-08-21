package us.ihmc.hardwareXMLToolkit.devices.parameters;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlRootElement(name = "MotorParameters")
public class XmlMotorParameters
{
   // Motor manufacturer and model
   @XmlElement(required = true)
   private String manufacturer;
   @XmlElement(required = true)
   private String model;

   // Electrical Parms
   @XmlElement(defaultValue = "0.0")
   private double kt;
   @XmlElement(defaultValue = "0.0")
   private double resistanceLineToLine;
   @XmlElement(defaultValue = "0.0")
   private double busVoltage;
   @XmlElement(defaultValue = "0.0")
   private double maxCurrentContinuous;
   @XmlElement(defaultValue = "0.0")
   private double maxCurrentPeak;

   // Mechanical Params
   @XmlElement(defaultValue = "0.0")
   private double maxTorqueContinuous;
   @XmlElement(defaultValue = "0.0")
   private double maxTorquePeak;
   @XmlElement(defaultValue = "0.0")
   private double maxSpeed;
   @XmlElement(defaultValue = "0.0")
   private double polePairs;
   @XmlElement(defaultValue = "0.0")
   private double mass;

   // Thermal params
   @XmlElement(defaultValue = "null")
   private XmlMotorThermalParameters motorThermalParameters;

   public String getManufacturer()
   {
      return manufacturer;
   }

   public String getModel()
   {
      return model;
   }

   public double getKt()
   {
      return kt;
   }

   public double getResistanceLineToLine()
   {
      return resistanceLineToLine;
   }

   public double getBusVoltage()
   {
      return busVoltage;
   }

   public double getMaxCurrentContinuous()
   {
      return maxCurrentContinuous;
   }

   public double getMaxCurrentPeak()
   {
      return maxCurrentPeak;
   }

   public double getMaxTorqueContinuous()
   {
      return maxTorqueContinuous;
   }

   public double getMaxTorquePeak()
   {
      return maxTorquePeak;
   }

   public double getMaxSpeed()
   {
      return maxSpeed;
   }

   public double getPolePairs()
   {
      return polePairs;
   }

   public double getMass()
   {
      return mass;
   }

   public XmlMotorThermalParameters getMotorThermalParameters()
   {
      return motorThermalParameters;
   }
}
