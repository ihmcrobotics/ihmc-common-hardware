package us.ihmc.hardwareXMLToolkit.devices.parameters;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlRootElement(name = "MotorThermalParameters")
public class XmlMotorThermalParameters
{
   // Thermal params
   @XmlElement(defaultValue = "0.0")
   private double housingThermalCapacitance; // J/K
   @XmlElement(defaultValue = "0.0")
   private double windingThermalCapacitance; // J/K
   @XmlElement(defaultValue = "0.0")
   private double windingResistanceAtReferenceTemperature; // Ω
   @XmlElement(defaultValue = "0.0")
   private double referenceTemperature; // °C
   @XmlElement(defaultValue = "0.0")
   private double maxWindingTemperature; // °C
   @XmlElement(defaultValue = "0.0")
   private double housingToAmbientThermalResistance; // K/W
   @XmlElement(defaultValue = "0.0")
   private double windingToHousingThermalResistance; // K/W

   public double getHousingThermalCapacitance()
   {
      return housingThermalCapacitance;
   }

   public double getWindingThermalCapacitance()
   {
      return windingThermalCapacitance;
   }

   public double getWindingResistanceAtReferenceTemperature()
   {
      return windingResistanceAtReferenceTemperature;
   }

   public double getReferenceTemperature()
   {
      return referenceTemperature;
   }

   public double getMaxWindingTemperature()
   {
      return maxWindingTemperature;
   }

   public double getHousingToAmbientThermalResistance()
   {
      return housingToAmbientThermalResistance;
   }

   public double getWindingToHousingThermalResistance()
   {
      return windingToHousingThermalResistance;
   }
}
