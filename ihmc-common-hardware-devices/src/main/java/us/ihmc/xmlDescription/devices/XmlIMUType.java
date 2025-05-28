package us.ihmc.xmlDescription.devices;

import jakarta.xml.bind.annotation.XmlEnum;
import jakarta.xml.bind.annotation.XmlType;

@XmlType(name = "XmlIMUType")
@XmlEnum
public enum XmlIMUType
{
   ORIENTUS, GABLE_MTS, BOTA, STIM_318, H4;
   
   public String value()
   {
      return name();
   }
   
   public static XmlIMUType fromValue(String v)
   {
      return valueOf(v);
   }
}
