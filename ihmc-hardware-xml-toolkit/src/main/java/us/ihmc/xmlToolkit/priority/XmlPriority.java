package us.ihmc.xmlToolkit.priority;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlPriority")
public class XmlPriority
{
   @XmlElement(required = true)
   protected int hardwareThreadCPUAffinity;
   @XmlElement(required = true)
   protected int hardwareThreadPriority;
   @XmlElement(required = true)
   protected int controlThreadCPUAffinity;
   @XmlElement(required = true)
   protected int controlThreadPriority;
   @XmlElement(required = false, defaultValue = "40")
   protected int handsThreadPriority;
   @XmlElement(required = true)
   protected int loggerThreadPriority;

   public int getHardwareThreadCPUAffinity()
   {
      return hardwareThreadCPUAffinity;
   }

   public void setHardwareThreadCPUAffinity(int hardwareThreadCPUAffinity)
   {
      this.hardwareThreadCPUAffinity = hardwareThreadCPUAffinity;
   }

   public int getHardwareThreadPriority()
   {
      return hardwareThreadPriority;
   }

   public void setHardwareThreadPriority(int hardwareThreadPriority)
   {
      this.hardwareThreadPriority = hardwareThreadPriority;
   }

   public int getControlThreadCPUAffinity()
   {
      return controlThreadCPUAffinity;
   }

   public void setControlThreadCPUAffinity(int controlThreadCPUAffinity)
   {
      this.controlThreadCPUAffinity = controlThreadCPUAffinity;
   }

   public int getControlThreadPriority()
   {
      return controlThreadPriority;
   }

   public void setControlThreadPriority(int controlThreadPriority)
   {
      this.controlThreadPriority = controlThreadPriority;
   }

   public int getLoggerThreadPriority()
   {
      return loggerThreadPriority;
   }

   public void setLoggerThreadPriority(int loggerThreadPriority)
   {
      this.loggerThreadPriority = loggerThreadPriority;
   }

   public int getHandsThreadPriority()
   {
      return handsThreadPriority;
   }

   public void setHandsThreadPriority(int handsThreadPriority)
   {
      this.handsThreadPriority = handsThreadPriority;
   }
   
   

}
