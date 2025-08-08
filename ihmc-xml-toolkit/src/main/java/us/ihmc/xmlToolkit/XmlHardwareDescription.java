package us.ihmc.xmlToolkit;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import us.ihmc.xmlToolkit.devices.XmlDevices;
import us.ihmc.xmlToolkit.joints.XmlJoints;
import us.ihmc.xmlToolkit.priority.XmlPriority;
import us.ihmc.xmlToolkit.settings.XmlSettings;
import us.ihmc.xmlToolkit.transmissions.XmlTransmissions;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlRootElement(name = "XmlHardware")
public class XmlHardwareDescription
{
   @XmlElement(required = true)
   protected XmlPriority priority;
   @XmlElement
   protected XmlDevices devices;
   @XmlElement
   protected XmlTransmissions transmissions;
   @XmlElement
   protected XmlJoints joints;
   @XmlElement
   protected XmlSettings settings;

   public XmlDevices getDevices()
   {
      return devices;
   }

   public void setDevices(XmlDevices devices)
   {
      this.devices = devices;
   }

   public XmlTransmissions getTransmissions()
   {
      return transmissions;
   }

   public void setTransmissions(XmlTransmissions transmissions)
   {
      this.transmissions = transmissions;
   }

   public XmlJoints getJoints()
   {
      return joints;
   }

   public void setJoints(XmlJoints joints)
   {
      this.joints = joints;
   }

   public XmlPriority getPriority()
   {
      return priority;
   }

   public void setPriority(XmlPriority priority)
   {
      this.priority = priority;
	}

   public XmlSettings getSettings()
   {
      return settings;
   }

   public void setSettings(XmlSettings settings)
   {
      this.settings = settings;
   }
}
