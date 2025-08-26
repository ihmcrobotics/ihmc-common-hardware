package us.ihmc.hardwareXMLToolkit;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import us.ihmc.hardwareXMLToolkit.devices.XmlDevices;
import us.ihmc.hardwareXMLToolkit.joints.XmlJoints;
import us.ihmc.hardwareXMLToolkit.priority.XmlPriority;
import us.ihmc.hardwareXMLToolkit.settings.XmlSettings;
import us.ihmc.hardwareXMLToolkit.transmissions.XmlTransmissions;

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

   /**
    * @return An {@code XmlDevices} object containing all the devices in the xml
    */
   public XmlDevices getDevices()
   {
      return devices;
   }

   /**
    * Set the devices on the robot
    * @param devices An {@code XmlDevices} object containing the devices on the robot
    */
   public void setDevices(XmlDevices devices)
   {
      this.devices = devices;
   }

   /**
    * @return An {@code XmlTransmissions} object containing all the transmissions in the xml
    */
   public XmlTransmissions getTransmissions()
   {
      return transmissions;
   }

   /**
    * Set the transmissions on the robot
    * @param transmissions An {@code XmlTransmissions} object containing the transmissions on the robot
    */
   public void setTransmissions(XmlTransmissions transmissions)
   {
      this.transmissions = transmissions;
   }

   /**
    * @return An {@code XmlJoints} object containing all the joints in the xml
    */
   public XmlJoints getJoints()
   {
      return joints;
   }

   /**
    * Set the joints on the robot
    * @param joints An {@code XmlJoints} object containing the joints on the robot
    */
   public void setJoints(XmlJoints joints)
   {
      this.joints = joints;
   }

   /**
    * @return An {@code XmlPriority} object containing the priority parameters of the robot
    */
   public XmlPriority getPriority()
   {
      return priority;
   }

   /**
    * Set the priority parameters on the robot
    * @param priority An {@code XmlPriority} object containing the priority parameters on the robot
    */
   public void setPriority(XmlPriority priority)
   {
      this.priority = priority;
	}

   /**
    * @return An {@code XmlSettings} object containing the ros settings of the robot
    */
   public XmlSettings getSettings()
   {
      return settings;
   }

   /**
    * Set the ros settings on the robot
    * @param settings An {@code XmlSettings} object containing the ros settings on the robot
    */
   public void setSettings(XmlSettings settings)
   {
      this.settings = settings;
   }
}
