package us.ihmc.hardwareXMLToolkit.transmissions;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;
import us.ihmc.hardwareXMLToolkit.AbstractXmlObject;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlCycloidMechanism")
public class XmlCycloidMechanism extends AbstractXmlObject
{
   @XmlElement(required = true)
   protected String jointName;

   @XmlElement(required = true)
   protected String motorName;

   @XmlElement(required = true)
   protected double jointPositionOffset;

   @XmlElement(defaultValue = "0.0")
   protected double upperJointLimit;

   @XmlElement(defaultValue = "0.0")
   protected double lowerJointLimit;

   @XmlElement(defaultValue = "Double.POSITIVE_INFINITY")
   protected double torqueBreakFrequency = Double.POSITIVE_INFINITY;

   @XmlElement(defaultValue = "false")
   protected boolean useFilteredStates;

   @XmlElement(defaultValue = "false")
   protected boolean publishFilteredStates;

   @XmlElement(defaultValue = "false")
   protected boolean doPDControlOnTwitter;

   public String getJointName()
   {
      return jointName;
   }

   public void setJointName(String joint)
   {
      this.jointName = joint;
   }

   public String getMotorName()
   {
      return motorName;
   }

   public void setMotor(String motorName)
   {
      this.motorName = motorName;
   }

   public double getJointPositionOffset()
   {
      return jointPositionOffset;
   }

   public double getUpperJointLimit()
   {
      return upperJointLimit;
   }

   public double getLowerJointLimit()
   {
      return lowerJointLimit;
   }

   public double getTorqueBreakFrequency()
   {
      return torqueBreakFrequency;
   }

   public boolean useFilteredStates()
   {
      return useFilteredStates;
   }

   public boolean publishFilteredStates()
   {
      return publishFilteredStates;
   }

   public boolean doPDControlOnTwitter()
   {
      return doPDControlOnTwitter;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + " [jointName=" + jointName + ", motorName=" + motorName
            + ", jointPositionOffset=" + jointPositionOffset + ", present=" + present + "]";
   }
}
