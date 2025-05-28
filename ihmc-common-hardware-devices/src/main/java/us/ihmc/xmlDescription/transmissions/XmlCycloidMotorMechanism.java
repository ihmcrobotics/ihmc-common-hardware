package us.ihmc.xmlDescription.transmissions;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;
import us.ihmc.xmlDescription.AbstractXmlObject;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlCycloidMotorMechanism")
public class XmlCycloidMotorMechanism extends AbstractXmlObject
{
   @XmlElement(required = true)
   protected String jointName;

   @XmlElement(required = true)
   protected String motorName;

   @XmlElement(required = true)
   protected double jointPositionOffset;

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

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + " [jointName=" + jointName + ", motorName=" + motorName
            + ", jointPositionOffset=" + jointPositionOffset + ", present=" + present + "]";
   }
}
