package us.ihmc.xmlDescription.transmissions;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlElements;
import jakarta.xml.bind.annotation.XmlType;
import us.ihmc.xmlDescription.AbstractXmlObject;

import java.util.ArrayList;
import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlTransmissions")
public class XmlTransmissions
{
   @XmlElements({@XmlElement(name = "XmlCycloidMotorMechanism", type = XmlCycloidMotorMechanism.class),
                 @XmlElement(name = "XmlSakeFingerMechanism", type = XmlSakeFingerMechanism.class),
                 @XmlElement(name = "XmlJointPairMechanism", type = XmlJointPairMechanism.class)})
   protected List<? extends AbstractXmlObject> transmissions;

   public List<? extends AbstractXmlObject> getTransmissions()
   {
      if (transmissions == null)
      {
         transmissions = new ArrayList<>();
      }
      return transmissions;
   }
}
