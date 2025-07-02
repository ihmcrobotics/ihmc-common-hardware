package us.ihmc.commonHardware.xmlDescription.joints;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlElements;
import jakarta.xml.bind.annotation.XmlType;

import java.util.ArrayList;
import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlJoints")
public class XmlJoints
{
   @XmlElements({@XmlElement(name = "XmlJoint", type = XmlJoint.class) })
   protected List<XmlJoint> joints;

   public List<XmlJoint> getJoints()
   {
      if (joints == null)
      {
         joints = new ArrayList<XmlJoint>();
      }
      return this.joints;
   }
}
