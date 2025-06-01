package us.ihmc.xmlDescription.devices.parameters;

import jakarta.xml.bind.JAXBContext;
import jakarta.xml.bind.JAXBException;
import jakarta.xml.bind.Unmarshaller;
import us.ihmc.xmlDescription.XmlHardwareDescription;

import java.io.InputStream;

public class XmlCycloidParameterLoader
{
   private static final String parameterDirectory = "/parameters/actuators/";

   public static XmlCycloidParameters getCycloidParametersFromActuatorPackageName(String actuatorPackage)
   {
      InputStream parameterStream = XmlCycloidParameterLoader.class.getResourceAsStream(parameterDirectory + actuatorPackage + ".xml");
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(XmlHardwareDescription.class);
         Unmarshaller unmarshaller = jaxbContext.createUnmarshaller();

         return (XmlCycloidParameters) unmarshaller.unmarshal(parameterStream);
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }
}
