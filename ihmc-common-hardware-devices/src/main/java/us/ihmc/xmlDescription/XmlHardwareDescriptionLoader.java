package us.ihmc.xmlDescription;

import jakarta.xml.bind.JAXBContext;
import jakarta.xml.bind.JAXBException;
import jakarta.xml.bind.Unmarshaller;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.Collection;
import java.util.List;

public class XmlHardwareDescriptionLoader
{
   public static final File descriptionFileDirectory = new File(System.getProperty("user.home"), "robot-configuration");

   public static List<XmlHardwareDescription> getHardwareDescriptionFromResources(Collection<String> names)
   {
      return names.stream().map(XmlHardwareDescriptionLoader::getHardwareDescriptionFromResources).toList();
   }

   public static XmlHardwareDescription getHardwareDescriptionFromResources(String name)
   {
      InputStream hardwareStream = XmlHardwareDescriptionLoader.class.getResourceAsStream("/parameters/" + name);
      XmlHardwareDescription hardwareDescription = XmlHardwareDescriptionLoader.getHardwareDescription(hardwareStream);
      return hardwareDescription;
   }

   public static XmlHardwareDescription getHardwareDescription(File descriptionFile)
   {
      if (!descriptionFileDirectory.exists() || !descriptionFileDirectory.isDirectory())
      {
         throw new RuntimeException("Cannot load robot configuration xml from " + descriptionFileDirectory.getAbsolutePath() + "." + System.lineSeparator());
      }

      try
      {
         InputStream is = new FileInputStream(descriptionFile);
         return getHardwareDescription(is);
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static XmlHardwareDescription getHardwareDescription(InputStream is)
   {
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(XmlHardwareDescription.class);
         Unmarshaller unmarshaller = jaxbContext.createUnmarshaller();

         XmlHardwareDescription hardware = (XmlHardwareDescription) unmarshaller.unmarshal(is);

         return hardware;
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }
}
