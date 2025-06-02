package us.ihmc.xmlDescription;

import jakarta.xml.bind.JAXBContext;
import jakarta.xml.bind.JAXBException;
import jakarta.xml.bind.Unmarshaller;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class XmlHardwareDescriptionLoader
{
   public static final File descriptionFileDirectory = new File(System.getProperty("user.home"), "robot-configuration");
   private static final String DEFAULT_DIRECTORY = "parameters/";

   public static List<XmlHardwareDescription> getHardwareDescriptionFromAlternateResources(String directory, Collection<String> names)
   {
      List<XmlHardwareDescription> hardwareDescriptions = new ArrayList<>();
      for(String resource : names)
         hardwareDescriptions.add(getHardwareDescriptionFromAlternateResource(directory, resource));
      return hardwareDescriptions;
   }

   public static List<XmlHardwareDescription> getHardwareDescriptionFromResources(Collection<String> names)
   {
      return names.stream().map(XmlHardwareDescriptionLoader::getHardwareDescriptionFromResources).toList();
   }

   public static XmlHardwareDescription getHardwareDescriptionFromAlternateResource(String directory, String name)
   {
      InputStream hardwareStream = XmlHardwareDescriptionLoader.class.getClassLoader().getResourceAsStream(directory + name);
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

   public static XmlHardwareDescription getHardwareDescriptionFromResources(String name)
   {
      return getHardwareDescriptionFromAlternateResource(DEFAULT_DIRECTORY, name);
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
