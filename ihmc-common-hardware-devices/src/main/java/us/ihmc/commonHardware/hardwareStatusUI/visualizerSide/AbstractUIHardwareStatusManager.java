package us.ihmc.commonHardware.hardwareStatusUI.visualizerSide;

import us.ihmc.commonHardware.hardwareStatusUI.controllerSide.DeviceStatusHolder;
import us.ihmc.commonHardware.xmlDescription.devices.AbstractXmlDevice;
import us.ihmc.commonHardware.xmlDescription.devices.XmlH4EtherCATJunctionPort;
import us.ihmc.commonHardware.xmlDescription.devices.XmlIMU;
import us.ihmc.commonHardware.xmlDescription.devices.XmlTemperatureSensor;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractUIHardwareStatusManager
{
   public enum DeviceType {MOTOR, BOARD, SENSOR}
   protected final ArrayList<UIDeviceStatusHolder> deviceDataHolders = new ArrayList<>();
   protected final SessionVisualizerControls sessionVisualizerControls;
   protected final SessionVisualizerToolkit toolkit;

   /**
    * Abstract class that parses a list of generic {@code AbstractXmlDevice}, and
    * creates a {@code UIDeviceStatusHolder} for each device in the list. In doing this,
    * this class searches a {@code YoVariableRegistry} for the device status YoVariables
    * corresponding to a given {@code AbstractXmlDevice} based on that xml device's name.
    * The specific list of {@code AbstractXmlDevice} to parse, and the details of how that
    * list is parsed is to be determined by the robot-specific child classes that extend
    * this one.
    */
   public AbstractUIHardwareStatusManager(List<AbstractXmlDevice> xmlDevices, SessionVisualizerControls sessionVisualizerControls, SessionVisualizerToolkit toolkit)
   {
      this.sessionVisualizerControls = sessionVisualizerControls;
      this.toolkit = toolkit;

      sessionVisualizerControls.addSessionChangedListener((oldSession, newSession) ->
                                                          {
                                                             deviceDataHolders.clear();

                                                             if (newSession != null)
                                                                createDevices(xmlDevices);
                                                          });
   }

   protected abstract void createDevices(List<AbstractXmlDevice> xmlDevices);

   protected void createNewEtherCATDataHolder(String name, String description, int position, int alias, List<AbstractXmlDevice> daughterDevices, boolean useParentPositionAndAlias, DeviceType deviceType)
   {
      createNewEtherCATDataHolder(name, description, position, alias, deviceType);

      for (AbstractXmlDevice daughterDevice : daughterDevices)
      {
         if (daughterDevice instanceof XmlIMU xmlIMU && useParentPositionAndAlias)
            createNewEtherCATDataHolder(xmlIMU.getName() + "_" + name, "IMU", position, alias, DeviceType.BOARD, true);

         else if (daughterDevice instanceof XmlIMU xmlIMU)
            createNewEtherCATDataHolder(xmlIMU.getName() + "_" + name, "IMU", xmlIMU.getPosition(), xmlIMU.getAlias(), DeviceType.BOARD, true);

         else if (daughterDevice instanceof XmlTemperatureSensor xmlTemperatureSensor)
            createNewEtherCATDataHolder(xmlTemperatureSensor.getName() + "_" + name, "Temp Sensor", position, alias, DeviceType.BOARD, true);

         else if (daughterDevice instanceof XmlH4EtherCATJunctionPort xmlH4EtherCATJunctionPort && useParentPositionAndAlias)
            createNewEtherCATDataHolder(xmlH4EtherCATJunctionPort.getName() + "_" + name, "Junction Port", position, alias, DeviceType.BOARD, true);

         else if (daughterDevice instanceof XmlH4EtherCATJunctionPort xmlH4EtherCATJunctionPort)
            createNewEtherCATDataHolder(xmlH4EtherCATJunctionPort.getName() + "_" + name, "Junction Port", xmlH4EtherCATJunctionPort.getPosition(), xmlH4EtherCATJunctionPort.getAlias(), DeviceType.BOARD, true);
      }
   }

   protected void createNewEtherCATDataHolder(String dataHolderName, String description, int position, int alias, DeviceType deviceType)
   {
      createNewEtherCATDataHolder(dataHolderName, description, position, alias, deviceType, false);
   }

   protected void createNewEtherCATDataHolder(String dataHolderName, String description, int position, int alias, DeviceType deviceType, boolean childDevice)
   {
      if (!doesVariableExist(YoBoolean.class, dataHolderName + DeviceStatusHolder.IS_RESPONDING_SUFFIX))
      {
         LogTools.warn("Could not create Hardware Status UI Data Holder for device: " + dataHolderName + DeviceStatusHolder.IS_RESPONDING_SUFFIX + ". Variable(s) not found in registry");
         return;
      }
      else if (!doesVariableExist(YoEnum.class, dataHolderName + DeviceStatusHolder.STATE_SUFFIX))
      {
         LogTools.warn("Could not create Hardware Status UI Data Holder for device: " + dataHolderName + DeviceStatusHolder.STATE_SUFFIX + ". Variable(s) not found in registry");
         return;
      }

      if (childDevice)
         deviceDataHolders.add(new UIDeviceStatusHolder(dataHolderName,
                                                        "",
                                                        description,
                                                        sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.IS_RESPONDING_SUFFIX).getYoVariable(),
                                                        sessionVisualizerControls.newYoEnumProperty(dataHolderName + DeviceStatusHolder.STATE_SUFFIX),
                                                        position,
                                                        alias,
                                                        deviceType));
      else
         deviceDataHolders.add(new UIDeviceStatusHolder(dataHolderName,
                                                        description,
                                                        "",
                                                        sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.IS_RESPONDING_SUFFIX).getYoVariable(),
                                                        sessionVisualizerControls.newYoEnumProperty(dataHolderName + DeviceStatusHolder.STATE_SUFFIX),
                                                        position,
                                                        alias,
                                                        deviceType));
   }

   protected <T extends YoVariable> boolean doesVariableExist(Class<T> type, String variableName)
   {
      int separatorIndex = variableName.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      String namespaceEnding = separatorIndex == -1 ? null : variableName.substring(0, separatorIndex);
      String name = separatorIndex == -1 ? variableName : variableName.substring(separatorIndex + 1);
      T variable = (T) YoSearchTools.findFirstVariable(namespaceEnding, name, type::isInstance, toolkit.getYoManager().getRootRegistry());
      return variable != null;
   }

   public ArrayList<UIDeviceStatusHolder> getDeviceDataHolders()
   {
      return deviceDataHolders;
   }
}
