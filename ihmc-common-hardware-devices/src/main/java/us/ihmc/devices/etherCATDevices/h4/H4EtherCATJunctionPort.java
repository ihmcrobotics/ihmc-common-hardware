package us.ihmc.devices.etherCATDevices.h4;

import us.ihmc.hardwareStatusUI.controllerSide.EtherCATDeviceStatusProvider;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

public class H4EtherCATJunctionPort extends Slave implements EtherCATDeviceStatusProvider
{
   private static final int VENDOR_ID = 0x000004d8;
   private static final int[] PRODUCT_CODES = {0x10000001, 0x10000002, 0x10000003, 0x10000004};

   class JunctionData extends TxPDO
   {
      protected JunctionData()
      {
         super(0x1a00);
      }
   }

   class JunctionControl extends RxPDO
   {
      protected JunctionControl()
      {
         super(0x1600);
      }
   }

   // Leave just in case we add commands/data to PDO of ethercat junctions
   private final JunctionControl junctionControl = new JunctionControl();
   private final JunctionData junctionData = new JunctionData();

   public H4EtherCATJunctionPort(int alias, int position, int junctionPort)
   {
      super(VENDOR_ID, PRODUCT_CODES[junctionPort], alias, position);

      registerSyncManager(new SyncManager(2, false));
      registerSyncManager(new SyncManager(3, false));

      sm(2).registerPDO(junctionControl);
      sm(3).registerPDO(junctionData);
   }

   @Override
   public boolean isResponding()
   {
      return isOperational();
   }
}