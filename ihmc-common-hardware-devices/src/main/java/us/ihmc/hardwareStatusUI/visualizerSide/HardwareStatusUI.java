package us.ihmc.hardwareStatusUI.visualizerSide;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.SplitPane;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.TableCell;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableRow;
import javafx.scene.control.TableView;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class HardwareStatusUI implements VisualizerController
{
   private final SimulationConstructionSet2 scs;
   private final AbstractUIHardwareStatusManager hardwareStatusUIDataManager;

   private final ObservableList<UIDeviceStatusHolder> allDevicesStatusData = FXCollections.observableArrayList();
   private final ObservableList<UIDeviceStatusHolder> motorsStatusData = FXCollections.observableArrayList();
   private final ObservableList<UIDeviceStatusHolder> boardsStatusData = FXCollections.observableArrayList();
   private final ObservableList<UIDeviceStatusHolder> sensorsStatusData = FXCollections.observableArrayList();

   private final TableView<UIDeviceStatusHolder> allDevicesStatusTable = new TableView<>();
   private final TableView<UIDeviceStatusHolder> motorsStatusTable = new TableView<>();
   private final TableView<UIDeviceStatusHolder> boardsStatusTable = new TableView<>();
   private final TableView<UIDeviceStatusHolder> sensorsStatusTable = new TableView<>();

   private final Tab allDevicesStatusTab = new Tab("All Devices");
   private final Tab motorsStatusTab = new Tab("Motors");
   private final Tab boardsStatusTab = new Tab("Boards");
   private final Tab sensorsStatusTab = new Tab("Sensors");

   private final TabPane deviceStatusTablePane = new TabPane();
   private final StackPane robotFrontViewPane = new StackPane();
   private final StackPane robotBackViewPane = new StackPane();

   private final SplitPane splitPane = new SplitPane();
   private final Scene scene = new Scene(splitPane, 1000, 800);
   private Stage stage;

   /**
    * Graphical User Interface (GUI) responsible for displaying device status info of certain
    * hardware devices on a given robot such as motors, boards, or sensors. This status
    * information can include the device name, its ID, whether it is responding, its state, and
    * potentially other data as well. It is designed to attach to a Simulation Construction
    * Set {@code SessionVisualizer} where it can be launched using an intractable GUI button,
    * and then resized, moved, or closed like any normal graphical window.
    */
   public HardwareStatusUI(SessionVisualizerControls sessionVisualizerControls, AbstractUIHardwareStatusManager hardwareStatusUIDataManager)
   {
      this.hardwareStatusUIDataManager = hardwareStatusUIDataManager;
      scs = new SimulationConstructionSet2();

      createDeviceStatusTablePane();

      createLaunchUIButtonAndPane(sessionVisualizerControls);

      sessionVisualizerControls.addSessionChangedListener((oldSession, newSession) ->
                                                          {
                                                             clearDataTables();

                                                             if (newSession != null)
                                                                populateDataTables();
                                                          });
   }

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      splitPane.getItems().addAll(deviceStatusTablePane);

      stage = toolkit.getWindow();
      stage.setTitle("Hardware Device Status UI");
      stage.setScene(scene);
      stage.setMaximized(true);
      JavaFXMissingTools.centerWindowInOwner(stage, toolkit.getWindow());
   }

   private void createDeviceStatusTablePane()
   {
      setupDataTable(allDevicesStatusTab, allDevicesStatusTable, true, true, true);
      setupDataTable(motorsStatusTab, motorsStatusTable, false, false, true);
      setupDataTable(boardsStatusTab, boardsStatusTable, true, false, true);
      setupDataTable(sensorsStatusTab, sensorsStatusTable, false, false, true);
   }

   private void createLaunchUIButtonAndPane(SessionVisualizerControls sessionVisualizerControls)
   {
      Pane hardwareStatusUILaunchButtonPane = new Pane();

      Button launchHardwareStatusUIButton = new Button("Launch Hardware Status UI");
      launchHardwareStatusUIButton.setOnAction(e -> show());
      hardwareStatusUILaunchButtonPane.getChildren().add(launchHardwareStatusUIButton);
      launchHardwareStatusUIButton.relocate(5, 5);

      sessionVisualizerControls.addCustomGUIPane("Hardware Status UI", hardwareStatusUILaunchButtonPane);
   }

   private void clearDataTables()
   {
      allDevicesStatusData.clear();
      motorsStatusData.clear();
      boardsStatusData.clear();
      sensorsStatusData.clear();

      allDevicesStatusTable.getItems().clear();
      motorsStatusTable.getItems().clear();
      boardsStatusTable.getItems().clear();
      sensorsStatusTable.getItems().clear();
   }

   private void populateDataTables()
   {
      for (int i = 0; i < hardwareStatusUIDataManager.getDeviceDataHolders().size(); i++)
         addData(hardwareStatusUIDataManager.getDeviceDataHolders().get(i));

      allDevicesStatusTable.setItems(allDevicesStatusData);
      motorsStatusTable.setItems(motorsStatusData);
      boardsStatusTable.setItems(boardsStatusData);
      sensorsStatusTable.setItems(sensorsStatusData);
   }

   private void addData(UIDeviceStatusHolder dataHolder)
   {
      allDevicesStatusData.add(dataHolder);
      dataHolder.addDataStringChangeListener((observable, oldValue, newValue) -> allDevicesStatusTable.refresh());
      dataHolder.addDataBooleanChangeListener((observable, oldValue, newValue) -> allDevicesStatusTable.refresh());

      switch (dataHolder.getDeviceType())
      {
         case MOTOR ->
         {
            motorsStatusData.add(dataHolder);
            dataHolder.addDataStringChangeListener((observable, oldValue, newValue) -> motorsStatusTable.refresh());
            dataHolder.addDataBooleanChangeListener((observable, oldValue, newValue) -> motorsStatusTable.refresh());
         }
         case BOARD ->
         {
            boardsStatusData.add(dataHolder);
            dataHolder.addDataStringChangeListener((observable, oldValue, newValue) -> boardsStatusTable.refresh());
            dataHolder.addDataBooleanChangeListener((observable, oldValue, newValue) -> boardsStatusTable.refresh());
         }
         case SENSOR ->
         {
            sensorsStatusData.add(dataHolder);
            dataHolder.addDataStringChangeListener((observable, oldValue, newValue) -> sensorsStatusTable.refresh());
            dataHolder.addDataBooleanChangeListener((observable, oldValue, newValue) -> sensorsStatusTable.refresh());
         }
      }
   }

   private void setupDataTable(Tab tab, TableView<UIDeviceStatusHolder> table, boolean addChildDeviceNameColumn, boolean showCANInfo, boolean showEtherCATInfo)
   {
      // Set up table settings
      table.setEditable(false);

      // Create device name column
      TableColumn<UIDeviceStatusHolder, String> deviceNameColumn = new TableColumn<>("Device Name");
      deviceNameColumn.setMinWidth(250);
      deviceNameColumn.setCellValueFactory(new PropertyValueFactory<>("description"));

      if (addChildDeviceNameColumn)
      {
         TableColumn<UIDeviceStatusHolder, String> parentNameColumn = new TableColumn<>("Parent Device Name");
         parentNameColumn.setMinWidth(250);
         parentNameColumn.setCellValueFactory(new PropertyValueFactory<>("description"));

         TableColumn<UIDeviceStatusHolder, String> childNameColumn = new TableColumn<>("Child Device Name");
         childNameColumn.setMinWidth(125);
         childNameColumn.setCellValueFactory(new PropertyValueFactory<>("childDescription"));

         deviceNameColumn.getColumns().clear();
         deviceNameColumn.getColumns().addAll(parentNameColumn, childNameColumn);
         deviceNameColumn.getColumns().forEach(column -> column.setSortable(false));
         deviceNameColumn.setMinWidth(325);
      }

      // Create column that displays the device responsiveness
      TableColumn<UIDeviceStatusHolder, Boolean> isRespondingColumn = new TableColumn<>("Is Responding");
      isRespondingColumn.setMinWidth(100);
      isRespondingColumn.setCellValueFactory(new PropertyValueFactory<>("isResponding"));
      isRespondingColumn.setCellFactory(column -> new TableCell<>() {
         @Override
         protected void updateItem(Boolean item, boolean empty)
         {
            super.updateItem(item, empty);

            setText(empty ? "" : getItem().toString());
            setGraphic(null);

            TableRow<UIDeviceStatusHolder> currentRow = getTableRow();

            if (!isEmpty())
            {
               if(!item)
                  currentRow.setStyle("-fx-background-color:red");
               else
                  currentRow.setStyle("");
            }
         }
      });

      // Create column that displays the device CAN read status
      TableColumn<UIDeviceStatusHolder, String> readStatusColumn = new TableColumn<>("CAN Read Status");
      readStatusColumn.setMinWidth(200);
      readStatusColumn.setCellValueFactory(new PropertyValueFactory<>("readStatus"));

      // Create column that displays the device CAN write status
      TableColumn<UIDeviceStatusHolder, String> writeStatusColumn = new TableColumn<>("CAN Write Status");
      writeStatusColumn.setMinWidth(200);
      writeStatusColumn.setCellValueFactory(new PropertyValueFactory<>("writeStatus"));

      // Create column that displays the device EtherCat state
      TableColumn<UIDeviceStatusHolder, String> etherCATStateColumn = new TableColumn<>("EtherCAT State");
      etherCATStateColumn.setMinWidth(150);
      etherCATStateColumn.setCellValueFactory(new PropertyValueFactory<>("state"));

      // Create column that displays the device ID/Position/Alias
      TableColumn<UIDeviceStatusHolder, String> idColumn = new TableColumn<>("ID");
      idColumn.setMinWidth(150);
      idColumn.setCellValueFactory(new PropertyValueFactory<>("id"));

      // Add columns and column headers to table
      table.getColumns().clear();
      table.getColumns().addAll(deviceNameColumn, isRespondingColumn, readStatusColumn, writeStatusColumn, etherCATStateColumn, idColumn);

      // Remove columns that don't need to be shown
      if (!showCANInfo)
      {
         table.getColumns().remove(readStatusColumn);
         table.getColumns().remove(writeStatusColumn);
      }

      if (!showEtherCATInfo)
         table.getColumns().remove(etherCATStateColumn);

      // Set up tab and table settings
      table.getColumns().forEach(column -> column.setSortable(false));
      tab.setClosable(false);

      // Add data table to tab
      tab.setContent(table);

      // If the tab pane doesn't have the tab added, add it now
      if (!deviceStatusTablePane.getTabs().contains(tab))
         deviceStatusTablePane.getTabs().add(tab);
   }

   public void show()
   {
      stage.show();
      splitPane.setDividerPositions(0.15, 0.3);
   }

   public void hide()
   {
      stage.hide();
   }
}
