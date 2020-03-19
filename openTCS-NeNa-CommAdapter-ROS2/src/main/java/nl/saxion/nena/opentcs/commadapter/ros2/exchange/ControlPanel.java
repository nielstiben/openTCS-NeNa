/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.exchange;

import com.google.common.base.Strings;
import com.google.inject.assistedinject.Assisted;
import nl.saxion.nena.opentcs.commadapter.ros2.ExampleProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SendRequestCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SetDisconnectingOnVehicleIdleCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SetIdleTimeoutCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SetLoggingEnabledCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SetReconnectingOnConnectionLossCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SetVehicleHostCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SetVehiclePortCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderAction;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderRequest;
import java.util.Objects;
import static java.util.Objects.requireNonNull;
import javax.inject.Inject;
import javax.swing.DefaultListModel;
import javax.swing.JOptionPane;
import javax.swing.SwingUtilities;
import javax.swing.text.AttributeSet;
import javax.swing.text.BadLocationException;
import javax.swing.text.PlainDocument;

import org.opentcs.components.kernel.services.VehicleService;
import org.opentcs.customizations.ServiceCallWrapper;
import org.opentcs.data.model.Point;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.opentcs.drivers.vehicle.management.VehicleCommAdapterPanel;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.CallWrapper;
import org.opentcs.util.Comparators;
import org.opentcs.util.gui.StringListCellRenderer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Implements the gui of the vehicle control and visualizes the status of protocol data.
 *
 * @author Mats Wilhelm (Fraunhofer IML)
 */
public class ControlPanel
    extends VehicleCommAdapterPanel {

  /**
   * This class' logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(ControlPanel.class);
  /**
   * Declares how many orders should be saved.
   */
  private static final int MAX_LAST_ORDERS = 20;
  /**
   * Model for JList.
   */
  private final DefaultListModel<OrderRequest> lastOrderListModel = new DefaultListModel<>();
  /**
   * The vehicle service used for interaction with the comm adapter.
   */
  private final VehicleService vehicleService;
  /**
   * The call wrapper to use for service calls.
   */
  private final CallWrapper callWrapper;
  /**
   * The comm adapter's process model.
   */
  private ExampleProcessModelTO processModel;

  /**
   * Creates a new instance.
   *
   * @param processModel The comm adapter's process model
   * @param vehicleService The vehicle service
   * @param callWrapper The call wrapper to use for service calls
   */
  @Inject
  public ControlPanel(@Assisted ExampleProcessModelTO processModel,
                      @Assisted VehicleService vehicleService,
                      @ServiceCallWrapper CallWrapper callWrapper) {
    this.processModel = requireNonNull(processModel, "processModel");
    this.vehicleService = requireNonNull(vehicleService, "vehicleService");
    this.callWrapper = requireNonNull(callWrapper, "callWrapper");
    initComponents();
    initComboBoxes();
    initGuiContent();
  }

  /**
   * Initializes combo boxes for destinations and actions.
   */
  private void initComboBoxes() {
    try {
      // Initialize the list of known points. Only add points whose names have a length of 8.
      destinationComboBox.removeAllItems();
      callWrapper.call(() -> vehicleService.fetchObjects(Point.class)).stream()
          .sorted(Comparators.objectsByName())
          .forEach(point -> destinationComboBox.addItem(point));
      // Initialize the list of valid actions.
      actionComboBox.removeAllItems();
      for (OrderAction curAction : OrderAction.values()) {
        actionComboBox.addItem(curAction.name());
      }
    }
    catch (Exception ex) {
      LOG.warn("Error fetching points", ex);
    }
  }

  /**
   * Updates all fields showing an attribute of the process model to the current state
   */
  private void initGuiContent() {
    // Trigger an update for all attributes once first.
    for (VehicleProcessModel.Attribute attribute : VehicleProcessModel.Attribute.values()) {
      processModelChange(attribute.name(), processModel);
    }
    for (ExampleProcessModel.Attribute attribute : ExampleProcessModel.Attribute.values()) {
      processModelChange(attribute.name(), processModel);
    }
  }

  @Override
  public void processModelChange(String attributeChanged, VehicleProcessModelTO newProcessModel) {
    if (!(newProcessModel instanceof ExampleProcessModelTO)) {
      return;
    }
    processModel = (ExampleProcessModelTO) newProcessModel;

    if (Objects.equals(attributeChanged,
                       VehicleProcessModel.Attribute.COMM_ADAPTER_ENABLED.name())) {
      updateCommAdapterEnabled(processModel.isCommAdapterEnabled());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.COMM_ADAPTER_CONNECTED.name())) {
      updateCommAdapterConnected(processModel.isCommAdapterConnected());
    }
    else if (Objects.equals(attributeChanged, ExampleProcessModel.Attribute.VEHICLE_HOST.name())) {
      updateVehicleHost(processModel.getVehicleHost());
    }
    else if (Objects.equals(attributeChanged, ExampleProcessModel.Attribute.VEHICLE_PORT.name())) {
      updateVehiclePort(processModel.getVehiclePort());
    }
    else if (Objects.equals(attributeChanged,
                            ExampleProcessModel.Attribute.VEHICLE_IDLE_TIMEOUT.name())) {
      updateVehicleTimeout(processModel.getVehicleIdleTimeout());
    }
    else if (Objects.equals(attributeChanged, ExampleProcessModel.Attribute.VEHICLE_IDLE.name())) {
      updateVehicleIdle(processModel.isVehicleIdle());
    }
    else if (Objects.equals(attributeChanged,
                            ExampleProcessModel.Attribute.DISCONNECTING_ON_IDLE.name())) {
      updateDisconnectingOnIdle(processModel.isDisconnectingOnVehicleIdle());
    }
    else if (Objects.equals(attributeChanged,
                            ExampleProcessModel.Attribute.RECONNECTING_ON_CONNECTION_LOSS.name())) {
      updateReconnectingOnConnectionLoss(processModel.isReconnectingOnConnectionLoss());
    }
    else if (Objects.equals(attributeChanged, ExampleProcessModel.Attribute.LAST_ORDER.name())) {
      updateLastOrder(processModel.getLastOrderSent(), processModel.isCommAdapterConnected());
    }
    else if (Objects.equals(attributeChanged,
                            ExampleProcessModel.Attribute.LOGGING_ENABLED.name())) {
      updateLoggingEnabled(processModel.isLoggingEnabled());
    }
  }

  /**
   * Updates the state of specific elements in the gui to let the user interact with them or not
   * depending on the enabled state of the comm adapter.
   *
   * @param enabled The enabled state of the comm adapter
   */
  private void updateCommAdapterEnabled(boolean enabled) {
    SwingUtilities.invokeLater(() -> {
      enableAdapterCheckBox.setSelected(enabled);

      hostTextField.setEditable(!enabled);
      portTextField.setEditable(!enabled);
      aliveTimeoutTextField.setEditable(!enabled);
      disconnectOnTimeoutChkBox.setEnabled(!enabled);
      reconnectOnConnectionLossChkBox.setEnabled(!enabled);
      enableLoggingChkBox.setEnabled(!enabled);
    });
  }

  /**
   * Updates buttons for interacting with the vehicle when the connection is established or not.
   *
   * @param connected Whether the connection to the vehicle is established
   */
  private void updateCommAdapterConnected(boolean connected) {
    SwingUtilities.invokeLater(() -> {
      connectedButton.setSelected(connected);

      sendOrderButton.setEnabled(connected);
      repeatLastOrderButton.setEnabled(connected);
    });
  }

  /**
   * Updates the shown host ip in the gui.
   *
   * @param host The host ip
   */
  private void updateVehicleHost(String host) {
    SwingUtilities.invokeLater(() -> hostTextField.setText(host));
  }

  /**
   * Updates the shown port in the gui
   *
   * @param port The port
   */
  private void updateVehiclePort(int port) {
    SwingUtilities.invokeLater(() -> portTextField.setText(Integer.toString(port)));
  }

  /**
   * Updates the shown idle timeout in the gui.
   *
   * @param timeout The idle timeout
   */
  private void updateVehicleTimeout(int timeout) {
    SwingUtilities.invokeLater(() -> aliveTimeoutTextField.setText(Integer.toString(timeout)));
  }

  /**
   * Updates the idle state of the vehicle in the gui.
   *
   * @param idle The idle state
   */
  private void updateVehicleIdle(boolean idle) {
    SwingUtilities.invokeLater(() -> activeButton.setSelected(!idle));
  }

  /**
   * Updates the checkbox on whether the connection should be closed when the vehicle is idle.
   *
   * @param disconnectingOnIdle Whether the connection should be closed if the vehicle is idle
   */
  private void updateDisconnectingOnIdle(boolean disconnectingOnIdle) {
    SwingUtilities.invokeLater(() -> disconnectOnTimeoutChkBox.setSelected(disconnectingOnIdle));
  }

  /**
   * Updates the checkbox on whether a connection should be reestablished after loss.
   *
   * @param reconnect Whether a connection should be reestablished after loss
   */
  private void updateReconnectingOnConnectionLoss(boolean reconnect) {
    SwingUtilities.invokeLater(() -> reconnectOnConnectionLossChkBox.setSelected(reconnect));
  }

  /**
   * Updates the list of last orders sent with the given one as last order.
   *
   * @param lastOrderSent The last order sent
   * @param connected The connection state to the vehicle with <code>true</code> indicates a
   * connection is established
   */
  private void updateLastOrder(OrderRequest lastOrderSent, boolean connected) {
    SwingUtilities.invokeLater(() -> {
      if (lastOrderSent != null) {
        OrderRequest selectedTelegram = lastOrdersList.getSelectedValue();
        if (selectedTelegram == null) {
          selectedTelegram = lastOrderSent;
        }

        updateLastOrderTextFields(selectedTelegram);
        repeatLastOrderButton.setEnabled(connected);

        lastOrderListModel.add(0, lastOrderSent);
        // Delete last element if our listmodel contains too many elements
        while (lastOrderListModel.getSize() > MAX_LAST_ORDERS) {
          lastOrderListModel.removeElement(lastOrderListModel.lastElement());
        }
      }
    });
  }

  /**
   * Updates the checkbox on whether the comm adapter logging is enabled.
   *
   * @param enabled Whether the comm adapter logging is enabled
   */
  private void updateLoggingEnabled(boolean enabled) {
    SwingUtilities.invokeLater(() -> enableLoggingChkBox.setSelected(enabled));
  }

  /**
   * Sends a command to the comm adapter.
   *
   * @param command The command
   */
  private void sendAdapterCommand(AdapterCommand command) {
    try {
      callWrapper.call(() -> vehicleService.sendCommAdapterCommand(processModel.getVehicleRef(),
                                                                   command));
    }
    catch (Exception ex) {
      LOG.warn("Error sending comm adapter command '{}'", command, ex);
    }
  }

  /**
   * Enables or disables the comm adapter.
   *
   * @param enable Whether the comm adapter should be enabled or disabled
   */
  private void enableCommAdapter(boolean enable) {
    try {
      if (enable) {
        callWrapper.call(() -> vehicleService.enableCommAdapter(processModel.getVehicleRef()));
      }
      else {
        callWrapper.call(() -> vehicleService.disableCommAdapter(processModel.getVehicleRef()));
      }
    }
    catch (Exception ex) {
      LOG.warn("Error enabling/disabling comm adapter", ex);
    }
  }

  /**
   * This method is called from within the constructor to
   * initialize the form.
   * WARNING: Do NOT modify this code. The content of this method is
   * always regenerated by the Form Editor.
   */
  @SuppressWarnings("unchecked")
  // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
  private void initComponents() {
    java.awt.GridBagConstraints gridBagConstraints;

    connectionSettingsPanel = new javax.swing.JPanel();
    enableAdapterCheckBox = new javax.swing.JCheckBox();
    hostLabel = new javax.swing.JLabel();
    hostTextField = new javax.swing.JTextField();
    connectedButton = new javax.swing.JButton();
    portLabel = new javax.swing.JLabel();
    portTextField = new javax.swing.JTextField();
    activeButton = new javax.swing.JButton();
    aliveTimeoutLable = new javax.swing.JLabel();
    aliveTimeoutTextField = new javax.swing.JTextField();
    disconnectOnTimeoutChkBox = new javax.swing.JCheckBox();
    reconnectOnConnectionLossChkBox = new javax.swing.JCheckBox();
    enableLoggingChkBox = new javax.swing.JCheckBox();
    setOrderPanel = new javax.swing.JPanel();
    destinationLabel = new javax.swing.JLabel();
    destinationComboBox = new javax.swing.JComboBox<>();
    actionLabel = new javax.swing.JLabel();
    actionComboBox = new javax.swing.JComboBox<>();
    actionParamPanel = new javax.swing.JPanel();
    orderIdLabel = new javax.swing.JLabel();
    orderIdTextField = new javax.swing.JTextField();
    sendOrderButton = new javax.swing.JButton();
    repeatOrderPanel = new javax.swing.JPanel();
    lastOrdersScrollPane = new javax.swing.JScrollPane();
    lastOrdersList = new javax.swing.JList<>();
    lastOrderDetailsPanel = new javax.swing.JPanel();
    destinationLabel1 = new javax.swing.JLabel();
    lastDestinationTextField = new javax.swing.JTextField();
    actionLabel1 = new javax.swing.JLabel();
    lastActionTextField = new javax.swing.JTextField();
    lastOrderIdLabel = new javax.swing.JLabel();
    lastOrderIdTextField = new javax.swing.JTextField();
    repeatLastOrderButton = new javax.swing.JButton();
    fillingLabel1 = new javax.swing.JLabel();

    setLayout(new java.awt.GridBagLayout());

    java.util.ResourceBundle bundle = java.util.ResourceBundle.getBundle("de/fraunhofer/iml/opentcs/example/commadapter/vehicle/Bundle"); // NOI18N
    connectionSettingsPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, bundle.getString("VehicleConnectionTitle"), javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Tahoma", 1, 11))); // NOI18N
    connectionSettingsPanel.setLayout(new java.awt.GridBagLayout());

    enableAdapterCheckBox.setText(bundle.getString("EnableAdapter")); // NOI18N
    enableAdapterCheckBox.addActionListener(new java.awt.event.ActionListener() {
      public void actionPerformed(java.awt.event.ActionEvent evt) {
        enableAdapterCheckBoxActionPerformed(evt);
      }
    });
    connectionSettingsPanel.add(enableAdapterCheckBox, new java.awt.GridBagConstraints());

    hostLabel.setText(bundle.getString("VehicleIpAddress")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
    gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
    connectionSettingsPanel.add(hostLabel, gridBagConstraints);

    hostTextField.setColumns(12);
    hostTextField.setText("XXX.XXX.XXX.XXX");
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 2;
    gridBagConstraints.gridy = 0;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
    gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
    connectionSettingsPanel.add(hostTextField, gridBagConstraints);

    connectedButton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/de/fraunhofer/iml/opentcs/example/commadapter/res/symbols/LEDGray.gif"))); // NOI18N
    connectedButton.setText(bundle.getString("AdapterConnected")); // NOI18N
    connectedButton.setBorderPainted(false);
    connectedButton.setDisabledIcon(new javax.swing.ImageIcon(getClass().getResource("/de/fraunhofer/iml/opentcs/example/commadapter/res/symbols/LEDRed.gif"))); // NOI18N
    connectedButton.setDisabledSelectedIcon(new javax.swing.ImageIcon(getClass().getResource("/de/fraunhofer/iml/opentcs/example/commadapter/res/symbols/LEDGreen.gif"))); // NOI18N
    connectedButton.setEnabled(false);
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 0;
    gridBagConstraints.gridy = 1;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
    connectionSettingsPanel.add(connectedButton, gridBagConstraints);

    portLabel.setText(bundle.getString("VehicleTcpPort")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 1;
    gridBagConstraints.gridy = 1;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
    gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
    connectionSettingsPanel.add(portLabel, gridBagConstraints);

    portTextField.setColumns(6);
    portTextField.setDocument(new PlainDocument(){
      @Override
      public void insertString(int offs, String str, AttributeSet a)
      throws BadLocationException {
        try {
          String oldString = getText(0, getLength());
          StringBuilder newString = new StringBuilder(oldString);
          newString.insert(offs, str);
          int newValue = Integer.parseInt(newString.toString());
          if (newValue >= 1 && newValue <= 65535) {
            super.insertString(offs, str, a);
          }
        }
        catch(NumberFormatException e) { }
      }
    }
  );
  portTextField.setText("XXXXX");
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 2;
  gridBagConstraints.gridy = 1;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  connectionSettingsPanel.add(portTextField, gridBagConstraints);

  activeButton.setIcon(new javax.swing.ImageIcon(getClass().getResource("/de/fraunhofer/iml/opentcs/example/commadapter/res/symbols/LEDGray.gif"))); // NOI18N
  activeButton.setText(bundle.getString("ControlPanel.AdapterActive")); // NOI18N
  activeButton.setBorderPainted(false);
  activeButton.setDisabledIcon(new javax.swing.ImageIcon(getClass().getResource("/de/fraunhofer/iml/opentcs/example/commadapter/res/symbols/LEDRed.gif"))); // NOI18N
  activeButton.setDisabledSelectedIcon(new javax.swing.ImageIcon(getClass().getResource("/de/fraunhofer/iml/opentcs/example/commadapter/res/symbols/LEDGreen.gif"))); // NOI18N
  activeButton.setEnabled(false);
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 2;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  connectionSettingsPanel.add(activeButton, gridBagConstraints);

  aliveTimeoutLable.setText(bundle.getString("ControlPanel.IdleAfter")); // NOI18N
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 1;
  gridBagConstraints.gridy = 2;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
  gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
  connectionSettingsPanel.add(aliveTimeoutLable, gridBagConstraints);

  aliveTimeoutTextField.setColumns(6);
  aliveTimeoutTextField.setDocument(new PlainDocument(){
    @Override
    public void insertString(int offs, String str, AttributeSet a)
    throws BadLocationException {
      try {
        Integer.parseInt(str);
        super.insertString(offs, str, a);
      }
      catch(NumberFormatException e) { }
    }
  });
  aliveTimeoutTextField.setText("XXXXX");
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 2;
  gridBagConstraints.gridy = 2;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  connectionSettingsPanel.add(aliveTimeoutTextField, gridBagConstraints);

  disconnectOnTimeoutChkBox.setText(bundle.getString("ControlPanel.DisconnectOnTimeout")); // NOI18N
  disconnectOnTimeoutChkBox.addActionListener(new java.awt.event.ActionListener() {
    public void actionPerformed(java.awt.event.ActionEvent evt) {
      disconnectOnTimeoutChkBoxActionPerformed(evt);
    }
  });
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 3;
  gridBagConstraints.gridwidth = 2;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  connectionSettingsPanel.add(disconnectOnTimeoutChkBox, gridBagConstraints);

  reconnectOnConnectionLossChkBox.setText(bundle.getString("ControlPanel.DisconnectOnConnectionLoss")); // NOI18N
  reconnectOnConnectionLossChkBox.addActionListener(new java.awt.event.ActionListener() {
    public void actionPerformed(java.awt.event.ActionEvent evt) {
      reconnectOnConnectionLossChkBoxActionPerformed(evt);
    }
  });
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 4;
  gridBagConstraints.gridwidth = 2;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  connectionSettingsPanel.add(reconnectOnConnectionLossChkBox, gridBagConstraints);

  enableLoggingChkBox.setText(bundle.getString("ControlPanel.EnableLogging")); // NOI18N
  enableLoggingChkBox.addActionListener(new java.awt.event.ActionListener() {
    public void actionPerformed(java.awt.event.ActionEvent evt) {
      enableLoggingChkBoxActionPerformed(evt);
    }
  });
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 5;
  gridBagConstraints.gridwidth = 2;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  connectionSettingsPanel.add(enableLoggingChkBox, gridBagConstraints);

  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 0;
  gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
  gridBagConstraints.insets = new java.awt.Insets(6, 6, 6, 6);
  add(connectionSettingsPanel, gridBagConstraints);

  setOrderPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, bundle.getString("NewOrderTelegramTitle"), javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Tahoma", 1, 11))); // NOI18N
  setOrderPanel.setLayout(new java.awt.GridBagLayout());

  destinationLabel.setText(bundle.getString("OrderDestination")); // NOI18N
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 0;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
  gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
  setOrderPanel.add(destinationLabel, gridBagConstraints);

  destinationComboBox.setEditable(true);
  destinationComboBox.setEditor(new TCSObjectComboBoxEditor());
  destinationComboBox.setRenderer(new StringListCellRenderer<Point>(point -> point.getName()));
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 1;
  gridBagConstraints.gridy = 0;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints.insets = new java.awt.Insets(0, 0, 3, 0);
  setOrderPanel.add(destinationComboBox, gridBagConstraints);

  actionLabel.setText(bundle.getString("OrderAction")); // NOI18N
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 9;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
  gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
  setOrderPanel.add(actionLabel, gridBagConstraints);

  actionComboBox.setPrototypeDisplayValue("1234567890");
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 1;
  gridBagConstraints.gridy = 9;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints.insets = new java.awt.Insets(0, 0, 3, 0);
  setOrderPanel.add(actionComboBox, gridBagConstraints);

  actionParamPanel.setLayout(new java.awt.GridBagLayout());
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 1;
  gridBagConstraints.gridy = 10;
  gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
  setOrderPanel.add(actionParamPanel, gridBagConstraints);

  orderIdLabel.setText(bundle.getString("OrderID")); // NOI18N
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 11;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
  gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
  setOrderPanel.add(orderIdLabel, gridBagConstraints);

  orderIdTextField.setColumns(6);
  orderIdTextField.setText("1");
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 1;
  gridBagConstraints.gridy = 11;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints.insets = new java.awt.Insets(0, 0, 3, 0);
  setOrderPanel.add(orderIdTextField, gridBagConstraints);

  sendOrderButton.setText(bundle.getString("SendNewOrder")); // NOI18N
  sendOrderButton.addActionListener(new java.awt.event.ActionListener() {
    public void actionPerformed(java.awt.event.ActionEvent evt) {
      sendOrderButtonActionPerformed(evt);
    }
  });
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 12;
  gridBagConstraints.gridwidth = 2;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  setOrderPanel.add(sendOrderButton, gridBagConstraints);

  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 1;
  gridBagConstraints.gridy = 0;
  gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
  gridBagConstraints.insets = new java.awt.Insets(6, 6, 6, 6);
  add(setOrderPanel, gridBagConstraints);

  repeatOrderPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, bundle.getString("LastOrderTelegramTitle"), javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Tahoma", 1, 11))); // NOI18N
  repeatOrderPanel.setMinimumSize(new java.awt.Dimension(199, 170));

  lastOrdersList.setModel(lastOrderListModel);
  lastOrdersList.setSelectionMode(javax.swing.ListSelectionModel.SINGLE_INTERVAL_SELECTION);
  lastOrdersList.setCellRenderer(new OrderListCellRenderer());
  lastOrdersList.setPrototypeCellValue(OrderListCellRenderer.PROTOTYPE_TELEGRAM);
  lastOrdersScrollPane.setViewportView(lastOrdersList);

  lastOrdersList.addListSelectionListener(new javax.swing.event.ListSelectionListener() {
    public void valueChanged(javax.swing.event.ListSelectionEvent evt) {
      lastOrdersSentValueChanged(evt);
    }
  });

  lastOrdersScrollPane.setViewportView(lastOrdersList);

  lastOrderDetailsPanel.setLayout(new java.awt.GridBagLayout());

  destinationLabel1.setText(bundle.getString("OrderDestination")); // NOI18N
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 0;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  lastOrderDetailsPanel.add(destinationLabel1, gridBagConstraints);

  lastDestinationTextField.setEditable(false);
  lastDestinationTextField.setColumns(12);
  lastDestinationTextField.setHorizontalAlignment(javax.swing.JTextField.RIGHT);
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 1;
  gridBagConstraints.gridy = 0;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  lastOrderDetailsPanel.add(lastDestinationTextField, gridBagConstraints);

  actionLabel1.setText(bundle.getString("OrderAction")); // NOI18N
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 9;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  lastOrderDetailsPanel.add(actionLabel1, gridBagConstraints);

  lastActionTextField.setEditable(false);
  lastActionTextField.setColumns(12);
  lastActionTextField.setHorizontalAlignment(javax.swing.JTextField.RIGHT);
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 1;
  gridBagConstraints.gridy = 9;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  lastOrderDetailsPanel.add(lastActionTextField, gridBagConstraints);

  lastOrderIdLabel.setText(bundle.getString("OrderID")); // NOI18N
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 11;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  lastOrderDetailsPanel.add(lastOrderIdLabel, gridBagConstraints);

  lastOrderIdTextField.setEditable(false);
  lastOrderIdTextField.setColumns(6);
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 1;
  gridBagConstraints.gridy = 11;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  lastOrderDetailsPanel.add(lastOrderIdTextField, gridBagConstraints);

  repeatLastOrderButton.setText(bundle.getString("SendLastOrderAgain")); // NOI18N
  repeatLastOrderButton.setEnabled(false);
  repeatLastOrderButton.addActionListener(new java.awt.event.ActionListener() {
    public void actionPerformed(java.awt.event.ActionEvent evt) {
      repeatLastOrderButtonActionPerformed(evt);
    }
  });
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 12;
  gridBagConstraints.gridwidth = 2;
  gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
  lastOrderDetailsPanel.add(repeatLastOrderButton, gridBagConstraints);

  javax.swing.GroupLayout repeatOrderPanelLayout = new javax.swing.GroupLayout(repeatOrderPanel);
  repeatOrderPanel.setLayout(repeatOrderPanelLayout);
  repeatOrderPanelLayout.setHorizontalGroup(
    repeatOrderPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
    .addGroup(repeatOrderPanelLayout.createSequentialGroup()
      .addComponent(lastOrdersScrollPane, javax.swing.GroupLayout.PREFERRED_SIZE, 166, javax.swing.GroupLayout.PREFERRED_SIZE)
      .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
      .addComponent(lastOrderDetailsPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
      .addGap(0, 0, 0))
  );
  repeatOrderPanelLayout.setVerticalGroup(
    repeatOrderPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
    .addGroup(repeatOrderPanelLayout.createSequentialGroup()
      .addGroup(repeatOrderPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
        .addComponent(lastOrdersScrollPane)
        .addComponent(lastOrderDetailsPanel, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
      .addGap(0, 0, 0))
  );

  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 1;
  gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
  gridBagConstraints.anchor = java.awt.GridBagConstraints.NORTHWEST;
  gridBagConstraints.insets = new java.awt.Insets(6, 6, 6, 6);
  add(repeatOrderPanel, gridBagConstraints);

  fillingLabel1.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
  gridBagConstraints = new java.awt.GridBagConstraints();
  gridBagConstraints.gridx = 0;
  gridBagConstraints.gridy = 3;
  gridBagConstraints.gridwidth = 2;
  gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
  gridBagConstraints.weightx = 1.0;
  gridBagConstraints.weighty = 1.0;
  add(fillingLabel1, gridBagConstraints);

  getAccessibleContext().setAccessibleName(bundle.getString("ControlsPanelTitle")); // NOI18N
  }// </editor-fold>//GEN-END:initComponents

  private void repeatLastOrderButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_repeatLastOrderButtonActionPerformed
    // If there's only one old telegram send this
    OrderRequest lastOrderSent = processModel.getLastOrderSent();
    if (lastOrderSent != null && lastOrdersList.getModel().getSize() == 1) {
      sendAdapterCommand(new SendRequestCommand(lastOrderSent));
      lastOrdersList.setSelectedIndex(0);
    }
    // Otherwise if there is an order selected send this
    else if (!lastOrdersList.isSelectionEmpty()) {
      sendAdapterCommand(new SendRequestCommand(lastOrdersList.getSelectedValue()));
      lastOrdersList.setSelectedIndex(0);
    }
    else {
      // This should never happen
      JOptionPane.showMessageDialog(this, "There's no last order to repeat.");
    }
  }//GEN-LAST:event_repeatLastOrderButtonActionPerformed

  private void sendOrderButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_sendOrderButtonActionPerformed
    // Parse and check destination
    Object selectedItem = destinationComboBox.getSelectedItem();
    String destinationName = selectedItem instanceof Point
        ? ((Point) selectedItem).getName() : selectedItem.toString();
    if (destinationName.length() > 8) {
      JOptionPane.showMessageDialog(this, "Destination must not be longer than 8 characters.");
      return;
    }
    int destinationId = 0;
    try {
      destinationId = Integer.parseInt(destinationName);
    }
    catch (NumberFormatException e) {
      JOptionPane.showMessageDialog(this, "Destination ID must be in [0..65535].");
      return;
    }
    // Parse and check action
    OrderAction action = OrderAction.NONE;
    String selectedAction = actionComboBox.getSelectedItem().toString();
    if (!Strings.isNullOrEmpty(selectedAction)) {
      action = OrderAction.valueOf(selectedAction);
    }
    // Parse and check order ID.
    int orderId;
    try {
      orderId = Integer.parseUnsignedInt(orderIdTextField.getText());
      if (orderId > 65535) {
        throw new NumberFormatException("orderId out of bounds");
      }
    }
    catch (NumberFormatException exc) {
      JOptionPane.showMessageDialog(this, "Order ID must be in [0..65535].");
      return;
    }

    OrderRequest telegram = new OrderRequest(0,
                                             orderId,
                                             destinationId,
                                             action);
    // Send this telegram to vehicle.
    SendRequestCommand command = new SendRequestCommand(telegram);
    sendAdapterCommand(command);

    lastOrdersList.setSelectedIndex(0);

    // Increment the order ID in case the user wants to send another one.
    orderId = (orderId + 1) % 65535;
    if (orderId == 0) {
      orderId = 1;
    }
    orderIdTextField.setText(Integer.toString(orderId));
  }//GEN-LAST:event_sendOrderButtonActionPerformed

  private void enableAdapterCheckBoxActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_enableAdapterCheckBoxActionPerformed
    if (enableAdapterCheckBox.isSelected()) {
      String host = hostTextField.getText();
      if (Strings.isNullOrEmpty(host)) {
        enableAdapterCheckBox.setSelected(false);
        JOptionPane.showMessageDialog(this, "IP address can not be empty.");
        return;
      }

      String portString = portTextField.getText();
      int port;
      if (!Strings.isNullOrEmpty(portString)) {
        port = Integer.valueOf(portString);
      }
      else {
        enableAdapterCheckBox.setSelected(false);
        JOptionPane.showMessageDialog(this, "TCP port can not be empty.");
        return;
      }

      String timeoutString = aliveTimeoutTextField.getText();
      int timeout;
      if (!Strings.isNullOrEmpty(timeoutString)) {
        timeout = Integer.valueOf(aliveTimeoutTextField.getText());
      }
      else {
        enableAdapterCheckBox.setSelected(false);
        JOptionPane.showMessageDialog(this, "Inactive after (ms) can not be empty.");
        return;
      }

      sendAdapterCommand(new SetVehicleHostCommand(host));
      sendAdapterCommand(new SetVehiclePortCommand(port));
      sendAdapterCommand(new SetIdleTimeoutCommand(timeout));
    }

    enableCommAdapter(enableAdapterCheckBox.isSelected());
  }//GEN-LAST:event_enableAdapterCheckBoxActionPerformed

  private void disconnectOnTimeoutChkBoxActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_disconnectOnTimeoutChkBoxActionPerformed
    sendAdapterCommand(new SetDisconnectingOnVehicleIdleCommand(disconnectOnTimeoutChkBox.isSelected()));
  }//GEN-LAST:event_disconnectOnTimeoutChkBoxActionPerformed

  private void reconnectOnConnectionLossChkBoxActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_reconnectOnConnectionLossChkBoxActionPerformed
    sendAdapterCommand(new SetReconnectingOnConnectionLossCommand(reconnectOnConnectionLossChkBox.isSelected()));
  }//GEN-LAST:event_reconnectOnConnectionLossChkBoxActionPerformed

  private void enableLoggingChkBoxActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_enableLoggingChkBoxActionPerformed
    sendAdapterCommand(new SetLoggingEnabledCommand(enableLoggingChkBox.isSelected()));
  }//GEN-LAST:event_enableLoggingChkBoxActionPerformed

  /**
   * Updates the panel containing informations about the last order sent with the selected order
   * from the list.
   *
   * @param evt The selection event for the order list
   */
  private void lastOrdersSentValueChanged(javax.swing.event.ListSelectionEvent evt) {
    OrderRequest selectedTelegram = lastOrdersList.getSelectedValue();
    updateLastOrderTextFields(selectedTelegram);
  }

  /**
   * Updates the last order text fields with the content of the given order request.
   *
   * @param telegram The order request
   */
  private void updateLastOrderTextFields(OrderRequest telegram) {
    if (telegram != null) {
      lastDestinationTextField.setText(String.valueOf(telegram.getDestinationId()));
      lastActionTextField.setText(telegram.getDestinationAction().toString());
      lastOrderIdTextField.setText(Integer.toString(telegram.getId()));
      repeatLastOrderButton.setEnabled(processModel.isCommAdapterConnected());
    }
    else {
      lastDestinationTextField.setText("-");
      lastActionTextField.setText("-");
      lastOrderIdTextField.setText("-");
      repeatLastOrderButton.setEnabled(false);
    }
  }

  // Variables declaration - do not modify//GEN-BEGIN:variables
  private javax.swing.JComboBox<String> actionComboBox;
  private javax.swing.JLabel actionLabel;
  private javax.swing.JLabel actionLabel1;
  private javax.swing.JPanel actionParamPanel;
  private javax.swing.JButton activeButton;
  private javax.swing.JLabel aliveTimeoutLable;
  private javax.swing.JTextField aliveTimeoutTextField;
  private javax.swing.JButton connectedButton;
  private javax.swing.JPanel connectionSettingsPanel;
  private javax.swing.JComboBox<Point> destinationComboBox;
  private javax.swing.JLabel destinationLabel;
  private javax.swing.JLabel destinationLabel1;
  private javax.swing.JCheckBox disconnectOnTimeoutChkBox;
  private javax.swing.JCheckBox enableAdapterCheckBox;
  private javax.swing.JCheckBox enableLoggingChkBox;
  private javax.swing.JLabel fillingLabel1;
  private javax.swing.JLabel hostLabel;
  private javax.swing.JTextField hostTextField;
  private javax.swing.JTextField lastActionTextField;
  private javax.swing.JTextField lastDestinationTextField;
  private javax.swing.JPanel lastOrderDetailsPanel;
  private javax.swing.JLabel lastOrderIdLabel;
  private javax.swing.JTextField lastOrderIdTextField;
  private javax.swing.JList<OrderRequest> lastOrdersList;
  private javax.swing.JScrollPane lastOrdersScrollPane;
  private javax.swing.JLabel orderIdLabel;
  private javax.swing.JTextField orderIdTextField;
  private javax.swing.JLabel portLabel;
  private javax.swing.JTextField portTextField;
  private javax.swing.JCheckBox reconnectOnConnectionLossChkBox;
  private javax.swing.JButton repeatLastOrderButton;
  private javax.swing.JPanel repeatOrderPanel;
  private javax.swing.JButton sendOrderButton;
  private javax.swing.JPanel setOrderPanel;
  // End of variables declaration//GEN-END:variables
}
