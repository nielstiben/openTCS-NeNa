/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.exchange;

import com.google.inject.assistedinject.Assisted;
import nl.saxion.nena.opentcs.commadapter.ros2.ExampleProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SendRequestCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SetPeriodicStateRequestEnabledCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands.SetStateRequestIntervalCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateRequest;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateResponse;
import java.util.Objects;
import static java.util.Objects.requireNonNull;
import java.util.ResourceBundle;
import javax.inject.Inject;
import javax.swing.JOptionPane;
import javax.swing.SwingUtilities;

import org.opentcs.components.kernel.services.VehicleService;
import org.opentcs.customizations.ServiceCallWrapper;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.opentcs.drivers.vehicle.management.VehicleCommAdapterPanel;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.CallWrapper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 *
 * @author Mats Wilhelm (Fraunhofer IML)
 */
public class StatusPanel
    extends VehicleCommAdapterPanel {

  private static final Logger LOG = LoggerFactory.getLogger(StatusPanel.class);
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
   * @param processModel The comm adapter's process model.
   * @param vehicleService The vehicle service.
   * @param callWrapper The call wrapper to use for service calls.
   */
  @Inject
  public StatusPanel(@Assisted ExampleProcessModelTO processModel,
                     @Assisted VehicleService vehicleService,
                     @ServiceCallWrapper CallWrapper callWrapper) {
    this.processModel = requireNonNull(processModel, "processModel");
    this.vehicleService = requireNonNull(vehicleService, "vehicleService");
    this.callWrapper = requireNonNull(callWrapper, "callWrapper");

    initComponents();
    initGuiContent();
  }

  /**
   * Sets the initial content for each attribute of the process model.
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
                       VehicleProcessModel.Attribute.COMM_ADAPTER_CONNECTED.name())) {
      updateCommAdapterConnected(processModel.isCommAdapterConnected());
    }
    else if (Objects.equals(attributeChanged,
                            ExampleProcessModel.Attribute.CURRENT_STATE.name())) {
      updateCurrentState(processModel.getCurrentState());
    }
    else if (Objects.equals(attributeChanged,
                            ExampleProcessModel.Attribute.PERIODIC_STATE_REQUESTS_ENABLED.name())) {
      updatePeriodicStateRequestsEnabled(processModel.isPeriodicStateRequestEnabled(),
                                         processModel.isCommAdapterConnected());
    }
    else if (Objects.equals(attributeChanged,
                            ExampleProcessModel.Attribute.PERIOD_STATE_REQUESTS_INTERVAL.name())) {
      updateStateRequestInterval(processModel.getStateRequestInterval());
    }
  }

  /**
   * Updates the status panel when the connection to the vehicle changes.
   *
   * @param connected Whether a connection to the vehicle is established
   */
  private void updateCommAdapterConnected(boolean connected) {
    buttonGetState.setEnabled(connected);
  }

  /**
   * Updates the status panel with the given state response.
   *
   * @param stateTelegram The state response
   */
  private void updateCurrentState(StateResponse stateTelegram) {
    SwingUtilities.invokeLater(() -> {
      updateStatusPanel(stateTelegram);
    });
  }

  /**
   * Updates checkboxes to reflect changes on the state of periodic state request.
   *
   * @param requestsEnabled Whether periodic requests are enabled
   * @param connected Whether a connection to the vehicle is established
   */
  private void updatePeriodicStateRequestsEnabled(boolean requestsEnabled, boolean connected) {
    SwingUtilities.invokeLater(() -> {
      chkBoxEnablePeriodicGetState.setSelected(requestsEnabled);
      txtGetStateInterval.setEditable(!requestsEnabled);
      buttonGetState.setEnabled(!requestsEnabled && connected);
    });
  }

  /**
   * Updates the get state interval textfield with the given interval.
   *
   * @param interval The new interval
   */
  private void updateStateRequestInterval(int interval) {
    SwingUtilities.invokeLater(() -> txtGetStateInterval.setText(Integer.toString(interval)));
  }

  /**
   * Updates the status panel with the content of a state response.
   *
   * @param state The state response
   */
  private void updateStatusPanel(final StateResponse state) {
    requireNonNull(state, "state");
    textFieldStateTeleCount.setText(String.valueOf(state.getCurrentOrderId()));
    textFieldStateOS.setText(String.valueOf(state.getOperatingState()));
    textFieldStateLS.setText(String.valueOf(state.getLoadState()));
    textFieldStateCRP.setText(String.valueOf(state.getPositionId()));
    textFieldLastOrderID.setText(String.valueOf(state.getLastFinishedOrderId()));
    textFieldCurrOrderID.setText(String.valueOf(state.getCurrentOrderId()));
    textFieldLastRcvdOrderID.setText(String.valueOf(state.getLastReceivedOrderId()));
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

    panelPeriodicStateRequests = new javax.swing.JPanel();
    chkBoxEnablePeriodicGetState = new javax.swing.JCheckBox();
    labelGetStateInterval = new javax.swing.JLabel();
    txtGetStateInterval = new javax.swing.JTextField();
    panelManualStateRequest = new javax.swing.JPanel();
    buttonGetState = new javax.swing.JButton();
    panelState = new javax.swing.JPanel();
    panelTelegramContent = new javax.swing.JPanel();
    labelStateTeleCount = new javax.swing.JLabel();
    textFieldStateTeleCount = new javax.swing.JTextField();
    labelStateOS = new javax.swing.JLabel();
    textFieldStateOS = new javax.swing.JTextField();
    labelStateLS = new javax.swing.JLabel();
    textFieldStateLS = new javax.swing.JTextField();
    labelStateCRP = new javax.swing.JLabel();
    textFieldStateCRP = new javax.swing.JTextField();
    fillingLabel2 = new javax.swing.JLabel();
    labelLastOrderID = new javax.swing.JLabel();
    textFieldLastOrderID = new javax.swing.JTextField();
    labelCurrOrderID = new javax.swing.JLabel();
    textFieldCurrOrderID = new javax.swing.JTextField();
    labelLastRcvdOrderID = new javax.swing.JLabel();
    textFieldLastRcvdOrderID = new javax.swing.JTextField();
    fillingLabel1 = new javax.swing.JLabel();

    setLayout(new java.awt.GridBagLayout());

    java.util.ResourceBundle bundle = java.util.ResourceBundle.getBundle("de/fraunhofer/iml/opentcs/example/commadapter/vehicle/Bundle"); // NOI18N
    panelPeriodicStateRequests.setBorder(javax.swing.BorderFactory.createTitledBorder(null, bundle.getString("PeriodicStateRequestsTitle"), javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Tahoma", 1, 11))); // NOI18N
    panelPeriodicStateRequests.setMaximumSize(new java.awt.Dimension(2147483647, 70));
    panelPeriodicStateRequests.setLayout(new java.awt.GridBagLayout());

    chkBoxEnablePeriodicGetState.setSelected(true);
    chkBoxEnablePeriodicGetState.setText(bundle.getString("EnablePeriodicStateRequests")); // NOI18N
    chkBoxEnablePeriodicGetState.addActionListener(new java.awt.event.ActionListener() {
      public void actionPerformed(java.awt.event.ActionEvent evt) {
        chkBoxEnablePeriodicGetStateActionPerformed(evt);
      }
    });
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridwidth = 2;
    gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
    panelPeriodicStateRequests.add(chkBoxEnablePeriodicGetState, gridBagConstraints);

    labelGetStateInterval.setText(bundle.getString("StateRequestInterval")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 0;
    gridBagConstraints.gridy = 1;
    gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
    panelPeriodicStateRequests.add(labelGetStateInterval, gridBagConstraints);

    txtGetStateInterval.setEditable(false);
    txtGetStateInterval.setColumns(4);
    txtGetStateInterval.setHorizontalAlignment(javax.swing.JTextField.RIGHT);
    txtGetStateInterval.setText("500");
    txtGetStateInterval.setMinimumSize(new java.awt.Dimension(38, 20));
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 1;
    gridBagConstraints.gridy = 1;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
    gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 3);
    panelPeriodicStateRequests.add(txtGetStateInterval, gridBagConstraints);

    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 0;
    gridBagConstraints.gridy = 0;
    gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
    gridBagConstraints.weightx = 1.0;
    add(panelPeriodicStateRequests, gridBagConstraints);

    panelManualStateRequest.setBorder(javax.swing.BorderFactory.createTitledBorder(null, bundle.getString("ManualStateRequestsTitle"), javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Tahoma", 1, 11))); // NOI18N
    panelManualStateRequest.setLayout(new java.awt.CardLayout());

    buttonGetState.setText(bundle.getString("SendStateRequest")); // NOI18N
    buttonGetState.setEnabled(false);
    buttonGetState.addActionListener(new java.awt.event.ActionListener() {
      public void actionPerformed(java.awt.event.ActionEvent evt) {
        buttonGetStateActionPerformed(evt);
      }
    });
    panelManualStateRequest.add(buttonGetState, "card2");

    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 1;
    gridBagConstraints.gridy = 0;
    gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
    gridBagConstraints.weightx = 1.0;
    add(panelManualStateRequest, gridBagConstraints);

    panelState.setBorder(javax.swing.BorderFactory.createEmptyBorder(1, 1, 1, 1));
    panelState.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
    panelState.setLayout(new java.awt.BorderLayout());

    panelTelegramContent.setBorder(javax.swing.BorderFactory.createTitledBorder(null, bundle.getString("LastReportedState"), javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Tahoma", 1, 11))); // NOI18N
    panelTelegramContent.setLayout(new java.awt.GridBagLayout());

    labelStateTeleCount.setText(bundle.getString("OrderID")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 0;
    gridBagConstraints.gridy = 0;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
    gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
    panelTelegramContent.add(labelStateTeleCount, gridBagConstraints);

    textFieldStateTeleCount.setEditable(false);
    textFieldStateTeleCount.setBackground(new java.awt.Color(255, 255, 204));
    textFieldStateTeleCount.setColumns(6);
    textFieldStateTeleCount.setFont(new java.awt.Font("Monospaced", 0, 11)); // NOI18N
    textFieldStateTeleCount.setHorizontalAlignment(javax.swing.JTextField.TRAILING);
    textFieldStateTeleCount.setText("00000");
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 1;
    gridBagConstraints.gridy = 0;
    gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.SOUTH;
    gridBagConstraints.insets = new java.awt.Insets(3, 0, 0, 0);
    panelTelegramContent.add(textFieldStateTeleCount, gridBagConstraints);

    labelStateOS.setText(bundle.getString("OperationState")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 0;
    gridBagConstraints.gridy = 2;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
    gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
    panelTelegramContent.add(labelStateOS, gridBagConstraints);

    textFieldStateOS.setEditable(false);
    textFieldStateOS.setBackground(new java.awt.Color(255, 255, 204));
    textFieldStateOS.setColumns(1);
    textFieldStateOS.setFont(new java.awt.Font("Monospaced", 0, 11)); // NOI18N
    textFieldStateOS.setHorizontalAlignment(javax.swing.JTextField.TRAILING);
    textFieldStateOS.setText("X");
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 1;
    gridBagConstraints.gridy = 2;
    gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
    gridBagConstraints.insets = new java.awt.Insets(4, 0, 1, 0);
    panelTelegramContent.add(textFieldStateOS, gridBagConstraints);

    labelStateLS.setText(bundle.getString("LoadState")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 2;
    gridBagConstraints.gridy = 2;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
    gridBagConstraints.insets = new java.awt.Insets(0, 6, 0, 3);
    panelTelegramContent.add(labelStateLS, gridBagConstraints);

    textFieldStateLS.setEditable(false);
    textFieldStateLS.setBackground(new java.awt.Color(255, 255, 204));
    textFieldStateLS.setColumns(1);
    textFieldStateLS.setFont(new java.awt.Font("Monospaced", 0, 11)); // NOI18N
    textFieldStateLS.setHorizontalAlignment(javax.swing.JTextField.TRAILING);
    textFieldStateLS.setText("X");
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 3;
    gridBagConstraints.gridy = 2;
    gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
    gridBagConstraints.insets = new java.awt.Insets(4, 0, 1, 0);
    panelTelegramContent.add(textFieldStateLS, gridBagConstraints);

    labelStateCRP.setText(bundle.getString("currentPoint")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 4;
    gridBagConstraints.gridy = 2;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
    gridBagConstraints.insets = new java.awt.Insets(0, 6, 0, 3);
    panelTelegramContent.add(labelStateCRP, gridBagConstraints);

    textFieldStateCRP.setEditable(false);
    textFieldStateCRP.setBackground(new java.awt.Color(255, 255, 204));
    textFieldStateCRP.setColumns(9);
    textFieldStateCRP.setFont(new java.awt.Font("Monospaced", 0, 11)); // NOI18N
    textFieldStateCRP.setHorizontalAlignment(javax.swing.JTextField.TRAILING);
    textFieldStateCRP.setText("CURPOINT");
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 5;
    gridBagConstraints.gridy = 2;
    gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.SOUTH;
    gridBagConstraints.insets = new java.awt.Insets(3, 0, 0, 0);
    panelTelegramContent.add(textFieldStateCRP, gridBagConstraints);
    panelTelegramContent.add(fillingLabel2, new java.awt.GridBagConstraints());

    labelLastOrderID.setText(bundle.getString("LastFinishedOrder")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 0;
    gridBagConstraints.gridy = 5;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
    gridBagConstraints.insets = new java.awt.Insets(0, 0, 3, 3);
    panelTelegramContent.add(labelLastOrderID, gridBagConstraints);

    textFieldLastOrderID.setEditable(false);
    textFieldLastOrderID.setBackground(new java.awt.Color(255, 255, 204));
    textFieldLastOrderID.setColumns(6);
    textFieldLastOrderID.setFont(new java.awt.Font("Monospaced", 0, 11)); // NOI18N
    textFieldLastOrderID.setHorizontalAlignment(javax.swing.JTextField.TRAILING);
    textFieldLastOrderID.setText("00000");
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 1;
    gridBagConstraints.gridy = 5;
    gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.SOUTH;
    gridBagConstraints.insets = new java.awt.Insets(3, 0, 0, 0);
    panelTelegramContent.add(textFieldLastOrderID, gridBagConstraints);

    labelCurrOrderID.setText(bundle.getString("currentOrderID")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 2;
    gridBagConstraints.gridy = 5;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
    gridBagConstraints.insets = new java.awt.Insets(0, 6, 0, 3);
    panelTelegramContent.add(labelCurrOrderID, gridBagConstraints);

    textFieldCurrOrderID.setEditable(false);
    textFieldCurrOrderID.setBackground(new java.awt.Color(255, 255, 204));
    textFieldCurrOrderID.setColumns(6);
    textFieldCurrOrderID.setFont(new java.awt.Font("Monospaced", 0, 11)); // NOI18N
    textFieldCurrOrderID.setHorizontalAlignment(javax.swing.JTextField.TRAILING);
    textFieldCurrOrderID.setText("11111");
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 3;
    gridBagConstraints.gridy = 5;
    gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
    gridBagConstraints.insets = new java.awt.Insets(3, 0, 0, 0);
    panelTelegramContent.add(textFieldCurrOrderID, gridBagConstraints);

    labelLastRcvdOrderID.setText(bundle.getString("lastReceivedOrderID")); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 4;
    gridBagConstraints.gridy = 5;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
    gridBagConstraints.insets = new java.awt.Insets(0, 6, 0, 3);
    panelTelegramContent.add(labelLastRcvdOrderID, gridBagConstraints);

    textFieldLastRcvdOrderID.setEditable(false);
    textFieldLastRcvdOrderID.setBackground(new java.awt.Color(255, 255, 204));
    textFieldLastRcvdOrderID.setColumns(6);
    textFieldLastRcvdOrderID.setFont(new java.awt.Font("Monospaced", 0, 11)); // NOI18N
    textFieldLastRcvdOrderID.setHorizontalAlignment(javax.swing.JTextField.TRAILING);
    textFieldLastRcvdOrderID.setText("22222");
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 5;
    gridBagConstraints.gridy = 5;
    gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
    gridBagConstraints.anchor = java.awt.GridBagConstraints.SOUTH;
    gridBagConstraints.insets = new java.awt.Insets(3, 0, 0, 0);
    panelTelegramContent.add(textFieldLastRcvdOrderID, gridBagConstraints);

    panelState.add(panelTelegramContent, java.awt.BorderLayout.CENTER);

    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 0;
    gridBagConstraints.gridy = 1;
    gridBagConstraints.gridwidth = 2;
    gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
    gridBagConstraints.weightx = 1.0;
    add(panelState, gridBagConstraints);

    fillingLabel1.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
    gridBagConstraints = new java.awt.GridBagConstraints();
    gridBagConstraints.gridx = 0;
    gridBagConstraints.gridy = 3;
    gridBagConstraints.gridwidth = 2;
    gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
    gridBagConstraints.weightx = 1.0;
    gridBagConstraints.weighty = 1.0;
    add(fillingLabel1, gridBagConstraints);

    getAccessibleContext().setAccessibleName(bundle.getString("StatusPanelTitle")); // NOI18N
  }// </editor-fold>//GEN-END:initComponents

    private void chkBoxEnablePeriodicGetStateActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_chkBoxEnablePeriodicGetStateActionPerformed
      if (chkBoxEnablePeriodicGetState.isSelected()) {
        try {
          txtGetStateInterval.setEditable(false);
          int interval = Integer.parseInt(txtGetStateInterval.getText());
          if (interval <= 0) {
            throw new NumberFormatException();
          }
          sendAdapterCommand(new SetStateRequestIntervalCommand(interval));
          sendAdapterCommand(new SetPeriodicStateRequestEnabledCommand(true));
        }
        catch (NumberFormatException exc) {
          // Reset check box and text field.
          chkBoxEnablePeriodicGetState.setSelected(false);
          txtGetStateInterval.setEditable(true);
          // Notify the user about the invalid input.
          ResourceBundle bundle = ResourceBundle.getBundle("de/fraunhofer/iml/opentcs/bmw_str/Bundle");
          JOptionPane.showMessageDialog(this, bundle.getString("InvalidRequestIntervalMsg"));
        }
      }
      else {
        sendAdapterCommand(new SetPeriodicStateRequestEnabledCommand(false));
        txtGetStateInterval.setEditable(true);
      }
    }//GEN-LAST:event_chkBoxEnablePeriodicGetStateActionPerformed

  private void buttonGetStateActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_buttonGetStateActionPerformed
    SendRequestCommand command = new SendRequestCommand(new StateRequest(0));
    sendAdapterCommand(command);
  }//GEN-LAST:event_buttonGetStateActionPerformed

  private void sendAdapterCommand(AdapterCommand command) {
    try {
      callWrapper.call(() -> vehicleService.sendCommAdapterCommand(processModel.getVehicleRef(),
                                                                   command));
    }
    catch (Exception ex) {
      LOG.warn("Error sending comm adapter command '{}'", command, ex);
    }
  }

  // Variables declaration - do not modify//GEN-BEGIN:variables
  private javax.swing.JButton buttonGetState;
  private javax.swing.JCheckBox chkBoxEnablePeriodicGetState;
  private javax.swing.JLabel fillingLabel1;
  private javax.swing.JLabel fillingLabel2;
  private javax.swing.JLabel labelCurrOrderID;
  private javax.swing.JLabel labelGetStateInterval;
  private javax.swing.JLabel labelLastOrderID;
  private javax.swing.JLabel labelLastRcvdOrderID;
  private javax.swing.JLabel labelStateCRP;
  private javax.swing.JLabel labelStateLS;
  private javax.swing.JLabel labelStateOS;
  private javax.swing.JLabel labelStateTeleCount;
  private javax.swing.JPanel panelManualStateRequest;
  private javax.swing.JPanel panelPeriodicStateRequests;
  private javax.swing.JPanel panelState;
  private javax.swing.JPanel panelTelegramContent;
  private javax.swing.JTextField textFieldCurrOrderID;
  private javax.swing.JTextField textFieldLastOrderID;
  private javax.swing.JTextField textFieldLastRcvdOrderID;
  private javax.swing.JTextField textFieldStateCRP;
  private javax.swing.JTextField textFieldStateLS;
  private javax.swing.JTextField textFieldStateOS;
  private javax.swing.JTextField textFieldStateTeleCount;
  private javax.swing.JTextField txtGetStateInterval;
  // End of variables declaration//GEN-END:variables
}
