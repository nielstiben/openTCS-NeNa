/**
 * Copyright (c) The openTCS Authors.
 *
 * This program is free software and subject to the MIT license. (For details,
 * see the licensing information (LICENSE.txt) you should have received with
 * this copy of the software.)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center;

import com.google.inject.assistedinject.Assisted;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands.*;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.gui_components.*;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionDDS;
import nl.saxion.nena.opentcs.commadapter.ros2.virtual_vehicle.Ros2ProcessModelTO;
import org.opentcs.components.kernel.services.VehicleService;
import org.opentcs.customizations.ServiceCallWrapper;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;
import org.opentcs.drivers.vehicle.VehicleCommAdapterEvent;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.opentcs.drivers.vehicle.management.VehicleCommAdapterPanel;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.CallWrapper;
import org.opentcs.util.Comparators;
import org.opentcs.util.gui.StringListCellRenderer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.inject.Inject;
import javax.swing.*;
import java.util.*;

import static java.util.Objects.requireNonNull;
import static nl.saxion.nena.opentcs.commadapter.ros2.common.I18nLoopbackCommAdapter.BUNDLE_PATH;

/**
 * The panel corresponding to the Ros2CommAdapter.
 *
 * @author Iryna Felko (Fraunhofer IML)
 * @author Stefan Walter (Fraunhofer IML)
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class Ros2CommAdapterPanel
    extends VehicleCommAdapterPanel {

  /**
   * The resource bundle.
   */
  private static final ResourceBundle bundle = ResourceBundle.getBundle(BUNDLE_PATH);

  /**
   * This class's Logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapterPanel.class);
  /**
   * The vehicle service used for interaction with the comm adapter.
   */
  private final VehicleService vehicleService;
  /**
   * The comm adapter's process model.
   */
  private Ros2ProcessModelTO processModel;
  /**
   * The call wrapper to use for service calls.
   */
  private final CallWrapper callWrapper;

  /**
   * Creates new Ros2CommAdapterPanel.
   *
   * @param processModel The comm adapter's process model.
   * @param vehicleService The vehicle service.
   * @param callWrapper The call wrapper to use for service calls.
   */
  @Inject
  public Ros2CommAdapterPanel(@Assisted Ros2ProcessModelTO processModel,
                              @Assisted VehicleService vehicleService,
                              @ServiceCallWrapper CallWrapper callWrapper) {

    this.processModel = requireNonNull(processModel, "processModel");
    this.vehicleService = requireNonNull(vehicleService, "vehicleService");
    this.callWrapper = requireNonNull(callWrapper, "callWrapper");

    initComponents();
    initGuiContent();
  }

  @Override
  public void processModelChange(String attributeChanged, VehicleProcessModelTO newProcessModel) {
    if (!(newProcessModel instanceof Ros2ProcessModelTO)) {
      return;
    }

    processModel = (Ros2ProcessModelTO) newProcessModel;
    updateRos2ProcessModelData(attributeChanged, processModel);
    updateVehicleProcessModelData(attributeChanged, processModel);
  }

  private void initGuiContent() {
    for (VehicleProcessModel.Attribute attribute : VehicleProcessModel.Attribute.values()) {
      processModelChange(attribute.name(), processModel);
    }
    for (Ros2ProcessModel.Attribute attribute : Ros2ProcessModel.Attribute.values()) {
      processModelChange(attribute.name(), processModel);
    }
  }

  private void updateRos2ProcessModelData(String attributeChanged,
                                              Ros2ProcessModelTO processModel) {
    if (Objects.equals(attributeChanged,
                       Ros2ProcessModel.Attribute.OPERATING_TIME.name())) {
//      updateOperatingTime(processModel.getOperatingTime());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.ACCELERATION.name())) {
//      updateMaxAcceleration(processModel.getMaxAcceleration());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.DECELERATION.name())) {
//      updateMaxDeceleration(processModel.getMaxDeceleration());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.MAX_FORWARD_VELOCITY.name())) {
//      updateMaxForwardVelocity(processModel.getMaxFwdVelocity());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.MAX_REVERSE_VELOCITY.name())) {
//      updateMaxReverseVelocity(processModel.getMaxRevVelocity());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.SINGLE_STEP_MODE.name())) {
//      updateSingleStepMode(processModel.isSingleStepModeEnabled());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.VEHICLE_PAUSED.name())) {
//      updateVehiclePaused(processModel.isVehiclePaused());
    }
  }

  private void updateVehicleProcessModelData(String attributeChanged,
                                             VehicleProcessModelTO processModel) {
    if (Objects.equals(attributeChanged,
                       VehicleProcessModel.Attribute.COMM_ADAPTER_ENABLED.name())) {
      updateCommAdapterEnabled(processModel.isCommAdapterEnabled());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.POSITION.name())) {
//      updatePosition(processModel.getVehiclePosition());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.STATE.name())) {
//      updateVehicleState(processModel.getVehicleState());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.PRECISE_POSITION.name())) {
//      updatePrecisePosition(processModel.getPrecisePosition());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.ORIENTATION_ANGLE.name())) {
//      updateOrientationAngle(processModel.getOrientationAngle());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.ENERGY_LEVEL.name())) {
//      updateEnergyLevel(processModel.getEnergyLevel());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.LOAD_HANDLING_DEVICES.name())) {
      updateVehicleLoadHandlingDevice(processModel.getLoadHandlingDevices());
    }
  }

  private void updateVehicleLoadHandlingDevice(List<LoadHandlingDevice> devices) {
    if (devices.size() > 1) {
      LOG.warn("size of load handling devices greater than 1 ({})", devices.size());
    }
    boolean loaded = devices.stream()
        .findFirst()
        .map(lhd -> lhd.isFull())
        .orElse(false);
    SwingUtilities.invokeLater(() -> lHDCheckbox.setSelected(loaded));
  }


  private void updateCommAdapterEnabled(boolean isEnabled) {
    SwingUtilities.invokeLater(() -> {
      setStatePanelEnabled(isEnabled);
      chkBoxEnable.setSelected(isEnabled);
    });
  }

  /**
   * Enable/disable the input fields and buttons in the "Current position/state" panel.
   * If disabled the user can not change any values or modify the vehicles state.
   *
   * @param enabled boolean indicating if the panel should be enabled
   */
  private void setStatePanelEnabled(boolean enabled) {
//    SwingUtilities.invokeLater(() -> positionTxt.setEnabled(enabled));
//    SwingUtilities.invokeLater(() -> stateTxt.setEnabled(enabled));
//    SwingUtilities.invokeLater(() -> energyLevelTxt.setEnabled(enabled));
//    SwingUtilities.invokeLater(() -> precisePosTextArea.setEnabled(enabled));
//    SwingUtilities.invokeLater(() -> orientationAngleTxt.setEnabled(enabled));
//    SwingUtilities.invokeLater(() -> pauseVehicleCheckBox.setEnabled(enabled));
  }

  private TCSObjectReference<Vehicle> getVehicleReference()
      throws Exception {
    return callWrapper.call(() -> vehicleService.
        fetchObject(Vehicle.class, processModel.getVehicleName())).getReference();
  }

  private void sendCommAdapterCommand(AdapterCommand command) {
    try {
      TCSObjectReference<Vehicle> vehicleRef = getVehicleReference();
      callWrapper.call(() -> vehicleService.sendCommAdapterCommand(vehicleRef, command));
    }
    catch (Exception ex) {
      LOG.warn("Error sending comm adapter command '{}'", command, ex);
    }
  }

  // CHECKSTYLE:OFF
  /**
   * This method is called from within the constructor to initialize the form.
   * WARNING: Do NOT modify this code. The content of this method is always
   * regenerated by the Form Editor.
   */
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {
        java.awt.GridBagConstraints gridBagConstraints;

        modeButtonGroup = new javax.swing.ButtonGroup();
        propertyEditorGroup = new javax.swing.ButtonGroup();
        jPanel4 = new javax.swing.JPanel();
        mainPanel = new javax.swing.JPanel();
        connectionPanel = new javax.swing.JPanel();
        chkBoxEnable = new javax.swing.JCheckBox();
        ConnectionPropertiesPanel = new javax.swing.JPanel();
        connectButton = new javax.swing.JButton();
        ConnectionPanelA = new javax.swing.JPanel();
        hostLabel = new javax.swing.JLabel();
        hostTextField = new javax.swing.JTextField();
        portLabel = new javax.swing.JLabel();
        portTextField = new javax.swing.JTextField();
        connectionStatusLabel = new javax.swing.JLabel();
        loadDevicePanel = new javax.swing.JPanel();
        jPanel1 = new javax.swing.JPanel();
        jPanel2 = new javax.swing.JPanel();
        lHDCheckbox = new javax.swing.JCheckBox();

        setName("Ros2CommAdapterPanel"); // NOI18N
        setLayout(new java.awt.BorderLayout());

        mainPanel.setBackground(new java.awt.Color(254, 181, 107));
        mainPanel.setLayout(new java.awt.GridBagLayout());

        connectionPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_adapterStatus.border.title"))); // NOI18N
        connectionPanel.setName("connectionPanel"); // NOI18N
        connectionPanel.setLayout(new java.awt.GridBagLayout());

        chkBoxEnable.setText(bundle.getString("ros2CommAdapterPanel.checkBox_enableAdapter.text")); // NOI18N
        chkBoxEnable.setName("chkBoxEnable"); // NOI18N
        chkBoxEnable.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                chkBoxEnableActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.weightx = 1.0;
        connectionPanel.add(chkBoxEnable, gridBagConstraints);

        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        mainPanel.add(connectionPanel, gridBagConstraints);

        java.util.ResourceBundle bundle = java.util.ResourceBundle.getBundle("nl/saxion/nena/opentcs/commadapter/ros2/Bundle"); // NOI18N
        ConnectionPropertiesPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_connection.border.title"))); // NOI18N
        ConnectionPropertiesPanel.setName(""); // NOI18N
        ConnectionPropertiesPanel.setPreferredSize(new java.awt.Dimension(100, 50));
        ConnectionPropertiesPanel.setLayout(new java.awt.GridBagLayout());

        connectButton.setText(bundle.getString("ros2CommAdapterPanel.button_connect.text")); // NOI18N
        connectButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                connectButtonActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        ConnectionPropertiesPanel.add(connectButton, gridBagConstraints);

        ConnectionPanelA.setBackground(new java.awt.Color(206, 253, 250));
        ConnectionPanelA.setLayout(new java.awt.GridBagLayout());

        hostLabel.setText(bundle.getString("ros2CommAdapterPanel.label_host.text")); // NOI18N
        ConnectionPanelA.add(hostLabel, new java.awt.GridBagConstraints());

        hostTextField.setText("localhost");
        hostTextField.setBorder(javax.swing.BorderFactory.createEtchedBorder());
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.weightx = 1.0;
        ConnectionPanelA.add(hostTextField, gridBagConstraints);

        portLabel.setText(bundle.getString("ros2CommAdapterPanel.label_port.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        ConnectionPanelA.add(portLabel, gridBagConstraints);

        portTextField.setBorder(javax.swing.BorderFactory.createEtchedBorder());
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.weightx = 1.0;
        ConnectionPanelA.add(portTextField, gridBagConstraints);

        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
        gridBagConstraints.ipady = 5;
        gridBagConstraints.weightx = 1.0;
        ConnectionPropertiesPanel.add(ConnectionPanelA, gridBagConstraints);

        connectionStatusLabel.setText(bundle.getString("ros2CommAdapterPanel.label_notConnected.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 2;
        ConnectionPropertiesPanel.add(connectionStatusLabel, gridBagConstraints);

        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 2;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.ipadx = 5;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.weighty = 1.0;
        mainPanel.add(ConnectionPropertiesPanel, gridBagConstraints);

        loadDevicePanel.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_loadHandlingDevice.border.title"))); // NOI18N
        loadDevicePanel.setLayout(new java.awt.GridBagLayout());

        jPanel1.setLayout(new java.awt.GridBagLayout());
        loadDevicePanel.add(jPanel1, new java.awt.GridBagConstraints());

        lHDCheckbox.setText("Device loaded");
        lHDCheckbox.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                lHDCheckboxClicked(evt);
            }
        });
        jPanel2.add(lHDCheckbox);

        loadDevicePanel.add(jPanel2, new java.awt.GridBagConstraints());

        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        mainPanel.add(loadDevicePanel, gridBagConstraints);

        add(mainPanel, java.awt.BorderLayout.CENTER);

        getAccessibleContext().setAccessibleName(bundle.getString("ros2CommAdapterPanel.accessibleName")); // NOI18N
    }// </editor-fold>//GEN-END:initComponents

    private void chkBoxEnableActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_chkBoxEnableActionPerformed
        try {
            Vehicle vehicle = callWrapper.call(() -> vehicleService.fetchObject(Vehicle.class, processModel.getVehicleName()));

            if (chkBoxEnable.isSelected()) {
                callWrapper.call(() -> vehicleService.enableCommAdapter(vehicle.getReference()));
            }
            else {
                callWrapper.call(() -> vehicleService.disableCommAdapter(vehicle.getReference()));
            }

            setStatePanelEnabled(chkBoxEnable.isSelected());
        }
        catch (Exception ex) {
            LOG.warn("Error enabling/disabling comm adapter", ex);
        }
    }//GEN-LAST:event_chkBoxEnableActionPerformed

    private void connectButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_connectButtonActionPerformed
        ConnectionDDS.connect();
        connectionStatusLabel.setText(bundle.getString("ros2CommAdapterPanel.label_connected.text"));
    }//GEN-LAST:event_connectButtonActionPerformed

    private void lHDCheckboxClicked(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_lHDCheckboxClicked
        List<LoadHandlingDevice> devices = Arrays.asList(
            new LoadHandlingDevice(Ros2CommAdapter.LHD_NAME, lHDCheckbox.isSelected()));
        sendCommAdapterCommand(new SetLoadHandlingDevicesCommand(devices));
    }//GEN-LAST:event_lHDCheckboxClicked

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JPanel ConnectionPanelA;
    private javax.swing.JPanel ConnectionPropertiesPanel;
    private javax.swing.JCheckBox chkBoxEnable;
    private javax.swing.JButton connectButton;
    private javax.swing.JPanel connectionPanel;
    private javax.swing.JLabel connectionStatusLabel;
    private javax.swing.JLabel hostLabel;
    private javax.swing.JTextField hostTextField;
    private javax.swing.JPanel jPanel1;
    private javax.swing.JPanel jPanel2;
    private javax.swing.JPanel jPanel4;
    private javax.swing.JCheckBox lHDCheckbox;
    private javax.swing.JPanel loadDevicePanel;
    private javax.swing.JPanel mainPanel;
    private javax.swing.ButtonGroup modeButtonGroup;
    private javax.swing.JLabel portLabel;
    private javax.swing.JTextField portTextField;
    private javax.swing.ButtonGroup propertyEditorGroup;
    // End of variables declaration//GEN-END:variables
  // CHECKSTYLE:ON

}
