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
public class Ros2CommAdapterPanelOLD
    extends VehicleCommAdapterPanel {

  /**
   * The resource bundle.
   */
  private static final ResourceBundle bundle = ResourceBundle.getBundle(BUNDLE_PATH);

  /**
   * This class's Logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapterPanelOLD.class);
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
  public Ros2CommAdapterPanelOLD(@Assisted Ros2ProcessModelTO processModel,
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
      updateOperatingTime(processModel.getOperatingTime());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.ACCELERATION.name())) {
      updateMaxAcceleration(processModel.getMaxAcceleration());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.DECELERATION.name())) {
      updateMaxDeceleration(processModel.getMaxDeceleration());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.MAX_FORWARD_VELOCITY.name())) {
      updateMaxForwardVelocity(processModel.getMaxFwdVelocity());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.MAX_REVERSE_VELOCITY.name())) {
      updateMaxReverseVelocity(processModel.getMaxRevVelocity());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.SINGLE_STEP_MODE.name())) {
      updateSingleStepMode(processModel.isSingleStepModeEnabled());
    }
    else if (Objects.equals(attributeChanged,
                            Ros2ProcessModel.Attribute.VEHICLE_PAUSED.name())) {
      updateVehiclePaused(processModel.isVehiclePaused());
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
      updatePosition(processModel.getVehiclePosition());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.STATE.name())) {
      updateVehicleState(processModel.getVehicleState());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.PRECISE_POSITION.name())) {
      updatePrecisePosition(processModel.getPrecisePosition());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.ORIENTATION_ANGLE.name())) {
      updateOrientationAngle(processModel.getOrientationAngle());
    }
    else if (Objects.equals(attributeChanged,
                            VehicleProcessModel.Attribute.ENERGY_LEVEL.name())) {
      updateEnergyLevel(processModel.getEnergyLevel());
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

  private void updateEnergyLevel(int energy) {
    SwingUtilities.invokeLater(() -> energyLevelTxt.setText(Integer.toString(energy)));
  }

  private void updateCommAdapterEnabled(boolean isEnabled) {
    SwingUtilities.invokeLater(() -> {
      setStatePanelEnabled(isEnabled);
      chkBoxEnable.setSelected(isEnabled);
    });
  }

  private void updatePosition(String vehiclePosition) {
    SwingUtilities.invokeLater(() -> {
      if (vehiclePosition == null) {
        positionTxt.setText("");
        return;
      }

      try {
        for (Point curPoint : callWrapper.call(() -> vehicleService.fetchObjects(Point.class))) {
          if (curPoint.getName().equals(vehiclePosition)) {
            positionTxt.setText(curPoint.getName());
            break;
          }
        }
      }
      catch (Exception ex) {
        LOG.warn("Error fetching points", ex);
      }
    });
  }

  private void updateVehicleState(Vehicle.State vehicleState) {
    SwingUtilities.invokeLater(() -> stateTxt.setText(vehicleState.toString()));
  }

  private void updatePrecisePosition(Triple precisePos) {
    SwingUtilities.invokeLater(() -> setPrecisePosText(precisePos));
  }

  private void updateOrientationAngle(double orientation) {
    SwingUtilities.invokeLater(() -> {
      if (Double.isNaN(orientation)) {
        orientationAngleTxt.setText(bundle.getString("ros2CommAdapterPanel.textField_orientationAngle.angleNotSetPlaceholder"));
      }
      else {
        orientationAngleTxt.setText(Double.toString(orientation));
      }
    });
  }

  private void updateOperatingTime(int defaultOperatingTime) {
    SwingUtilities.invokeLater(() -> opTimeTxt.setText(Integer.toString(defaultOperatingTime)));
  }

  private void updateMaxAcceleration(int maxAcceleration) {
    SwingUtilities.invokeLater(() -> maxAccelTxt.setText(Integer.toString(maxAcceleration)));
  }

  private void updateMaxDeceleration(int maxDeceleration) {
    SwingUtilities.invokeLater(() -> maxDecelTxt.setText(Integer.toString(maxDeceleration)));
  }

  private void updateMaxForwardVelocity(int maxFwdVelocity) {
    SwingUtilities.invokeLater(() -> maxFwdVeloTxt.setText(Integer.toString(maxFwdVelocity)));
  }

  private void updateMaxReverseVelocity(int maxRevVelocity) {
    SwingUtilities.invokeLater(() -> maxRevVeloTxt.setText(Integer.toString(maxRevVelocity)));
  }

  private void updateSingleStepMode(boolean singleStepMode) {
    SwingUtilities.invokeLater(() -> {
      triggerButton.setEnabled(singleStepMode);
      singleModeRadioButton.setSelected(singleStepMode);
      flowModeRadioButton.setSelected(!singleStepMode);
    });
  }

  private void updateVehiclePaused(boolean isVehiclePaused) {
    SwingUtilities.invokeLater(() -> pauseVehicleCheckBox.setSelected(isVehiclePaused));
  }

  /**
   * Enable/disable the input fields and buttons in the "Current position/state" panel.
   * If disabled the user can not change any values or modify the vehicles state.
   *
   * @param enabled boolean indicating if the panel should be enabled
   */
  private void setStatePanelEnabled(boolean enabled) {
    SwingUtilities.invokeLater(() -> positionTxt.setEnabled(enabled));
    SwingUtilities.invokeLater(() -> stateTxt.setEnabled(enabled));
    SwingUtilities.invokeLater(() -> energyLevelTxt.setEnabled(enabled));
    SwingUtilities.invokeLater(() -> precisePosTextArea.setEnabled(enabled));
    SwingUtilities.invokeLater(() -> orientationAngleTxt.setEnabled(enabled));
    SwingUtilities.invokeLater(() -> pauseVehicleCheckBox.setEnabled(enabled));
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

        modeButtonGroup = new ButtonGroup();
        propertyEditorGroup = new ButtonGroup();
        vehicleBahaviourPanel = new JPanel();
        PropsPowerOuterContainerPanel = new JPanel();
        PropsPowerInnerContainerPanel = new JPanel();
        vehiclePropsPanel = new JPanel();
        maxFwdVeloLbl = new JLabel();
        maxFwdVeloTxt = new JTextField();
        maxFwdVeloUnitLbl = new JLabel();
        maxRevVeloLbl = new JLabel();
        maxRevVeloTxt = new JTextField();
        maxRevVeloUnitLbl = new JLabel();
        maxAccelLbl = new JLabel();
        maxAccelTxt = new JTextField();
        maxAccelUnitLbl = new JLabel();
        maxDecelTxt = new JTextField();
        maxDecelLbl = new JLabel();
        maxDecelUnitLbl = new JLabel();
        defaultOpTimeLbl = new JLabel();
        defaultOpTimeUntiLbl = new JLabel();
        opTimeTxt = new JTextField();
        profilesContainerPanel = new JPanel();
        filler1 = new Box.Filler(new java.awt.Dimension(0, 0), new java.awt.Dimension(0, 0), new java.awt.Dimension(32767, 0));
        ConnectionPropertiesPanel = new JPanel();
        connectButton = new JButton();
        ConnectionPanelA = new JPanel();
        hostLabel = new JLabel();
        hostTextField = new JTextField();
        portLabel = new JLabel();
        portTextField = new JTextField();
        connectionStatusLabel = new JLabel();
        vehicleStatePanel = new JPanel();
        stateContainerPanel = new JPanel();
        connectionPanel = new JPanel();
        chkBoxEnable = new JCheckBox();
        curPosPanel = new JPanel();
        energyLevelTxt = new JTextField();
        energyLevelLbl = new JLabel();
        pauseVehicleCheckBox = new JCheckBox();
        orientationAngleLbl = new JLabel();
        precisePosUnitLabel = new JLabel();
        orientationAngleTxt = new JTextField();
        energyLevelLabel = new JLabel();
        orientationLabel = new JLabel();
        positionTxt = new JTextField();
        positionLabel = new JLabel();
        pauseVehicleLabel = new JLabel();
        jLabel2 = new JLabel();
        stateTxt = new JTextField();
        jLabel3 = new JLabel();
        precisePosTextArea = new JTextArea();
        propertySetterPanel = new JPanel();
        keyLabel = new JLabel();
        valueTextField = new JTextField();
        propSetButton = new JButton();
        removePropRadioBtn = new JRadioButton();
        setPropValueRadioBtn = new JRadioButton();
        jPanel3 = new JPanel();
        keyTextField = new JTextField();
        eventPanel = new JPanel();
        includeAppendixCheckBox = new JCheckBox();
        appendixTxt = new JTextField();
        dispatchEventButton = new JButton();
        dispatchCommandFailedButton = new JButton();
        controlTabPanel = new JPanel();
        singleModeRadioButton = new JRadioButton();
        flowModeRadioButton = new JRadioButton();
        triggerButton = new JButton();
        loadDevicePanel = new JPanel();
        jPanel1 = new JPanel();
        jPanel2 = new JPanel();
        lHDCheckbox = new JCheckBox();

        setName("Ros2CommAdapterPanel"); // NOI18N
        setLayout(new java.awt.BorderLayout());

        vehicleBahaviourPanel.setLayout(new java.awt.BorderLayout());

        PropsPowerOuterContainerPanel.setLayout(new java.awt.BorderLayout());

        PropsPowerInnerContainerPanel.setLayout(new BoxLayout(PropsPowerInnerContainerPanel, BoxLayout.X_AXIS));

        vehiclePropsPanel.setBorder(BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_vehicleProperties.border.title"))); // NOI18N
        vehiclePropsPanel.setOpaque(false);
        vehiclePropsPanel.setLayout(new java.awt.GridBagLayout());

        maxFwdVeloLbl.setHorizontalAlignment(SwingConstants.TRAILING);
        maxFwdVeloLbl.setText(bundle.getString("ros2CommAdapterPanel.label_maximumForwardVelocity.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxFwdVeloLbl, gridBagConstraints);

        maxFwdVeloTxt.setEditable(false);
        maxFwdVeloTxt.setColumns(5);
        maxFwdVeloTxt.setHorizontalAlignment(JTextField.RIGHT);
        maxFwdVeloTxt.setText("0");
        maxFwdVeloTxt.setBorder(BorderFactory.createEtchedBorder());
        maxFwdVeloTxt.setEnabled(false);
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxFwdVeloTxt, gridBagConstraints);

        maxFwdVeloUnitLbl.setText("mm/s");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxFwdVeloUnitLbl, gridBagConstraints);

        maxRevVeloLbl.setHorizontalAlignment(SwingConstants.TRAILING);
        maxRevVeloLbl.setText(bundle.getString("ros2CommAdapterPanel.label_maximumReverseVelocity.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxRevVeloLbl, gridBagConstraints);

        maxRevVeloTxt.setEditable(false);
        maxRevVeloTxt.setColumns(5);
        maxRevVeloTxt.setHorizontalAlignment(JTextField.RIGHT);
        maxRevVeloTxt.setText("0");
        maxRevVeloTxt.setBorder(BorderFactory.createEtchedBorder());
        maxRevVeloTxt.setEnabled(false);
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxRevVeloTxt, gridBagConstraints);

        maxRevVeloUnitLbl.setText("mm/s");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxRevVeloUnitLbl, gridBagConstraints);

        maxAccelLbl.setHorizontalAlignment(SwingConstants.TRAILING);
        maxAccelLbl.setText(bundle.getString("ros2CommAdapterPanel.label_maximumAcceleration.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 2;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxAccelLbl, gridBagConstraints);

        maxAccelTxt.setEditable(false);
        maxAccelTxt.setColumns(5);
        maxAccelTxt.setHorizontalAlignment(JTextField.RIGHT);
        maxAccelTxt.setText("1000");
        maxAccelTxt.setBorder(BorderFactory.createEtchedBorder());
        maxAccelTxt.setEnabled(false);
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 2;
        gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxAccelTxt, gridBagConstraints);

        maxAccelUnitLbl.setText("<html>mm/s<sup>2</sup>");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 2;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxAccelUnitLbl, gridBagConstraints);

        maxDecelTxt.setEditable(false);
        maxDecelTxt.setColumns(5);
        maxDecelTxt.setHorizontalAlignment(JTextField.RIGHT);
        maxDecelTxt.setText("1000");
        maxDecelTxt.setBorder(BorderFactory.createEtchedBorder());
        maxDecelTxt.setEnabled(false);
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 3;
        gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxDecelTxt, gridBagConstraints);

        maxDecelLbl.setHorizontalAlignment(SwingConstants.TRAILING);
        maxDecelLbl.setText(bundle.getString("ros2CommAdapterPanel.label_maximumDeceleration.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 3;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxDecelLbl, gridBagConstraints);

        maxDecelUnitLbl.setText("<html>mm/s<sup>2</sup>");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 3;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(maxDecelUnitLbl, gridBagConstraints);

        defaultOpTimeLbl.setText(bundle.getString("ros2CommAdapterPanel.label_operatingTime.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 4;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(defaultOpTimeLbl, gridBagConstraints);

        defaultOpTimeUntiLbl.setText("ms");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 4;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(defaultOpTimeUntiLbl, gridBagConstraints);

        opTimeTxt.setEditable(false);
        opTimeTxt.setColumns(5);
        opTimeTxt.setHorizontalAlignment(JTextField.RIGHT);
        opTimeTxt.setText("1000");
        opTimeTxt.setBorder(BorderFactory.createEtchedBorder());
        opTimeTxt.setEnabled(false);

        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 4;
        gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        vehiclePropsPanel.add(opTimeTxt, gridBagConstraints);

        PropsPowerInnerContainerPanel.add(vehiclePropsPanel);
        ResourceBundle bundle = ResourceBundle.getBundle("nl/saxion/nena/opentcs/commadapter/ros2/Bundle"); // NOI18N
        vehiclePropsPanel.getAccessibleContext().setAccessibleName(bundle.getString("ros2CommAdapterPanel.panel_vehicleProperties.border.title")); // NOI18N

        PropsPowerOuterContainerPanel.add(PropsPowerInnerContainerPanel, java.awt.BorderLayout.WEST);

        vehicleBahaviourPanel.add(PropsPowerOuterContainerPanel, java.awt.BorderLayout.NORTH);

        profilesContainerPanel.setLayout(new java.awt.BorderLayout());
        profilesContainerPanel.add(filler1, java.awt.BorderLayout.CENTER);

        vehicleBahaviourPanel.add(profilesContainerPanel, java.awt.BorderLayout.SOUTH);

        ConnectionPropertiesPanel.setBorder(BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_connection.border.title"))); // NOI18N
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
        hostTextField.setBorder(BorderFactory.createEtchedBorder());
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

        portTextField.setBorder(BorderFactory.createEtchedBorder());
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

        vehicleBahaviourPanel.add(ConnectionPropertiesPanel, java.awt.BorderLayout.CENTER);
        ConnectionPropertiesPanel.getAccessibleContext().setAccessibleName("Connection");

        add(vehicleBahaviourPanel, java.awt.BorderLayout.CENTER);

        vehicleStatePanel.setLayout(new java.awt.BorderLayout());

        stateContainerPanel.setLayout(new BoxLayout(stateContainerPanel, BoxLayout.Y_AXIS));

        connectionPanel.setBorder(BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_adapterStatus.border.title"))); // NOI18N
        connectionPanel.setName("connectionPanel"); // NOI18N
        connectionPanel.setLayout(new java.awt.GridBagLayout());

        chkBoxEnable.setText(bundle.getString("ros2CommAdapterPanel.checkBox_enableAdapter.text")); // NOI18N
        chkBoxEnable.setName("chkBoxEnable"); // NOI18N
        chkBoxEnable.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                chkBoxEnableActionPerformed(evt);
            }
        });
        connectionPanel.add(chkBoxEnable, new java.awt.GridBagConstraints());

        stateContainerPanel.add(connectionPanel);

        curPosPanel.setBorder(BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_vehicleStatus.border.title"))); // NOI18N
        curPosPanel.setName("curPosPanel"); // NOI18N
        curPosPanel.setLayout(new java.awt.GridBagLayout());

        energyLevelTxt.setEditable(false);
        energyLevelTxt.setBackground(new java.awt.Color(255, 255, 255));
        energyLevelTxt.setText("100");
        energyLevelTxt.setBorder(BorderFactory.createEtchedBorder());
        energyLevelTxt.setName("energyLevelTxt"); // NOI18N
        energyLevelTxt.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                energyLevelTxtMouseClicked(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 2;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(3, 0, 0, 0);
        curPosPanel.add(energyLevelTxt, gridBagConstraints);

        energyLevelLbl.setText("%");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 2;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
        curPosPanel.add(energyLevelLbl, gridBagConstraints);

        pauseVehicleCheckBox.setEnabled(false);
        pauseVehicleCheckBox.setHorizontalAlignment(SwingConstants.LEFT);
        pauseVehicleCheckBox.setHorizontalTextPosition(SwingConstants.LEADING);
        pauseVehicleCheckBox.setName("pauseVehicleCheckBox"); // NOI18N
        pauseVehicleCheckBox.addItemListener(new java.awt.event.ItemListener() {
            public void itemStateChanged(java.awt.event.ItemEvent evt) {
                pauseVehicleCheckBoxItemStateChanged(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 5;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        curPosPanel.add(pauseVehicleCheckBox, gridBagConstraints);

        orientationAngleLbl.setText("<html>&#186;");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 4;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
        curPosPanel.add(orientationAngleLbl, gridBagConstraints);

        precisePosUnitLabel.setText("mm");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 2;
        gridBagConstraints.gridy = 3;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
        curPosPanel.add(precisePosUnitLabel, gridBagConstraints);

        orientationAngleTxt.setEditable(false);
        orientationAngleTxt.setBackground(new java.awt.Color(255, 255, 255));
        orientationAngleTxt.setText(bundle.getString("ros2CommAdapterPanel.textField_orientationAngle.angleNotSetPlaceholder")); // NOI18N
        orientationAngleTxt.setBorder(BorderFactory.createEtchedBorder());
        orientationAngleTxt.setName("orientationAngleTxt"); // NOI18N
        orientationAngleTxt.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                orientationAngleTxtMouseClicked(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 4;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.insets = new java.awt.Insets(3, 0, 0, 0);
        curPosPanel.add(orientationAngleTxt, gridBagConstraints);

        energyLevelLabel.setHorizontalAlignment(SwingConstants.TRAILING);
        energyLevelLabel.setText(bundle.getString("ros2CommAdapterPanel.label_energyLevel.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 2;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.insets = new java.awt.Insets(3, 3, 0, 3);
        curPosPanel.add(energyLevelLabel, gridBagConstraints);

        orientationLabel.setHorizontalAlignment(SwingConstants.RIGHT);
        orientationLabel.setText(bundle.getString("ros2CommAdapterPanel.label_orientationAngle.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 4;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.insets = new java.awt.Insets(3, 3, 0, 3);
        curPosPanel.add(orientationLabel, gridBagConstraints);

        positionTxt.setEditable(false);
        positionTxt.setBackground(new java.awt.Color(255, 255, 255));
        positionTxt.setBorder(BorderFactory.createEtchedBorder());
        positionTxt.setName("positionTxt"); // NOI18N
        positionTxt.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                positionTxtMouseClicked(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        curPosPanel.add(positionTxt, gridBagConstraints);

        positionLabel.setHorizontalAlignment(SwingConstants.TRAILING);
        positionLabel.setText(bundle.getString("ros2CommAdapterPanel.label_position.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.insets = new java.awt.Insets(3, 3, 0, 3);
        curPosPanel.add(positionLabel, gridBagConstraints);

        pauseVehicleLabel.setHorizontalAlignment(SwingConstants.TRAILING);
        pauseVehicleLabel.setText(bundle.getString("ros2CommAdapterPanel.label_pauseVehicle.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 5;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.insets = new java.awt.Insets(3, 3, 0, 3);
        curPosPanel.add(pauseVehicleLabel, gridBagConstraints);

        jLabel2.setHorizontalAlignment(SwingConstants.TRAILING);
        jLabel2.setText(bundle.getString("ros2CommAdapterPanel.label_state.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.insets = new java.awt.Insets(3, 3, 0, 3);
        curPosPanel.add(jLabel2, gridBagConstraints);

        stateTxt.setEditable(false);
        stateTxt.setBackground(new java.awt.Color(255, 255, 255));
        stateTxt.setBorder(BorderFactory.createEtchedBorder());
        stateTxt.setName("stateTxt"); // NOI18N
        stateTxt.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                stateTxtMouseClicked(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(3, 0, 0, 0);
        curPosPanel.add(stateTxt, gridBagConstraints);

        jLabel3.setHorizontalAlignment(SwingConstants.TRAILING);
        jLabel3.setText(bundle.getString("ros2CommAdapterPanel.label_precisePosition.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 3;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.insets = new java.awt.Insets(3, 3, 0, 3);
        curPosPanel.add(jLabel3, gridBagConstraints);

        precisePosTextArea.setEditable(false);
        precisePosTextArea.setFont(positionTxt.getFont());
        precisePosTextArea.setRows(3);
        precisePosTextArea.setText("X:\nY:\nZ:");
        precisePosTextArea.setBorder(BorderFactory.createEtchedBorder());
        precisePosTextArea.setName("precisePosTextArea"); // NOI18N
        precisePosTextArea.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                precisePosTextAreaMouseClicked(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 3;
        gridBagConstraints.fill = java.awt.GridBagConstraints.HORIZONTAL;
        gridBagConstraints.insets = new java.awt.Insets(3, 0, 0, 0);
        curPosPanel.add(precisePosTextArea, gridBagConstraints);

        stateContainerPanel.add(curPosPanel);
        curPosPanel.getAccessibleContext().setAccessibleName("Change");

        propertySetterPanel.setBorder(BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_vehicleProperty.border.title"))); // NOI18N
        propertySetterPanel.setLayout(new java.awt.GridBagLayout());

        keyLabel.setText(bundle.getString("ros2CommAdapterPanel.label_propertyKey.text")); // NOI18N
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        gridBagConstraints.insets = new java.awt.Insets(0, 0, 0, 3);
        propertySetterPanel.add(keyLabel, gridBagConstraints);

        valueTextField.setMaximumSize(new java.awt.Dimension(4, 18));
        valueTextField.setMinimumSize(new java.awt.Dimension(4, 18));
        valueTextField.setPreferredSize(new java.awt.Dimension(100, 20));
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 2;
        propertySetterPanel.add(valueTextField, gridBagConstraints);

        propSetButton.setText(bundle.getString("ros2CommAdapterPanel.button_setProperty.text")); // NOI18N
        propSetButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                propSetButtonActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 4;
        gridBagConstraints.gridwidth = 2;
        propertySetterPanel.add(propSetButton, gridBagConstraints);

        propertyEditorGroup.add(removePropRadioBtn);
        removePropRadioBtn.setText(bundle.getString("ros2CommAdapterPanel.radioButton_removeProperty.text")); // NOI18N
        removePropRadioBtn.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                removePropRadioBtnActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 3;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        propertySetterPanel.add(removePropRadioBtn, gridBagConstraints);

        propertyEditorGroup.add(setPropValueRadioBtn);
        setPropValueRadioBtn.setSelected(true);
        setPropValueRadioBtn.setText(bundle.getString("ros2CommAdapterPanel.radioButton_setProperty.text")); // NOI18N
        setPropValueRadioBtn.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                setPropValueRadioBtnActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 2;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        propertySetterPanel.add(setPropValueRadioBtn, gridBagConstraints);

        jPanel3.setLayout(new java.awt.GridBagLayout());

        keyTextField.setPreferredSize(new java.awt.Dimension(100, 20));
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 0;
        jPanel3.add(keyTextField, gridBagConstraints);

        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 1;
        propertySetterPanel.add(jPanel3, gridBagConstraints);

        stateContainerPanel.add(propertySetterPanel);

        eventPanel.setBorder(BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_eventDispatching.title"))); // NOI18N
        eventPanel.setLayout(new java.awt.GridBagLayout());

        includeAppendixCheckBox.setText(bundle.getString("ros2CommAdapterPanel.checkBox_includeAppendix.text")); // NOI18N
        includeAppendixCheckBox.addItemListener(new java.awt.event.ItemListener() {
            public void itemStateChanged(java.awt.event.ItemEvent evt) {
                includeAppendixCheckBoxItemStateChanged(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.weightx = 1.0;
        eventPanel.add(includeAppendixCheckBox, gridBagConstraints);

        appendixTxt.setEditable(false);
        appendixTxt.setColumns(10);
        appendixTxt.setText("XYZ");
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
        eventPanel.add(appendixTxt, gridBagConstraints);

        dispatchEventButton.setText(bundle.getString("ros2CommAdapterPanel.button_dispatchEvent.text")); // NOI18N
        dispatchEventButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                dispatchEventButtonActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.gridwidth = 2;
        gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
        eventPanel.add(dispatchEventButton, gridBagConstraints);

        dispatchCommandFailedButton.setText(bundle.getString("ros2CommAdapterPanel.button_failCurrentCommand.text")); // NOI18N
        dispatchCommandFailedButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                dispatchCommandFailedButtonActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 2;
        gridBagConstraints.gridwidth = 2;
        gridBagConstraints.fill = java.awt.GridBagConstraints.BOTH;
        gridBagConstraints.insets = new java.awt.Insets(3, 3, 3, 3);
        eventPanel.add(dispatchCommandFailedButton, gridBagConstraints);

        stateContainerPanel.add(eventPanel);

        controlTabPanel.setBorder(BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_commandProcessing.border.title"))); // NOI18N
        controlTabPanel.setLayout(new java.awt.GridBagLayout());

        modeButtonGroup.add(singleModeRadioButton);
        singleModeRadioButton.setText(bundle.getString("ros2CommAdapterPanel.checkBox_commandProcessingManual.text")); // NOI18N
        singleModeRadioButton.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 0));
        singleModeRadioButton.setName("singleModeRadioButton"); // NOI18N
        singleModeRadioButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                singleModeRadioButtonActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
        controlTabPanel.add(singleModeRadioButton, gridBagConstraints);

        modeButtonGroup.add(flowModeRadioButton);
        flowModeRadioButton.setSelected(true);
        flowModeRadioButton.setText(bundle.getString("ros2CommAdapterPanel.checkBox_commandProcessingAutomatic.text")); // NOI18N
        flowModeRadioButton.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 0));
        flowModeRadioButton.setName("flowModeRadioButton"); // NOI18N
        flowModeRadioButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                flowModeRadioButtonActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 0;
        gridBagConstraints.gridy = 0;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.WEST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 0);
        controlTabPanel.add(flowModeRadioButton, gridBagConstraints);

        triggerButton.setText(bundle.getString("ros2CommAdapterPanel.button_nextStep.text")); // NOI18N
        triggerButton.setEnabled(false);
        triggerButton.setName("triggerButton"); // NOI18N
        triggerButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                triggerButtonActionPerformed(evt);
            }
        });
        gridBagConstraints = new java.awt.GridBagConstraints();
        gridBagConstraints.gridx = 1;
        gridBagConstraints.gridy = 1;
        gridBagConstraints.anchor = java.awt.GridBagConstraints.EAST;
        gridBagConstraints.weightx = 1.0;
        gridBagConstraints.insets = new java.awt.Insets(0, 3, 0, 3);
        controlTabPanel.add(triggerButton, gridBagConstraints);

        stateContainerPanel.add(controlTabPanel);

        vehicleStatePanel.add(stateContainerPanel, java.awt.BorderLayout.NORTH);

        loadDevicePanel.setBorder(BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_loadHandlingDevice.border.title"))); // NOI18N
        loadDevicePanel.setLayout(new java.awt.BorderLayout());

        jPanel1.setLayout(new java.awt.GridBagLayout());
        loadDevicePanel.add(jPanel1, java.awt.BorderLayout.SOUTH);

        lHDCheckbox.setText("Device loaded");
        lHDCheckbox.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                lHDCheckboxClicked(evt);
            }
        });
        jPanel2.add(lHDCheckbox);

        loadDevicePanel.add(jPanel2, java.awt.BorderLayout.WEST);

        vehicleStatePanel.add(loadDevicePanel, java.awt.BorderLayout.CENTER);

        add(vehicleStatePanel, java.awt.BorderLayout.WEST);

        getAccessibleContext().setAccessibleName(bundle.getString("ros2CommAdapterPanel.accessibleName")); // NOI18N
    }// </editor-fold>//GEN-END:initComponents

  private void singleModeRadioButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_singleModeRadioButtonActionPerformed
    if (singleModeRadioButton.isSelected()) {
      triggerButton.setEnabled(true);

      sendCommAdapterCommand(new SetSingleStepModeEnabledCommand(true));
    }
  }//GEN-LAST:event_singleModeRadioButtonActionPerformed

  private void flowModeRadioButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_flowModeRadioButtonActionPerformed
    if (flowModeRadioButton.isSelected()) {
      triggerButton.setEnabled(false);

      sendCommAdapterCommand(new SetSingleStepModeEnabledCommand(false));
      sendCommAdapterCommand(new TriggerCommand());
    }
  }//GEN-LAST:event_flowModeRadioButtonActionPerformed

  private void triggerButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_triggerButtonActionPerformed
    sendCommAdapterCommand(new TriggerCommand());
  }//GEN-LAST:event_triggerButtonActionPerformed

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


  private void precisePosTextAreaMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_precisePosTextAreaMouseClicked
    if (precisePosTextArea.isEnabled()) {
      Triple pos = processModel.getPrecisePosition();
      // Create panel and dialog
      TripleTextInputPanel.Builder builder
          = new TripleTextInputPanel.Builder(bundle.getString("ros2CommAdapterPanel.dialog_setPrecisePosition.title"));
      builder.setUnitLabels("mm");
      builder.setLabels("X:", "Y:", "Z:");
      builder.enableResetButton(null);
      builder.enableValidation(TextInputPanel.TextInputValidator.REGEX_INT);
      if (pos != null) {
        builder.setInitialValues(Long.toString(pos.getX()),
                                 Long.toString(pos.getY()),
                                 Long.toString(pos.getZ()));
      }
      InputPanel panel = builder.build();
      InputDialog dialog = new InputDialog(panel);
      dialog.setVisible(true);
      // Get dialog result and set vehicle precise position
      if (dialog.getReturnStatus() == InputDialog.ReturnStatus.ACCEPTED) {
        if (dialog.getInput() == null) {
          // Clear precise position
          sendCommAdapterCommand(new SetPrecisePositionCommand(null));
        }
        else {
          // Set new precise position
          long x, y, z;
          String[] newPos = (String[]) dialog.getInput();
          try {
            x = Long.parseLong(newPos[0]);
            y = Long.parseLong(newPos[1]);
            z = Long.parseLong(newPos[2]);
          }
          catch (NumberFormatException | NullPointerException e) {
            return;
          }

          sendCommAdapterCommand(new SetPrecisePositionCommand(new Triple(x, y, z)));
        }
      }
    }
  }//GEN-LAST:event_precisePosTextAreaMouseClicked

  private void stateTxtMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_stateTxtMouseClicked
    if (!stateTxt.isEnabled()) {
      return;
    }

    Vehicle.State currentState = processModel.getVehicleState();
    // Create panel and dialog
    InputPanel panel = new DropdownListInputPanel.Builder<>(bundle.getString("ros2CommAdapterPanel.dialog_setState.title"),
                                                            Arrays.asList(Vehicle.State.values()))
        .setSelectionRepresenter(x -> x == null ? "" : x.name())
        .setLabel(bundle.getString("ros2CommAdapterPanel.label_state.text"))
        .setInitialSelection(currentState)
        .build();
    InputDialog dialog = new InputDialog(panel);
    dialog.setVisible(true);
    // Get dialog results and set vahicle stare
    if (dialog.getReturnStatus() == InputDialog.ReturnStatus.ACCEPTED) {
      Vehicle.State newState = (Vehicle.State) dialog.getInput();
      if (newState != currentState) {
        sendCommAdapterCommand(new SetStateCommand(newState));
      }
    }
  }//GEN-LAST:event_stateTxtMouseClicked

  private void positionTxtMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_positionTxtMouseClicked
    if (!positionTxt.isEnabled()) {
      return;
    }

    // Prepare list of model points
    Set<Point> pointSet;
    try {
      pointSet = callWrapper.call(() -> vehicleService.fetchObjects(Point.class));
    }
    catch (Exception ex) {
      LOG.warn("Error fetching points", ex);
      return;
    }

    List<Point> pointList = new ArrayList<>(pointSet);
    Collections.sort(pointList, Comparators.objectsByName());
    pointList.add(0, null);
    // Get currently selected point
    // TODO is there a better way to do this?
    Point currentPoint = null;
    String currentPointName = processModel.getVehiclePosition();
    for (Point p : pointList) {
      if (p != null && p.getName().equals(currentPointName)) {
        currentPoint = p;
        break;
      }
    }
    // Create panel and dialog
    InputPanel panel = new DropdownListInputPanel.Builder<>(
        bundle.getString("ros2CommAdapterPanel.dialog_setPosition.title"), pointList)
        .setSelectionRepresenter(x -> x == null ? "" : x.getName())
        .setLabel(bundle.getString("ros2CommAdapterPanel.label_position.text"))
        .setEditable(true)
        .setInitialSelection(currentPoint)
        .setRenderer(new StringListCellRenderer<>(x -> x == null ? "" : x.getName()))
        .build();
    InputDialog dialog = new InputDialog(panel);
    dialog.setVisible(true);
    // Get result from dialog and set vehicle position
    if (dialog.getReturnStatus() == InputDialog.ReturnStatus.ACCEPTED) {
      Object item = dialog.getInput();
      if (item == null) {
        sendCommAdapterCommand(new SetPositionCommand(null));
      }
      else {
        sendCommAdapterCommand(new SetPositionCommand(((Point) item).getName()));
      }
    }
  }//GEN-LAST:event_positionTxtMouseClicked

  private void orientationAngleTxtMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_orientationAngleTxtMouseClicked
    if (!orientationAngleTxt.isEnabled()) {
      return;
    }

    double currentAngle = processModel.getOrientationAngle();
    String initialValue = (Double.isNaN(currentAngle) ? "" : Double.toString(currentAngle));
    // Create dialog and panel
    InputPanel panel = new SingleTextInputPanel.Builder(
        bundle.getString("ros2CommAdapterPanel.dialog_setOrientationAngle.title"))
        .setLabel(bundle.getString("ros2CommAdapterPanel.label_orientationAngle.text"))
        .setUnitLabel("<html>&#186;")
        .setInitialValue(initialValue)
        .enableResetButton(null)
        .enableValidation(TextInputPanel.TextInputValidator.REGEX_FLOAT)
        .build();
    InputDialog dialog = new InputDialog(panel);
    dialog.setVisible(true);
    // Get input from dialog
    InputDialog.ReturnStatus returnStatus = dialog.getReturnStatus();
    if (returnStatus == InputDialog.ReturnStatus.ACCEPTED) {
      String input = (String) dialog.getInput();
      if (input == null) {
        // The reset button was pressed
        if (!Double.isNaN(processModel.getOrientationAngle())) {
          sendCommAdapterCommand(new SetOrientationAngleCommand(Double.NaN));
        }
      }
      else {
        // Set orientation provided by the user
        double angle;
        try {
          angle = Double.parseDouble(input);
        }
        catch (NumberFormatException e) {
          LOG.warn("Exception parsing orientation angle value '{}'", input, e);
          return;
        }

        sendCommAdapterCommand(new SetOrientationAngleCommand(angle));
      }
    }
  }//GEN-LAST:event_orientationAngleTxtMouseClicked

  private void pauseVehicleCheckBoxItemStateChanged(java.awt.event.ItemEvent evt) {//GEN-FIRST:event_pauseVehicleCheckBoxItemStateChanged
    if (evt.getStateChange() == java.awt.event.ItemEvent.SELECTED) {
      sendCommAdapterCommand(new SetVehiclePausedCommand(true));
    }
    else if (evt.getStateChange() == java.awt.event.ItemEvent.DESELECTED) {
      sendCommAdapterCommand(new SetVehiclePausedCommand(false));
    }
  }//GEN-LAST:event_pauseVehicleCheckBoxItemStateChanged

  private void energyLevelTxtMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_energyLevelTxtMouseClicked
    if (!energyLevelTxt.isEnabled()) {
      return;
    }

    // Create panel and dialog
    InputPanel panel = new SingleTextInputPanel.Builder(
        bundle.getString("ros2CommAdapterPanel.dialog_setEnergyLevel.title"))
        .setLabel(bundle.getString("ros2CommAdapterPanel.label_energyLevel.text"))
        .setUnitLabel("%")
        .setInitialValue(energyLevelTxt.getText())
        .enableValidation(TextInputPanel.TextInputValidator.REGEX_INT_RANGE_0_100)
        .build();
    InputDialog dialog = new InputDialog(panel);
    dialog.setVisible(true);
    // Get result from dialog and set energy level
    if (dialog.getReturnStatus() == InputDialog.ReturnStatus.ACCEPTED) {
      String input = (String) dialog.getInput();
      int energy;
      try {
        energy = Integer.parseInt(input);
      }
      catch (NumberFormatException e) {
        return;
      }

      sendCommAdapterCommand(new SetEnergyLevelCommand(energy));
    }
  }//GEN-LAST:event_energyLevelTxtMouseClicked

  private void includeAppendixCheckBoxItemStateChanged(java.awt.event.ItemEvent evt) {//GEN-FIRST:event_includeAppendixCheckBoxItemStateChanged
    appendixTxt.setEditable(includeAppendixCheckBox.isSelected());
  }//GEN-LAST:event_includeAppendixCheckBoxItemStateChanged

  private void dispatchEventButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_dispatchEventButtonActionPerformed
    String appendix = includeAppendixCheckBox.isSelected() ? appendixTxt.getText() : null;
    VehicleCommAdapterEvent event = new VehicleCommAdapterEvent(processModel.getVehicleName(),
                                                                appendix);
    sendCommAdapterCommand(new PublishEventCommand(event));
  }//GEN-LAST:event_dispatchEventButtonActionPerformed

  private void dispatchCommandFailedButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_dispatchCommandFailedButtonActionPerformed
    sendCommAdapterCommand(new CurrentMovementCommandFailedCommand());
  }//GEN-LAST:event_dispatchCommandFailedButtonActionPerformed

  private void propSetButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_propSetButtonActionPerformed
    sendCommAdapterCommand(new SetVehiclePropertyCommand(keyTextField.getText(),
                                                         setPropValueRadioBtn.isSelected()
                                                         ? valueTextField.getText() : null));
  }//GEN-LAST:event_propSetButtonActionPerformed

  private void removePropRadioBtnActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_removePropRadioBtnActionPerformed
    valueTextField.setEnabled(false);
  }//GEN-LAST:event_removePropRadioBtnActionPerformed

  private void setPropValueRadioBtnActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_setPropValueRadioBtnActionPerformed
    valueTextField.setEnabled(true);
  }//GEN-LAST:event_setPropValueRadioBtnActionPerformed

  private void lHDCheckboxClicked(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_lHDCheckboxClicked
    List<LoadHandlingDevice> devices = Arrays.asList(
        new LoadHandlingDevice(Ros2CommAdapter.LHD_NAME, lHDCheckbox.isSelected()));
    sendCommAdapterCommand(new SetLoadHandlingDevicesCommand(devices));
  }//GEN-LAST:event_lHDCheckboxClicked

    private void connectButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_connectButtonActionPerformed
        ConnectionDDS.connect();
        connectionStatusLabel.setText(bundle.getString("ros2CommAdapterPanel.label_connected.text"));
    }//GEN-LAST:event_connectButtonActionPerformed

  /**
   * Set the specified precise position to the text area. The method takes care
   * of the formatting. If any of the parameters is null all values will be set
   * to the "clear"-value.
   *
   * @param x x-position
   * @param y y-position
   * @param z z-poition
   */
  private void setPrecisePosText(Triple precisePos) {
    // Convert values to strings
    String xS = bundle.getString("ros2CommAdapterPanel.textArea_precisePosition.positionNotSetPlaceholder");
    String yS = bundle.getString("ros2CommAdapterPanel.textArea_precisePosition.positionNotSetPlaceholder");
    String zS = bundle.getString("ros2CommAdapterPanel.textArea_precisePosition.positionNotSetPlaceholder");

    if (precisePos != null) {
      xS = String.valueOf(precisePos.getX());
      yS = String.valueOf(precisePos.getY());
      zS = String.valueOf(precisePos.getZ());
    }

    // Clip extremely long string values
    xS = (xS.length() > 20) ? (xS.substring(0, 20) + "...") : xS;
    yS = (yS.length() > 20) ? (yS.substring(0, 20) + "...") : yS;
    zS = (zS.length() > 20) ? (zS.substring(0, 20) + "...") : zS;

    // Build formatted text
    StringBuilder text = new StringBuilder("");
    text.append("X: ").append(xS).append("\n")
        .append("Y: ").append(yS).append("\n")
        .append("Z: ").append(zS);
    precisePosTextArea.setText(text.toString());
  }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private JPanel ConnectionPanelA;
    private JPanel ConnectionPropertiesPanel;
    private JPanel PropsPowerInnerContainerPanel;
    private JPanel PropsPowerOuterContainerPanel;
    private JTextField appendixTxt;
    private JCheckBox chkBoxEnable;
    private JButton connectButton;
    private JPanel connectionPanel;
    private JLabel connectionStatusLabel;
    private JPanel controlTabPanel;
    private JPanel curPosPanel;
    private JLabel defaultOpTimeLbl;
    private JLabel defaultOpTimeUntiLbl;
    private JButton dispatchCommandFailedButton;
    private JButton dispatchEventButton;
    private JLabel energyLevelLabel;
    private JLabel energyLevelLbl;
    private JTextField energyLevelTxt;
    private JPanel eventPanel;
    private Box.Filler filler1;
    private JRadioButton flowModeRadioButton;
    private JLabel hostLabel;
    private JTextField hostTextField;
    private JCheckBox includeAppendixCheckBox;
    private JLabel jLabel2;
    private JLabel jLabel3;
    private JPanel jPanel1;
    private JPanel jPanel2;
    private JPanel jPanel3;
    private JLabel keyLabel;
    private JTextField keyTextField;
    private JCheckBox lHDCheckbox;
    private JPanel loadDevicePanel;
    private JLabel maxAccelLbl;
    private JTextField maxAccelTxt;
    private JLabel maxAccelUnitLbl;
    private JLabel maxDecelLbl;
    private JTextField maxDecelTxt;
    private JLabel maxDecelUnitLbl;
    private JLabel maxFwdVeloLbl;
    private JTextField maxFwdVeloTxt;
    private JLabel maxFwdVeloUnitLbl;
    private JLabel maxRevVeloLbl;
    private JTextField maxRevVeloTxt;
    private JLabel maxRevVeloUnitLbl;
    private ButtonGroup modeButtonGroup;
    private JTextField opTimeTxt;
    private JLabel orientationAngleLbl;
    private JTextField orientationAngleTxt;
    private JLabel orientationLabel;
    private JCheckBox pauseVehicleCheckBox;
    private JLabel pauseVehicleLabel;
    private JLabel portLabel;
    private JTextField portTextField;
    private JLabel positionLabel;
    private JTextField positionTxt;
    private JTextArea precisePosTextArea;
    private JLabel precisePosUnitLabel;
    private JPanel profilesContainerPanel;
    private JButton propSetButton;
    private ButtonGroup propertyEditorGroup;
    private JPanel propertySetterPanel;
    private JRadioButton removePropRadioBtn;
    private JRadioButton setPropValueRadioBtn;
    private JRadioButton singleModeRadioButton;
    private JPanel stateContainerPanel;
    private JTextField stateTxt;
    private JButton triggerButton;
    private JTextField valueTextField;
    private JPanel vehicleBahaviourPanel;
    private JPanel vehiclePropsPanel;
    private JPanel vehicleStatePanel;
    // End of variables declaration//GEN-END:variables
  // CHECKSTYLE:ON

}
