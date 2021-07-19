/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel;

import com.google.inject.assistedinject.Assisted;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.commands.*;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.gui_components.CoordinateInputPanel;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.gui_components.InputDialog;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.gui_components.InputPanel;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.gui_components.PointListInputPanel;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.library.InputValidationLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2ProcessModelAttribute;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2ProcessModelTO;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.constants.NodeRunningStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.UnitConverterLib;
import org.opentcs.components.kernel.services.VehicleService;
import org.opentcs.customizations.ServiceCallWrapper;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.opentcs.drivers.vehicle.management.VehicleCommAdapterPanel;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.CallWrapper;
import org.opentcs.util.Comparators;
import org.opentcs.util.gui.StringListCellRenderer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;
import javax.inject.Inject;
import javax.swing.*;
import javax.swing.table.DefaultTableModel;
import java.awt.*;
import java.io.IOException;
import java.net.URI;
import java.util.*;
import java.util.List;

import static java.util.Objects.requireNonNull;
import static javax.swing.JOptionPane.showMessageDialog;
import static nl.saxion.nena.opentcs.commadapter.ros2.I18nROS2CommAdapter.BUNDLE_PATH;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2ProcessModelAttribute.*;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.operation.ExecuteOperationWorkflow.LOAD_HANDLING_DEVICE_NAME;

/**
 * The panel corresponding to the Ros2CommAdapter.
 *
 * @author Niels Tiben
 */
public class Ros2CommAdapterPanel extends VehicleCommAdapterPanel {
    private static final ResourceBundle bundle = ResourceBundle.getBundle(BUNDLE_PATH);
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapterPanel.class);
    private final VehicleService vehicleService;
    private final CallWrapper callWrapper;

    //================================================================================
    // Class variables.
    //================================================================================

    private boolean isAdapterEnabled = false;
    private Ros2ProcessModelTO processModel;

    //================================================================================
    // Methods for construction / initiation.
    //================================================================================

    /**
     * Creates new Ros2CommAdapterPanel.
     *
     * @param processModel   The comm adapter's process model.
     * @param vehicleService The vehicle service.
     * @param callWrapper    The call wrapper to use for service calls.
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

    private void initGuiContent() {
        for (VehicleProcessModel.Attribute attribute : VehicleProcessModel.Attribute.values()) {
            processModelChange(attribute.name(), this.processModel);
        }
        for (Ros2ProcessModelAttribute attribute : Ros2ProcessModelAttribute.values()) {
            processModelChange(attribute.name(), this.processModel);
        }
    }

    //================================================================================
    // Methods for updating GUI based on given attributes.
    //================================================================================

    /**
     * Callback method when a vehicle's attribute has changed.
     *
     * @param attributeChanged The changed attribute.
     * @param newProcessModel  Updated processModel
     */
    @Override
    public void processModelChange(String attributeChanged, VehicleProcessModelTO newProcessModel) {
        if (!(newProcessModel instanceof Ros2ProcessModelTO)) {
            return;
        }

        this.processModel = (Ros2ProcessModelTO) newProcessModel;
        updateRos2ProcessModelData(attributeChanged, this.processModel);
        updateVehicleProcessModelData(attributeChanged, this.processModel);
    }

    /* --------------- Attribute switch: general attributes ---------------*/
    private void updateVehicleProcessModelData(String attributeChanged, VehicleProcessModelTO processModel) {
        if (attributeChanged.equals(VehicleProcessModel.Attribute.COMM_ADAPTER_ENABLED.name())) {
            updateIsAdapterEnabled(processModel.isCommAdapterEnabled());
        } else if (attributeChanged.equals(VehicleProcessModel.Attribute.POSITION.name())) {
            updatePositionPointValueLabel(processModel.getVehiclePosition());
        } else if (attributeChanged.equals(VehicleProcessModel.Attribute.PRECISE_POSITION.name())) {
            updatePositionCoordinateValueLabel(processModel.getPrecisePosition());
        } else if (attributeChanged.equals(VehicleProcessModel.Attribute.ORIENTATION_ANGLE.name())) {
            updateOrientationAngleValueLabel(processModel.getOrientationAngle());
        } else if (attributeChanged.equals(VehicleProcessModel.Attribute.LOAD_HANDLING_DEVICES.name())) {
            updateVehicleLoadHandlingDevice(processModel.getLoadHandlingDevices());
        }
    }

    /* --------------- Attribute switch: driver specific attributes ---------------*/
    private void updateRos2ProcessModelData(@Nonnull String attributeChanged, @Nonnull Ros2ProcessModelTO processModel) {

        if (attributeChanged.equals(NODE_STATUS.name())) {
            updateNodeStatus(processModel.getNodeStatus());
        } else if (attributeChanged.equals(NAVIGATION_GOALS.name())) {
            updateNavigationGoalsTable(processModel.getNavigationGoalTable());
        }
    }

    /* --------------- Attribute change: Enabled ---------------*/
    private void updateIsAdapterEnabled(boolean isEnabled) {
        this.isAdapterEnabled = isEnabled;
        SwingUtilities.invokeLater(() -> setStatePanelEnabled(this.isAdapterEnabled));
    }

    /* --------------- Attribute change: Node Status ---------------*/
    private void updateNodeStatus(@Nonnull String nodeStatus) {
        if (nodeStatus.equals(NodeRunningStatus.NOT_ACTIVE.name())) {
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setText("Node is not active"));
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setForeground(Color.BLACK));
            SwingUtilities.invokeLater(() -> enableButton.setEnabled(true)); // Release the enabled button.
        } else if (nodeStatus.equals(NodeRunningStatus.INITIATING.name())) {
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setText("Node is initiating"));
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setForeground(Color.ORANGE));
            SwingUtilities.invokeLater(() -> enableButton.setEnabled(false)); // Lock the enabled button.
        } else if (nodeStatus.equals(NodeRunningStatus.ACTIVE.name())) {
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setText("Node is active"));
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setForeground(Color.GREEN));
            SwingUtilities.invokeLater(() -> enableButton.setEnabled(true)); // Release the enabled button.
        } else if (nodeStatus.equals(NodeRunningStatus.TERMINATING.name())) {
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setText("Node is shutting down"));
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setForeground(Color.ORANGE));
            SwingUtilities.invokeLater(() -> enableButton.setEnabled(false)); // Lock the enabled button.
        } else {
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setText("Node has an unknown state"));
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setForeground(Color.RED));
        }
    }

    /* --------------- Attribute change: Position point ---------------*/
    private void updatePositionPointValueLabel(String updatedPointName) {
        if (updatedPointName != null && !updatedPointName.isEmpty()) {
            SwingUtilities.invokeLater(() -> positionPointValueLabel.setText(updatedPointName));
        }
    }

    /* --------------- Attribute change: Position coordinate ---------------*/
    private void updatePositionCoordinateValueLabel(Triple updatedCoordinate) {
        if (updatedCoordinate != null) {
            double[] xyz = UnitConverterLib.convertTripleToCoordinatesInMeter(updatedCoordinate);
            String coordinateText = String.format("%.2f, %.2f, %.2f", xyz[0], xyz[1], xyz[2]); // Print as two-decimal numbers
            SwingUtilities.invokeLater(() -> positionCoordinateValueLabel.setText(coordinateText));
        }
    }

    /* --------------- Attribute change: Rotation ---------------*/
    private void updateOrientationAngleValueLabel(double updatedOrientationAngle) {
        if (!Double.isNaN(updatedOrientationAngle)) {
            SwingUtilities.invokeLater(() -> orientationDegreesValueLabel.setText(
                    String.format("%.2f°", updatedOrientationAngle))
            );
        }
    }

    /* --------------- Attribute change: Navigation Goal Table ---------------*/
    private void updateNavigationGoalsTable(Object[][] navigationGoalData) {
        final String[] navigationGoalColumnNames = {
                bundle.getString("ros2CommAdapterPanel.navigation_goal_table_column_uuid.text"),
                bundle.getString("ros2CommAdapterPanel.navigation_goal_table_column_last_updated.text"),
                bundle.getString("ros2CommAdapterPanel.navigation_goal_table_column_destination.text"),
                bundle.getString("ros2CommAdapterPanel.navigation_goal_table_column_status.text")
        };
        DefaultTableModel tableModel = new DefaultTableModel(navigationGoalData, navigationGoalColumnNames) {
            @Override
            public boolean isCellEditable(int row, int column) {
                return false;
            }
        };
        SwingUtilities.invokeLater(() -> navigationGoalTable.setModel(tableModel));
    }

    /* --------------- Attribute change: Load handling device ---------------*/
    private void updateVehicleLoadHandlingDevice(List<LoadHandlingDevice> devices) {
        if (devices.size() > 1) {
            LOG.warn("size of load handling devices greater than 1 ({})", devices.size());
        }
        boolean loaded = devices.stream()
                .findFirst()
                .map(LoadHandlingDevice::isFull)
                .orElse(false);
        SwingUtilities.invokeLater(() -> loadHandlingDeviceCheckbox.setSelected(loaded));
    }

    //================================================================================
    // Methods executed on driver enable / disable
    //================================================================================

    /**
     * Enable/disable the input fields and buttons in the "Current position/state" panel.
     * If disabled the user can not change any values or modify the vehicles state.
     *
     * @param enabled boolean indicating if the panel should be enabled
     */
    private void setStatePanelEnabled(boolean enabled) {
        // Top panel pane
        SwingUtilities.invokeLater(() -> namespaceLabel.setEnabled(!enabled));
        SwingUtilities.invokeLater(() -> namespaceTextField.setEnabled(!enabled));
        SwingUtilities.invokeLater(() -> domainIdLabel.setEnabled(!enabled));
        SwingUtilities.invokeLater(() -> domainIdTextField.setEnabled(!enabled));
        SwingUtilities.invokeLater(() -> setEnableButtonTextByEnabledBoolean(enabled));

        // Navigation goals pane
        SwingUtilities.invokeLater(() -> setInitialPointButton.setEnabled(enabled));
        SwingUtilities.invokeLater(() -> dispatchToCoordinateButton.setEnabled(enabled));
        SwingUtilities.invokeLater(() -> dispatchToPointButton.setEnabled(enabled));
        SwingUtilities.invokeLater(() -> navigationGoalTable.setEnabled(enabled));

        // Load handling device
        SwingUtilities.invokeLater(() -> loadHandlingDeviceCheckbox.setEnabled(enabled));

        // Position & Orientation
        SwingUtilities.invokeLater(this::resetAllPositionValues);
    }

    private void setEnableButtonTextByEnabledBoolean(boolean isDriverEnabled) {
        if (isDriverEnabled)
            enableButton.setText(bundle.getString("ros2CommAdapterPanel.button_disable.text"));
        else
            enableButton.setText(bundle.getString("ros2CommAdapterPanel.button_enable.text"));

    }

    private void resetAllPositionValues() {
        String emptyString = " ";
        SwingUtilities.invokeLater(() -> positionPointValueLabel.setText(emptyString));
        SwingUtilities.invokeLater(() -> positionCoordinateValueLabel.setText(emptyString));
        SwingUtilities.invokeLater(() -> positionEstimateValueLabel.setText(emptyString));
        SwingUtilities.invokeLater(() -> orientationDegreesValueLabel.setText(emptyString));
    }
    //================================================================================
    // Methods for sending commands
    //================================================================================

    private void sendCommand(AdapterCommand command) {
        try {
            TCSObjectReference<Vehicle> vehicleRef = getVehicleReference();
            callWrapper.call(() -> vehicleService.sendCommAdapterCommand(vehicleRef, command));
        } catch (Exception ex) {
            LOG.warn("Error sending adapter command '{}'", command, ex);
        }
    }

    private TCSObjectReference<Vehicle> getVehicleReference()
            throws Exception {
        return callWrapper.call(() -> vehicleService.
                fetchObject(Vehicle.class, processModel.getVehicleName())).getReference();
    }


    // CHECKSTYLE:OFF

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        jPanel3 = new javax.swing.JPanel();
        jMenuBar1 = new javax.swing.JMenuBar();
        jMenu1 = new javax.swing.JMenu();
        jMenu2 = new javax.swing.JMenu();
        mainPanel = new javax.swing.JPanel();
        topPanel = new javax.swing.JPanel();
        topPanelLeft = new javax.swing.JPanel();
        enableButtonWrapper = new javax.swing.JPanel();
        enableButton = new javax.swing.JButton();
        nodeStatusLabel = new javax.swing.JLabel();
        inputPanel = new javax.swing.JPanel();
        domainIdPanel = new javax.swing.JPanel();
        domainIdLabel = new javax.swing.JLabel();
        domainIdTextField = new javax.swing.JTextField();
        namespacePanel = new javax.swing.JPanel();
        namespaceLabel = new javax.swing.JLabel();
        namespaceTextField = new javax.swing.JTextField();
        topPanelRight = new javax.swing.JPanel();
        titleLabel = new javax.swing.JLabel();
        versionLabel = new javax.swing.JLabel();
        copyrightLabel = new javax.swing.JLabel();
        licenseLabel = new javax.swing.JLabel();
        emailLabel = new javax.swing.JLabel();
        bottomPanel = new javax.swing.JPanel();
        loadDevicePanel = new javax.swing.JPanel();
        loadHandlingDeviceCheckbox = new javax.swing.JCheckBox();
        vehiclePropertiesPanel = new javax.swing.JPanel();
        vehiclePropertiesLeftPanel = new javax.swing.JPanel();
        vehiclePropertiesLabelsPanel = new javax.swing.JPanel();
        positionPointLabelLabel = new javax.swing.JLabel();
        positionCoordinateLabelLabel = new javax.swing.JLabel();
        orientationDegreesLabelLabel = new javax.swing.JLabel();
        vehiclePropertiesValuesPanel = new javax.swing.JPanel();
        positionPointValueLabel = new javax.swing.JLabel();
        positionCoordinateValueLabel = new javax.swing.JLabel();
        positionEstimateValueLabel = new javax.swing.JLabel();
        orientationDegreesValueLabel = new javax.swing.JLabel();
        testPanel = new javax.swing.JPanel();
        navigationGoalTableScrollPane = new javax.swing.JScrollPane();
        navigationGoalTable = new javax.swing.JTable();
        dispatchPanel = new javax.swing.JPanel();
        setInitialPointButton = new javax.swing.JButton();
        dispatchToCoordinateButton = new javax.swing.JButton();
        dispatchToPointButton = new javax.swing.JButton();

        jMenu1.setText("File");
        jMenuBar1.add(jMenu1);

        jMenu2.setText("Edit");
        jMenuBar1.add(jMenu2);

        setName("Ros2CommAdapterPanel"); // NOI18N
        setLayout(new java.awt.BorderLayout());

        mainPanel.setBackground(new java.awt.Color(0, 156, 130));
        mainPanel.setBorder(javax.swing.BorderFactory.createLineBorder(new java.awt.Color(0, 156, 130), 15));
        mainPanel.setLayout(new java.awt.BorderLayout());

        topPanel.setBackground(new java.awt.Color(254, 254, 254));
        topPanel.setBorder(javax.swing.BorderFactory.createEmptyBorder(5, 5, 10, 15));
        topPanel.setLayout(new java.awt.BorderLayout());

        topPanelLeft.setBackground(new java.awt.Color(254, 254, 254));
        java.util.ResourceBundle bundle = java.util.ResourceBundle.getBundle("nl/saxion/nena/opentcs/commadapter/ros2/Bundle"); // NOI18N
        topPanelLeft.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("EnableAdapter"))); // NOI18N
        topPanelLeft.setPreferredSize(new java.awt.Dimension(300, 125));
        topPanelLeft.setLayout(new java.awt.BorderLayout());

        enableButtonWrapper.setBackground(new java.awt.Color(255, 255, 255));
        enableButtonWrapper.setLayout(new java.awt.BorderLayout());

        enableButton.setText(bundle.getString("ros2CommAdapterPanel.button_enable.text")); // NOI18N
        enableButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                enableButtonActionPerformed(evt);
            }
        });
        enableButtonWrapper.add(enableButton, java.awt.BorderLayout.CENTER);

        topPanelLeft.add(enableButtonWrapper, java.awt.BorderLayout.CENTER);

        nodeStatusLabel.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        nodeStatusLabel.setText(bundle.getString("ros2CommAdapterPanel.label_nodeInactive.text")); // NOI18N
        nodeStatusLabel.setVerticalAlignment(javax.swing.SwingConstants.TOP);
        topPanelLeft.add(nodeStatusLabel, java.awt.BorderLayout.PAGE_END);

        inputPanel.setBackground(new java.awt.Color(254, 254, 254));
        inputPanel.setLayout(new java.awt.BorderLayout());

        domainIdPanel.setBackground(new java.awt.Color(254, 254, 254));
        domainIdPanel.setBorder(javax.swing.BorderFactory.createEmptyBorder(5, 5, 0, 5));
        domainIdPanel.setMaximumSize(new java.awt.Dimension(2147483647, 25));
        domainIdPanel.setLayout(new java.awt.BorderLayout());

        domainIdLabel.setText(bundle.getString("ros2CommAdapterPanel.label_domainId.text")); // NOI18N
        domainIdLabel.setPreferredSize(new java.awt.Dimension(90, 17));
        domainIdPanel.add(domainIdLabel, java.awt.BorderLayout.WEST);

        domainIdTextField.setText("30");
        domainIdTextField.setBorder(javax.swing.BorderFactory.createEtchedBorder());
        domainIdPanel.add(domainIdTextField, java.awt.BorderLayout.CENTER);

        inputPanel.add(domainIdPanel, java.awt.BorderLayout.PAGE_START);

        namespacePanel.setBackground(new java.awt.Color(254, 254, 254));
        namespacePanel.setBorder(javax.swing.BorderFactory.createEmptyBorder(5, 5, 0, 5));
        namespacePanel.setMaximumSize(new java.awt.Dimension(2147483647, 25));
        namespacePanel.setLayout(new java.awt.BorderLayout());

        namespaceLabel.setText(bundle.getString("ros2CommAdapterPanel.label_namespace.text")); // NOI18N
        namespaceLabel.setPreferredSize(new java.awt.Dimension(90, 17));
        namespacePanel.add(namespaceLabel, java.awt.BorderLayout.WEST);

        namespaceTextField.setBorder(javax.swing.BorderFactory.createEtchedBorder());
        namespacePanel.add(namespaceTextField, java.awt.BorderLayout.CENTER);

        inputPanel.add(namespacePanel, java.awt.BorderLayout.PAGE_END);

        topPanelLeft.add(inputPanel, java.awt.BorderLayout.PAGE_START);

        topPanel.add(topPanelLeft, java.awt.BorderLayout.WEST);

        topPanelRight.setBackground(new java.awt.Color(254, 254, 254));
        topPanelRight.setLayout(new javax.swing.BoxLayout(topPanelRight, javax.swing.BoxLayout.PAGE_AXIS));

        titleLabel.setFont(new java.awt.Font("Ubuntu", 1, 18)); // NOI18N
        titleLabel.setForeground(new java.awt.Color(0, 156, 130));
        titleLabel.setText("OpenTCS-NeNa");
        titleLabel.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                titleLabelMouseClicked(evt);
            }
            public void mouseExited(java.awt.event.MouseEvent evt) {
                titleLabelMouseExited(evt);
            }
            public void mouseEntered(java.awt.event.MouseEvent evt) {
                titleLabelMouseEntered(evt);
            }
        });
        topPanelRight.add(titleLabel);

        versionLabel.setText("Release 2.0.1");
        versionLabel.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                versionLabelMouseClicked(evt);
            }
            public void mouseExited(java.awt.event.MouseEvent evt) {
                versionLabelMouseExited(evt);
            }
            public void mouseEntered(java.awt.event.MouseEvent evt) {
                versionLabelMouseEntered(evt);
            }
        });
        topPanelRight.add(versionLabel);

        copyrightLabel.setText("Copyright © 2021 Niels Tiben ");
        copyrightLabel.setBorder(javax.swing.BorderFactory.createEmptyBorder(30, 0, 0, 0));
        topPanelRight.add(copyrightLabel);

        licenseLabel.setText("License info (BSD-3)");
        licenseLabel.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                licenseLabelMouseClicked(evt);
            }
            public void mouseExited(java.awt.event.MouseEvent evt) {
                licenseLabelMouseExited(evt);
            }
            public void mouseEntered(java.awt.event.MouseEvent evt) {
                licenseLabelMouseEntered(evt);
            }
        });
        topPanelRight.add(licenseLabel);

        emailLabel.setFont(new java.awt.Font("Ubuntu", 0, 15)); // NOI18N
        emailLabel.setText("nielstiben@outlook.com");
        emailLabel.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                emailLabelMouseClicked(evt);
            }
            public void mouseExited(java.awt.event.MouseEvent evt) {
                emailLabelMouseExited(evt);
            }
            public void mouseEntered(java.awt.event.MouseEvent evt) {
                emailLabelMouseEntered(evt);
            }
        });
        topPanelRight.add(emailLabel);

        topPanel.add(topPanelRight, java.awt.BorderLayout.EAST);

        mainPanel.add(topPanel, java.awt.BorderLayout.PAGE_START);

        bottomPanel.setBackground(new java.awt.Color(254, 254, 254));
        bottomPanel.setBorder(javax.swing.BorderFactory.createEmptyBorder(0, 5, 0, 5));
        bottomPanel.setPreferredSize(new java.awt.Dimension(200, 50));
        bottomPanel.setLayout(new java.awt.BorderLayout());

        loadDevicePanel.setBackground(new java.awt.Color(254, 254, 254));
        loadDevicePanel.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_loadHandlingDevice.border.title"))); // NOI18N
        loadDevicePanel.setLayout(new java.awt.BorderLayout());

        loadHandlingDeviceCheckbox.setBackground(new java.awt.Color(254, 254, 254));
        loadHandlingDeviceCheckbox.setText("Device loaded");
        loadHandlingDeviceCheckbox.setEnabled(false);
        loadHandlingDeviceCheckbox.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                loadHandlingDeviceCheckboxClicked(evt);
            }
        });
        loadDevicePanel.add(loadHandlingDeviceCheckbox, java.awt.BorderLayout.PAGE_END);

        bottomPanel.add(loadDevicePanel, java.awt.BorderLayout.PAGE_END);

        vehiclePropertiesPanel.setBackground(new java.awt.Color(254, 254, 254));
        vehiclePropertiesPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_vehicle.border.title"))); // NOI18N
        vehiclePropertiesPanel.setName(""); // NOI18N
        vehiclePropertiesPanel.setPreferredSize(new java.awt.Dimension(500, 150));
        vehiclePropertiesPanel.setLayout(new java.awt.BorderLayout());

        vehiclePropertiesLeftPanel.setBackground(new java.awt.Color(255, 255, 255));
        vehiclePropertiesLeftPanel.setPreferredSize(new java.awt.Dimension(340, 100));
        vehiclePropertiesLeftPanel.setLayout(new java.awt.BorderLayout());

        vehiclePropertiesLabelsPanel.setBackground(new java.awt.Color(255, 255, 255));
        vehiclePropertiesLabelsPanel.setBorder(javax.swing.BorderFactory.createEmptyBorder(5, 5, 5, 5));
        vehiclePropertiesLabelsPanel.setPreferredSize(new java.awt.Dimension(170, 100));
        vehiclePropertiesLabelsPanel.setLayout(new javax.swing.BoxLayout(vehiclePropertiesLabelsPanel, javax.swing.BoxLayout.PAGE_AXIS));

        positionPointLabelLabel.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        positionPointLabelLabel.setText(bundle.getString("ros2CommAdapterPanel.label_position_point.text")); // NOI18N
        vehiclePropertiesLabelsPanel.add(positionPointLabelLabel);

        positionCoordinateLabelLabel.setText(bundle.getString("ros2CommAdapterPanel.label_position_coordinate.text")); // NOI18N
        vehiclePropertiesLabelsPanel.add(positionCoordinateLabelLabel);

        orientationDegreesLabelLabel.setText(bundle.getString("ros2CommAdapterPanel.label_orientation_degrees.text")); // NOI18N
        vehiclePropertiesLabelsPanel.add(orientationDegreesLabelLabel);

        vehiclePropertiesLeftPanel.add(vehiclePropertiesLabelsPanel, java.awt.BorderLayout.WEST);

        vehiclePropertiesValuesPanel.setBackground(new java.awt.Color(255, 255, 255));
        vehiclePropertiesValuesPanel.setBorder(javax.swing.BorderFactory.createEmptyBorder(5, 5, 5, 5));
        vehiclePropertiesValuesPanel.setPreferredSize(new java.awt.Dimension(170, 100));
        vehiclePropertiesValuesPanel.setLayout(new javax.swing.BoxLayout(vehiclePropertiesValuesPanel, javax.swing.BoxLayout.PAGE_AXIS));

        positionPointValueLabel.setFont(new java.awt.Font("Ubuntu", 2, 15)); // NOI18N
        positionPointValueLabel.setText(" ");
        vehiclePropertiesValuesPanel.add(positionPointValueLabel);

        positionCoordinateValueLabel.setFont(new java.awt.Font("Ubuntu", 2, 15)); // NOI18N
        positionCoordinateValueLabel.setText(" ");
        vehiclePropertiesValuesPanel.add(positionCoordinateValueLabel);

        positionEstimateValueLabel.setFont(new java.awt.Font("Ubuntu", 2, 15)); // NOI18N
        positionEstimateValueLabel.setText(" ");
        vehiclePropertiesValuesPanel.add(positionEstimateValueLabel);

        orientationDegreesValueLabel.setFont(new java.awt.Font("Ubuntu", 2, 15)); // NOI18N
        orientationDegreesValueLabel.setText(" ");
        vehiclePropertiesValuesPanel.add(orientationDegreesValueLabel);

        vehiclePropertiesLeftPanel.add(vehiclePropertiesValuesPanel, java.awt.BorderLayout.LINE_END);

        vehiclePropertiesPanel.add(vehiclePropertiesLeftPanel, java.awt.BorderLayout.WEST);

        bottomPanel.add(vehiclePropertiesPanel, java.awt.BorderLayout.NORTH);

        testPanel.setBackground(new java.awt.Color(254, 254, 254));
        testPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_navigationGoals.border.title"))); // NOI18N
        testPanel.setLayout(new java.awt.BorderLayout());

        navigationGoalTable.setModel(new javax.swing.table.DefaultTableModel(
            new Object [][] {
                {null, null, null, null},
                {null, null, null, null},
                {null, null, null, null},
                {null, null, null, null}
            },
            new String [] {
                "Title 1", "Title 2", "Title 3", "Title 4"
            }
        ));
        navigationGoalTableScrollPane.setViewportView(navigationGoalTable);

        testPanel.add(navigationGoalTableScrollPane, java.awt.BorderLayout.CENTER);

        dispatchPanel.setBackground(new java.awt.Color(255, 255, 255));
        dispatchPanel.setBorder(javax.swing.BorderFactory.createEmptyBorder(1, 1, 1, 1));

        setInitialPointButton.setText(bundle.getString("ros2CommAdapterPanel.dialog_setInitialPoint.title")); // NOI18N
        setInitialPointButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                setInitialPointButtonActionPerformed(evt);
            }
        });
        dispatchPanel.add(setInitialPointButton);

        dispatchToCoordinateButton.setText(bundle.getString("ros2CommAdapterPanel.button_dispatch_to_coordinate.text")); // NOI18N
        dispatchToCoordinateButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                dispatchToCoordinateButtonActionPerformed(evt);
            }
        });
        dispatchPanel.add(dispatchToCoordinateButton);

        dispatchToPointButton.setText(bundle.getString("ros2CommAdapterPanel.button_dispatch_to_point.text")); // NOI18N
        dispatchToPointButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                dispatchToPointButtonActionPerformed(evt);
            }
        });
        dispatchPanel.add(dispatchToPointButton);

        testPanel.add(dispatchPanel, java.awt.BorderLayout.PAGE_START);

        bottomPanel.add(testPanel, java.awt.BorderLayout.CENTER);

        mainPanel.add(bottomPanel, java.awt.BorderLayout.CENTER);

        add(mainPanel, java.awt.BorderLayout.CENTER);

        getAccessibleContext().setAccessibleName(bundle.getString("ros2CommAdapterPanel.accessibleName")); // NOI18N
    }// </editor-fold>//GEN-END:initComponents

    private void loadHandlingDeviceCheckboxClicked(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_loadHandlingDeviceCheckboxClicked
        List<LoadHandlingDevice> devices = Arrays.asList(
                new LoadHandlingDevice(LOAD_HANDLING_DEVICE_NAME, loadHandlingDeviceCheckbox.isSelected()));
        sendCommand(new SetLoadHandlingDevicesCommand(devices));
    }//GEN-LAST:event_loadHandlingDeviceCheckboxClicked

    private void enableButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_enableButtonActionPerformed
        try {
            Vehicle vehicle = callWrapper.call(() -> vehicleService.fetchObject(Vehicle.class, processModel.getVehicleName()));

            if (this.isAdapterEnabled) {
                // Disable adapter
                callWrapper.call(() -> vehicleService.disableCommAdapter(vehicle.getReference()));
                updateIsAdapterEnabled(false);

            } else {
                // Enable adapter:
                String namespaceString = namespaceTextField.getText();
                if (!InputValidationLib.isValidNamespace(namespaceString)) {
                    // Invalid namespace
                    showMessageDialog(this, bundle.getString("ros2CommAdapterPanel.messageDialogInvalidNamespace.text"));
                    return;
                }

                String domainIdString = domainIdTextField.getText();
                if (!InputValidationLib.isValidDomainId(domainIdString)) {
                    // Invalid namespace
                    showMessageDialog(this, bundle.getString("ros2CommAdapterPanel.messageDialogInvalidDomainId.text"));
                    return;
                }

                sendCommand(new SetNamespaceCommand(namespaceString));
                sendCommand(new SetDomainIdCommand(Integer.parseInt(domainIdString)));

                callWrapper.call(() -> vehicleService.enableCommAdapter(vehicle.getReference()));
            }
        } catch (Exception exception) {
            LOG.error("Error enabling/disabling ROS2 adapter: ", exception);
        }


    }//GEN-LAST:event_enableButtonActionPerformed

    private void dispatchToCoordinateButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_dispatchToCoordinateButtonActionPerformed
        if (!dispatchToCoordinateButton.isEnabled()) {
            return; // Button is not enabled.
        }

        InputPanel panel = new CoordinateInputPanel.Builder(bundle.getString("ros2CommAdapterPanel.dialog_dispatchVehicleToCoordinate.title")).build();
        InputDialog dialog = new InputDialog(panel);
        dialog.setVisible(true);

        // Get dialog result and set vehicle precise position
        if (dialog.getReturnStatus() == InputDialog.ReturnStatus.ACCEPTED) {
            // Set new precise position
            Triple triple;
            String[] newPos = (String[]) dialog.getInput();
            try {
                triple = UnitConverterLib.convertCoordinatesInMeterToTriple(
                        Double.parseDouble(newPos[0]),
                        Double.parseDouble(newPos[1]),
                        Double.parseDouble(newPos[2])
                );
            } catch (NumberFormatException | NullPointerException e) {
                return;
            }
            sendCommand(new DispatchToCoordinateCommand(triple));
        }

    }//GEN-LAST:event_dispatchToCoordinateButtonActionPerformed

    private void dispatchToPointButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_dispatchToPointButtonActionPerformed
        List<Point> pointList = getModelPointList();
        if (pointList == null) {
            return;
        }

        // Get currently selected point
        Point currentPoint = null;
        String currentPointName = processModel.getVehiclePosition();
        for (org.opentcs.data.model.Point p : pointList) {
            if (p != null && p.getName().equals(currentPointName)) {
                currentPoint = p;
                break;
            }
        }
        // Create panel and dialog
        InputPanel panel = new PointListInputPanel.Builder<>(bundle.getString("ros2CommAdapterPanel.dialog_dispatchToPoint.title"), pointList)
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
                // Do nothing
            } else {
                sendCommand(new DispatchToPointCommand(((Point) item)));
            }
        }
    }//GEN-LAST:event_dispatchToPointButtonActionPerformed

    private void setInitialPointButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_setInitialPointButtonActionPerformed
        // Prepare list of model points
        List<Point> pointList = getModelPointList();
        if (pointList == null) {
            return;
        }

        // Create panel and dialog
        InputPanel panel = new PointListInputPanel.Builder<>(bundle.getString("ros2CommAdapterPanel.dialog_setInitialPoint.title"), pointList)
                .setSelectionRepresenter(x -> x == null ? "" : x.getName())
                .setLabel(bundle.getString("ros2CommAdapterPanel.label_position.text"))
                .setEditable(true)
                .setRenderer(new StringListCellRenderer<>(x -> x == null ? "" : x.getName()))
                .build();
        InputDialog dialog = new InputDialog(panel);
        dialog.setVisible(true);
        // Get result from dialog and set vehicle position
        if (dialog.getReturnStatus() == InputDialog.ReturnStatus.ACCEPTED) {
            Object item = dialog.getInput();
            if (item == null) {
                // Do nothing
            } else {
                sendCommand(new SetInitialPointCommand(((Point) item)));
            }
        }
    }

    private List<Point> getModelPointList() {
        // Prepare list of model points
        Set<org.opentcs.data.model.Point> pointSet;
        try {
            pointSet = callWrapper.call(() -> vehicleService.fetchObjects(org.opentcs.data.model.Point.class));
        } catch (Exception ex) {
            LOG.warn("Error fetching points", ex);
            return null;
        }

        List<org.opentcs.data.model.Point> pointList = new ArrayList<>(pointSet);
        pointList.sort(Comparators.objectsByName());
        pointList.add(0, null);

        return pointList;
    }

//GEN-LAST:event_setInitialPointButtonActionPerformed

  private void licenseLabelMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_licenseLabelMouseClicked
      try {
          Desktop.getDesktop().browse(URI.create("https://github.com/nielstiben/openTCS-NeNa/blob/RELEASE2.0/LICENSE"));
      }
      catch (java.io.IOException e) {
          LOG.warn("Could not open license page in browser");
      }
  }//GEN-LAST:event_licenseLabelMouseClicked

  private void licenseLabelMouseEntered(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_licenseLabelMouseEntered
    licenseLabel.setCursor(new Cursor(Cursor.HAND_CURSOR));
    
  }//GEN-LAST:event_licenseLabelMouseEntered

  private void licenseLabelMouseExited(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_licenseLabelMouseExited
      licenseLabel.setCursor(new Cursor(Cursor.DEFAULT_CURSOR));
  }//GEN-LAST:event_licenseLabelMouseExited

  private void emailLabelMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_emailLabelMouseClicked
      try {
          Desktop desktop = Desktop.getDesktop();
          String message = "mailto:nielstiben@outlook.com";
          URI uri = URI.create(message);
          desktop.mail(uri);
      } catch (IOException e) {
          showMessageDialog(this, bundle.getString("ros2CommAdapterPanel.messageDefaultEmailNotSet.text"));
      }  }//GEN-LAST:event_emailLabelMouseClicked

  private void emailLabelMouseEntered(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_emailLabelMouseEntered
      emailLabel.setCursor(new Cursor(Cursor.HAND_CURSOR));
  }//GEN-LAST:event_emailLabelMouseEntered

  private void emailLabelMouseExited(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_emailLabelMouseExited
      emailLabel.setCursor(new Cursor(Cursor.DEFAULT_CURSOR));
  }//GEN-LAST:event_emailLabelMouseExited

  private void titleLabelMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_titleLabelMouseClicked
      try {
          Desktop.getDesktop().browse(URI.create("https://github.com/nielstiben/openTCS-NeNa/tree/RELEASE2.0"));
      }
      catch (java.io.IOException e) {
          LOG.warn("Could not open GitHub page in browser");
      }  }//GEN-LAST:event_titleLabelMouseClicked

  private void titleLabelMouseEntered(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_titleLabelMouseEntered
      licenseLabel.setCursor(new Cursor(Cursor.HAND_CURSOR));
  }//GEN-LAST:event_titleLabelMouseEntered

  private void titleLabelMouseExited(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_titleLabelMouseExited
      licenseLabel.setCursor(new Cursor(Cursor.DEFAULT_CURSOR));
  }//GEN-LAST:event_titleLabelMouseExited

  private void versionLabelMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_versionLabelMouseClicked
      try {
          Desktop.getDesktop().browse(URI.create("https://github.com/nielstiben/openTCS-NeNa/tree/RELEASE2.0"));
      }
      catch (java.io.IOException e) {
          LOG.warn("Could not open GitHub page in browser");
      }   }//GEN-LAST:event_versionLabelMouseClicked

  private void versionLabelMouseEntered(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_versionLabelMouseEntered
      licenseLabel.setCursor(new Cursor(Cursor.HAND_CURSOR));
  }//GEN-LAST:event_versionLabelMouseEntered

  private void versionLabelMouseExited(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_versionLabelMouseExited
      licenseLabel.setCursor(new Cursor(Cursor.DEFAULT_CURSOR));
  }//GEN-LAST:event_versionLabelMouseExited

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JPanel bottomPanel;
    private javax.swing.JLabel copyrightLabel;
    private javax.swing.JPanel dispatchPanel;
    private javax.swing.JButton dispatchToCoordinateButton;
    private javax.swing.JButton dispatchToPointButton;
    private javax.swing.JLabel domainIdLabel;
    private javax.swing.JPanel domainIdPanel;
    private javax.swing.JTextField domainIdTextField;
    private javax.swing.JLabel emailLabel;
    private javax.swing.JButton enableButton;
    private javax.swing.JPanel enableButtonWrapper;
    private javax.swing.JPanel inputPanel;
    private javax.swing.JMenu jMenu1;
    private javax.swing.JMenu jMenu2;
    private javax.swing.JMenuBar jMenuBar1;
    private javax.swing.JPanel jPanel3;
    private javax.swing.JLabel licenseLabel;
    private javax.swing.JPanel loadDevicePanel;
    private javax.swing.JCheckBox loadHandlingDeviceCheckbox;
    private javax.swing.JPanel mainPanel;
    private javax.swing.JLabel namespaceLabel;
    private javax.swing.JPanel namespacePanel;
    private javax.swing.JTextField namespaceTextField;
    private javax.swing.JTable navigationGoalTable;
    private javax.swing.JScrollPane navigationGoalTableScrollPane;
    private javax.swing.JLabel nodeStatusLabel;
    private javax.swing.JLabel orientationDegreesLabelLabel;
    private javax.swing.JLabel orientationDegreesValueLabel;
    private javax.swing.JLabel positionCoordinateLabelLabel;
    private javax.swing.JLabel positionCoordinateValueLabel;
    private javax.swing.JLabel positionEstimateValueLabel;
    private javax.swing.JLabel positionPointLabelLabel;
    private javax.swing.JLabel positionPointValueLabel;
    private javax.swing.JButton setInitialPointButton;
    private javax.swing.JPanel testPanel;
    private javax.swing.JLabel titleLabel;
    private javax.swing.JPanel topPanel;
    private javax.swing.JPanel topPanelLeft;
    private javax.swing.JPanel topPanelRight;
    private javax.swing.JPanel vehiclePropertiesLabelsPanel;
    private javax.swing.JPanel vehiclePropertiesLeftPanel;
    private javax.swing.JPanel vehiclePropertiesPanel;
    private javax.swing.JPanel vehiclePropertiesValuesPanel;
    private javax.swing.JLabel versionLabel;
    // End of variables declaration//GEN-END:variables
    // CHECKSTYLE:ON
}
