package nl.saxion.nena.opentcs.commadapter.ros2.control_center;

import com.google.inject.assistedinject.Assisted;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands.DispatchToCoordinateCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands.SetDomainIdCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands.SetLoadHandlingDevicesCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.gui_components.InputDialog;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.gui_components.InputPanel;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.gui_components.NewGoalInputPanel;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.lib.InputValidationLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModelTO;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.NodeStatus;
import org.opentcs.components.kernel.services.VehicleService;
import org.opentcs.customizations.ServiceCallWrapper;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.opentcs.drivers.vehicle.management.VehicleCommAdapterPanel;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.CallWrapper;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.inject.Inject;
import javax.swing.*;
import javax.swing.table.DefaultTableModel;
import java.awt.*;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.ResourceBundle;

import static java.util.Objects.requireNonNull;
import static javax.swing.JOptionPane.showMessageDialog;
import static nl.saxion.nena.opentcs.commadapter.ros2.common.I18nLoopbackCommAdapter.BUNDLE_PATH;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel.Attribute.NAVIGATION_GOALS;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel.Attribute.NODE_STATUS;

/**
 * The panel corresponding to the Ros2CommAdapter.
 *
 * @author Iryna Felko (Fraunhofer IML)
 * @author Stefan Walter (Fraunhofer IML)
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class Ros2CommAdapterPanel extends VehicleCommAdapterPanel {
    private boolean isAdapterEnabled = false;
    private static final ResourceBundle bundle = ResourceBundle.getBundle(BUNDLE_PATH);
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapterPanel.class);
    private final VehicleService vehicleService;
    private Ros2ProcessModelTO processModel;
    private final CallWrapper callWrapper;
    private final String[] navigationGoalColumnNames = {"UUID", "Status", "Date"};


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
            processModelChange(attribute.name(), processModel);
        }
        for (Ros2ProcessModel.Attribute attribute : Ros2ProcessModel.Attribute.values()) {
            processModelChange(attribute.name(), processModel);
        }
    }

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

        processModel = (Ros2ProcessModelTO) newProcessModel;
        updateRos2ProcessModelData(attributeChanged, processModel);
        updateVehicleProcessModelData(attributeChanged, processModel);
    }

    private void updateRos2ProcessModelData(String attributeChanged, Ros2ProcessModelTO processModel) {

        if (attributeChanged.equals(NODE_STATUS.name())) {
            updateNodeStatus(processModel.getNodeStatus());
        } else if (attributeChanged.equals(NAVIGATION_GOALS.name())) {
            updateNavigationGoalsTable(processModel.getNavigationGoalTable());
        }
    }

    private void updateNodeStatus(String nodeStatus) {
        if (nodeStatus.equals(NodeStatus.NOT_ACTIVE.name())) {
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setText("Node is not active"));
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setForeground(Color.BLACK));
        } else if (nodeStatus.equals(NodeStatus.INITIATING.name())) {
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setText("Node is initiating"));
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setForeground(Color.ORANGE));
        } else if (nodeStatus.equals(NodeStatus.ACTIVE.name())) {
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setText("Node is active"));
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setForeground(Color.GREEN));
        } else {
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setText("Node has an unknown state"));
            SwingUtilities.invokeLater(() -> nodeStatusLabel.setForeground(Color.RED));
        }
    }

    private void updateVehicleProcessModelData(String attributeChanged, VehicleProcessModelTO processModel) {
        if (Objects.equals(attributeChanged,
                VehicleProcessModel.Attribute.COMM_ADAPTER_ENABLED.name())) {
            updateIsAdapterEnabled(processModel.isCommAdapterEnabled());
        } else if (Objects.equals(attributeChanged,
                VehicleProcessModel.Attribute.LOAD_HANDLING_DEVICES.name())) {
            updateVehicleLoadHandlingDevice(processModel.getLoadHandlingDevices());
        }
    }

    private void updateNavigationGoalsTable(Object[][] navigationGoalData) {
        DefaultTableModel tableModel = new DefaultTableModel(navigationGoalData, navigationGoalColumnNames) {
            @Override
            public boolean isCellEditable(int row, int column) {
                return false;
            }
        };
        SwingUtilities.invokeLater(() -> navigationGoalTable.setModel(tableModel));
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


    private void updateIsAdapterEnabled(boolean isEnabled) {
        this.isAdapterEnabled = isEnabled;
        SwingUtilities.invokeLater(() -> setStatePanelEnabled(this.isAdapterEnabled));
    }

    /**
     * Enable/disable the input fields and buttons in the "Current position/state" panel.
     * If disabled the user can not change any values or modify the vehicles state.
     *
     * @param enabled boolean indicating if the panel should be enabled
     */
    private void setStatePanelEnabled(boolean enabled) {
        // Top panel pane
        SwingUtilities.invokeLater(() -> domainIdLabel.setEnabled(!enabled));
        SwingUtilities.invokeLater(() -> domainIdTextField.setEnabled(!enabled));
        SwingUtilities.invokeLater(() -> setEnableButtonTextByEnabledBoolean(enabled));

        // Navigation goals pane
        SwingUtilities.invokeLater(() -> newGoalButton.setEnabled(enabled));

    }

    private void setEnableButtonTextByEnabledBoolean(boolean isDriverEnabled) {
        if (isDriverEnabled)
            enableButton.setText(bundle.getString("ros2CommAdapterPanel.button_disable.text"));
        else
            enableButton.setText(bundle.getString("ros2CommAdapterPanel.button_enable.text"));

    }

    private TCSObjectReference<Vehicle> getVehicleReference()
            throws Exception {
        return callWrapper.call(() -> vehicleService.
                fetchObject(Vehicle.class, processModel.getVehicleName())).getReference();
    }

    private void sendCommand(AdapterCommand command) {
        try {
            TCSObjectReference<Vehicle> vehicleRef = getVehicleReference();
            callWrapper.call(() -> vehicleService.sendCommAdapterCommand(vehicleRef, command));
        } catch (Exception ex) {
            LOG.warn("Error sending adapter command '{}'", command, ex);
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

        jPanel3 = new javax.swing.JPanel();
        jMenuBar1 = new javax.swing.JMenuBar();
        jMenu1 = new javax.swing.JMenu();
        jMenu2 = new javax.swing.JMenu();
        mainPanel = new javax.swing.JPanel();
        topPanel = new javax.swing.JPanel();
        topPanelLeft = new javax.swing.JPanel();
        domainIdPanel = new javax.swing.JPanel();
        domainIdLabel = new javax.swing.JLabel();
        domainIdTextField = new javax.swing.JTextField();
        enableButtonWrapper = new javax.swing.JPanel();
        enableButton = new javax.swing.JButton();
        nodeStatusLabel = new javax.swing.JLabel();
        topPanelRight = new javax.swing.JPanel();
        saxionLogoLabel = new javax.swing.JLabel();
        bottomPanel = new javax.swing.JPanel();
        loadDevicePanel = new javax.swing.JPanel();
        lHDCheckbox = new javax.swing.JCheckBox();
        connectionPropertiesPanel = new javax.swing.JPanel();
        testPanel = new javax.swing.JPanel();
        navigationGoalTableScrollPane = new javax.swing.JScrollPane();
        navigationGoalTable = new javax.swing.JTable();
        newGoalButton = new javax.swing.JButton();

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
        topPanelLeft.setPreferredSize(new java.awt.Dimension(300, 110));
        topPanelLeft.setLayout(new java.awt.BorderLayout());

        domainIdPanel.setBackground(new java.awt.Color(254, 254, 254));
        domainIdPanel.setBorder(javax.swing.BorderFactory.createEmptyBorder(5, 5, 0, 5));
        domainIdPanel.setMaximumSize(new java.awt.Dimension(2147483647, 25));
        domainIdPanel.setLayout(new java.awt.BorderLayout());

        domainIdLabel.setText(bundle.getString("ros2CommAdapterPanel.label_domain_id.text")); // NOI18N
        domainIdPanel.add(domainIdLabel, java.awt.BorderLayout.WEST);

        domainIdTextField.setText(bundle.getString("ros2CommAdapterPanel.textField_domainId")); // NOI18N
        domainIdTextField.setBorder(javax.swing.BorderFactory.createEtchedBorder());
        domainIdPanel.add(domainIdTextField, java.awt.BorderLayout.CENTER);

        topPanelLeft.add(domainIdPanel, java.awt.BorderLayout.PAGE_START);

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

        topPanel.add(topPanelLeft, java.awt.BorderLayout.WEST);

        topPanelRight.setBackground(new java.awt.Color(254, 254, 254));
        topPanelRight.setLayout(new java.awt.BorderLayout());

        saxionLogoLabel.setBackground(new java.awt.Color(254, 254, 254));
        saxionLogoLabel.setIcon(new javax.swing.ImageIcon(getClass().getResource("/nl/saxion/nena/opentcs/commadapter/ros2/res/logos/saxion.png"))); // NOI18N
        topPanelRight.add(saxionLogoLabel, java.awt.BorderLayout.CENTER);

        topPanel.add(topPanelRight, java.awt.BorderLayout.EAST);

        mainPanel.add(topPanel, java.awt.BorderLayout.PAGE_START);

        bottomPanel.setBackground(new java.awt.Color(254, 254, 254));
        bottomPanel.setBorder(javax.swing.BorderFactory.createEmptyBorder(0, 5, 0, 5));
        bottomPanel.setPreferredSize(new java.awt.Dimension(200, 50));
        bottomPanel.setLayout(new java.awt.BorderLayout());

        loadDevicePanel.setBackground(new java.awt.Color(254, 254, 254));
        loadDevicePanel.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_loadHandlingDevice.border.title"))); // NOI18N
        loadDevicePanel.setLayout(new java.awt.BorderLayout());

        lHDCheckbox.setBackground(new java.awt.Color(254, 254, 254));
        lHDCheckbox.setText("Device loaded");
        lHDCheckbox.setEnabled(false);
        lHDCheckbox.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                lHDCheckboxClicked(evt);
            }
        });
        loadDevicePanel.add(lHDCheckbox, java.awt.BorderLayout.PAGE_END);

        bottomPanel.add(loadDevicePanel, java.awt.BorderLayout.PAGE_END);

        connectionPropertiesPanel.setBackground(new java.awt.Color(254, 254, 254));
        connectionPropertiesPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_connection.border.title"))); // NOI18N
        connectionPropertiesPanel.setMinimumSize(new java.awt.Dimension(500, 200));
        connectionPropertiesPanel.setName(""); // NOI18N
        connectionPropertiesPanel.setPreferredSize(new java.awt.Dimension(500, 93));
        connectionPropertiesPanel.setLayout(new java.awt.BorderLayout());
        bottomPanel.add(connectionPropertiesPanel, java.awt.BorderLayout.NORTH);

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
        navigationGoalTable.setShowGrid(true);
        navigationGoalTableScrollPane.setViewportView(navigationGoalTable);

        testPanel.add(navigationGoalTableScrollPane, java.awt.BorderLayout.CENTER);

        newGoalButton.setText(bundle.getString("ros2CommAdapterPanel.button_new_goal.text")); // NOI18N
        newGoalButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                newGoalButtonActionPerformed(evt);
            }
        });
        testPanel.add(newGoalButton, java.awt.BorderLayout.PAGE_START);

        bottomPanel.add(testPanel, java.awt.BorderLayout.CENTER);

        mainPanel.add(bottomPanel, java.awt.BorderLayout.CENTER);

        add(mainPanel, java.awt.BorderLayout.CENTER);

        getAccessibleContext().setAccessibleName(bundle.getString("ros2CommAdapterPanel.accessibleName")); // NOI18N
    }// </editor-fold>//GEN-END:initComponents

    private void lHDCheckboxClicked(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_lHDCheckboxClicked
        List<LoadHandlingDevice> devices = Arrays.asList(
                new LoadHandlingDevice(Ros2CommAdapter.LHD_NAME, lHDCheckbox.isSelected()));
        sendCommand(new SetLoadHandlingDevicesCommand(devices));
    }//GEN-LAST:event_lHDCheckboxClicked

    private void enableButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_enableButtonActionPerformed
        try {
            Vehicle vehicle = callWrapper.call(() -> vehicleService.fetchObject(Vehicle.class, processModel.getVehicleName()));

            if (this.isAdapterEnabled) {
                // Disable adapter
                callWrapper.call(() -> vehicleService.disableCommAdapter(vehicle.getReference()));
                updateIsAdapterEnabled(false);

            } else {
                // Enable adapter:
                String domainIdString = domainIdTextField.getText();
                if (!InputValidationLib.isValidDomainId(domainIdString)) {
                    // Invalid domain ID
                    showMessageDialog(this, bundle.getString("ros2CommAdapterPanel.messageDialog.text"));
                    return;
                }

                int domainId = Integer.parseInt(domainIdString);
                sendCommand(new SetDomainIdCommand(domainId));

                callWrapper.call(() -> vehicleService.enableCommAdapter(vehicle.getReference()));
            }
        } catch (Exception exception) {
            LOG.error("Error enabling/disabling ROS2 adapter: ", exception);
        }


    }//GEN-LAST:event_enableButtonActionPerformed

    private void newGoalButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_newGoalButtonActionPerformed
        // TODO add your handling code here:
        if(!newGoalButton.isEnabled()){
            return; // Button is not enabled.
        }

        // Create panel and dialog
        InputPanel panel = new NewGoalInputPanel.Builder<>(bundle.getString("ros2CommAdapterPanel.dialog_newGoal.title"))
                .build();
        InputDialog dialog = new InputDialog(panel);
        dialog.setVisible(true);

        // Get results from dialog
        if(dialog.getReturnStatus() == InputDialog.ReturnStatus.ACCEPTED) {
            double[] coordinates = (double[]) dialog.getInput();
            // Todo: better input validation
            if(coordinates != null){
                DispatchToCoordinateCommand command = new DispatchToCoordinateCommand(coordinates[0], coordinates[1]);
                sendCommand(command);
            }
        }

    }//GEN-LAST:event_newGoalButtonActionPerformed

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JPanel bottomPanel;
    private javax.swing.JPanel connectionPropertiesPanel;
    private javax.swing.JLabel domainIdLabel;
    private javax.swing.JPanel domainIdPanel;
    private javax.swing.JTextField domainIdTextField;
    private javax.swing.JButton enableButton;
    private javax.swing.JPanel enableButtonWrapper;
    private javax.swing.JMenu jMenu1;
    private javax.swing.JMenu jMenu2;
    private javax.swing.JMenuBar jMenuBar1;
    private javax.swing.JPanel jPanel3;
    private javax.swing.JCheckBox lHDCheckbox;
    private javax.swing.JPanel loadDevicePanel;
    private javax.swing.JPanel mainPanel;
    private javax.swing.JTable navigationGoalTable;
    private javax.swing.JScrollPane navigationGoalTableScrollPane;
    private javax.swing.JButton newGoalButton;
    private javax.swing.JLabel nodeStatusLabel;
    private javax.swing.JLabel saxionLogoLabel;
    private javax.swing.JPanel testPanel;
    private javax.swing.JPanel topPanel;
    private javax.swing.JPanel topPanelLeft;
    private javax.swing.JPanel topPanelRight;
    // End of variables declaration//GEN-END:variables
    // CHECKSTYLE:ON
}
