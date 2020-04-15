package nl.saxion.nena.opentcs.commadapter.ros2.control_center;

import com.google.inject.assistedinject.Assisted;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands.SetLoadHandlingDevicesCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands.SetupRos2ConnectionCommand;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.lib.InputValidationLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.virtual_vehicle.Ros2ProcessModelTO;
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
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.ResourceBundle;

import static java.util.Objects.requireNonNull;
import static javax.swing.JOptionPane.showMessageDialog;
import static nl.saxion.nena.opentcs.commadapter.ros2.common.I18nLoopbackCommAdapter.BUNDLE_PATH;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel.Attribute.CONNECTION_STATUS;

/**
 * The panel corresponding to the Ros2CommAdapter.
 *
 * @author Iryna Felko (Fraunhofer IML)
 * @author Stefan Walter (Fraunhofer IML)
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class Ros2CommAdapterPanel
        extends VehicleCommAdapterPanel {

    private static final ResourceBundle bundle = ResourceBundle.getBundle(BUNDLE_PATH);
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapterPanel.class);
    private final VehicleService vehicleService;
    private Ros2ProcessModelTO processModel;
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
     * @param attributeChanged The changed attribute.
     * @param newProcessModel Updated processModel
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

    private void updateRos2ProcessModelData(String attributeChanged,
                                            Ros2ProcessModelTO processModel) {

        if (attributeChanged.equals(CONNECTION_STATUS.name())) {
            updateConnectionStatus(processModel.getConnectionStatus());
        }
    }

    private void updateConnectionStatus(String connectionStatus) {
        SwingUtilities.invokeLater(() -> connectionStatusLabel.setText(connectionStatus.toLowerCase()));
    }

    private void updateVehicleProcessModelData(String attributeChanged,
                                               VehicleProcessModelTO processModel) {
        if (Objects.equals(attributeChanged,
                VehicleProcessModel.Attribute.COMM_ADAPTER_ENABLED.name())) {
            updateCommAdapterEnabled(processModel.isCommAdapterEnabled());
        } else if (Objects.equals(attributeChanged,
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
        // Connection properties
        SwingUtilities.invokeLater(() -> domainIdTextField.setEnabled(enabled));
        SwingUtilities.invokeLater(() -> connectButton.setEnabled(enabled));
        SwingUtilities.invokeLater(() -> domainIdLabel.setEnabled(enabled));
        SwingUtilities.invokeLater(() -> connectionStatusLabel.setEnabled(enabled));

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
        adapterStatusPanel = new javax.swing.JPanel();
        chkBoxEnable = new javax.swing.JCheckBox();
        loadDevicePanel = new javax.swing.JPanel();
        lHDCheckbox = new javax.swing.JCheckBox();
        topPanelRight = new javax.swing.JPanel();
        saxionLogoLabel = new javax.swing.JLabel();
        bottomPanel = new javax.swing.JPanel();
        connectionPropertiesPanel = new javax.swing.JPanel();
        connectPanel = new javax.swing.JPanel();
        domainIdLabel = new javax.swing.JLabel();
        domainIdTextField = new javax.swing.JFormattedTextField();
        connectButton = new javax.swing.JButton();
        connectionStatusLabel = new javax.swing.JLabel();
        testPanel = new javax.swing.JPanel();

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
        topPanelLeft.setMinimumSize(new java.awt.Dimension(200, 110));
        topPanelLeft.setPreferredSize(new java.awt.Dimension(300, 110));
        topPanelLeft.setLayout(new java.awt.BorderLayout());

        adapterStatusPanel.setBackground(new java.awt.Color(254, 254, 254));
        adapterStatusPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, bundle.getString("ros2CommAdapterPanel.panel_adapterStatus.border.title"), javax.swing.border.TitledBorder.LEFT, javax.swing.border.TitledBorder.DEFAULT_POSITION)); // NOI18N
        adapterStatusPanel.setName("adapterStatusPanel"); // NOI18N
        adapterStatusPanel.setLayout(new java.awt.BorderLayout());

        chkBoxEnable.setBackground(new java.awt.Color(254, 254, 254));
        chkBoxEnable.setText(bundle.getString("ros2CommAdapterPanel.checkBox_enableAdapter.text")); // NOI18N
        chkBoxEnable.setName("chkBoxEnable"); // NOI18N
        chkBoxEnable.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                chkBoxEnableActionPerformed(evt);
            }
        });
        adapterStatusPanel.add(chkBoxEnable, java.awt.BorderLayout.CENTER);

        topPanelLeft.add(adapterStatusPanel, java.awt.BorderLayout.PAGE_START);

        loadDevicePanel.setBackground(new java.awt.Color(254, 254, 254));
        java.util.ResourceBundle bundle = java.util.ResourceBundle.getBundle("nl/saxion/nena/opentcs/commadapter/ros2/Bundle"); // NOI18N
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

        topPanelLeft.add(loadDevicePanel, java.awt.BorderLayout.PAGE_END);

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

        connectionPropertiesPanel.setBackground(new java.awt.Color(254, 254, 254));
        connectionPropertiesPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(bundle.getString("ros2CommAdapterPanel.panel_connection.border.title"))); // NOI18N
        connectionPropertiesPanel.setMinimumSize(new java.awt.Dimension(500, 200));
        connectionPropertiesPanel.setName(""); // NOI18N
        connectionPropertiesPanel.setPreferredSize(new java.awt.Dimension(500, 93));
        connectionPropertiesPanel.setLayout(new java.awt.BorderLayout());

        connectPanel.setBackground(new java.awt.Color(254, 254, 254));
        connectPanel.setMaximumSize(new java.awt.Dimension(2147483647, 25));
        connectPanel.setLayout(new java.awt.BorderLayout());

        domainIdLabel.setText(bundle.getString("ros2CommAdapterPanel.label_domain_id.text")); // NOI18N
        domainIdLabel.setEnabled(false);
        connectPanel.add(domainIdLabel, java.awt.BorderLayout.WEST);

        domainIdTextField.setText(bundle.getString("ros2CommAdapterPanel.textField_domainId")); // NOI18N
        domainIdTextField.setBorder(javax.swing.BorderFactory.createEtchedBorder());
        domainIdTextField.setEnabled(false);
        connectPanel.add(domainIdTextField, java.awt.BorderLayout.CENTER);

        connectButton.setText(bundle.getString("ros2CommAdapterPanel.button_connect.text")); // NOI18N
        connectButton.setEnabled(false);
        connectButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                connectButtonActionPerformed(evt);
            }
        });
        connectPanel.add(connectButton, java.awt.BorderLayout.EAST);

        connectionPropertiesPanel.add(connectPanel, java.awt.BorderLayout.PAGE_START);

        connectionStatusLabel.setText(bundle.getString("ros2CommAdapterPanel.label_notConnected.text")); // NOI18N
        connectionStatusLabel.setVerticalAlignment(javax.swing.SwingConstants.TOP);
        connectionStatusLabel.setEnabled(false);
        connectionPropertiesPanel.add(connectionStatusLabel, java.awt.BorderLayout.CENTER);

        bottomPanel.add(connectionPropertiesPanel, java.awt.BorderLayout.NORTH);

        testPanel.setBackground(new java.awt.Color(254, 254, 254));
        testPanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Test"));
        bottomPanel.add(testPanel, java.awt.BorderLayout.CENTER);

        mainPanel.add(bottomPanel, java.awt.BorderLayout.CENTER);

        add(mainPanel, java.awt.BorderLayout.CENTER);

        getAccessibleContext().setAccessibleName(bundle.getString("ros2CommAdapterPanel.accessibleName")); // NOI18N
    }// </editor-fold>//GEN-END:initComponents

    private void chkBoxEnableActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_chkBoxEnableActionPerformed
        try {
            Vehicle vehicle = callWrapper.call(() -> vehicleService.fetchObject(Vehicle.class, processModel.getVehicleName()));

            if (chkBoxEnable.isSelected()) {
                callWrapper.call(() -> vehicleService.enableCommAdapter(vehicle.getReference()));
            } else {
                callWrapper.call(() -> vehicleService.disableCommAdapter(vehicle.getReference()));
            }

            setStatePanelEnabled(chkBoxEnable.isSelected());
        } catch (Exception ex) {
            LOG.warn("Error enabling/disabling comm adapter", ex);
        }
    }//GEN-LAST:event_chkBoxEnableActionPerformed

    private void lHDCheckboxClicked(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_lHDCheckboxClicked
        List<LoadHandlingDevice> devices = Arrays.asList(
                new LoadHandlingDevice(Ros2CommAdapter.LHD_NAME, lHDCheckbox.isSelected()));
        sendCommand(new SetLoadHandlingDevicesCommand(devices));
    }//GEN-LAST:event_lHDCheckboxClicked

    private void connectButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_connectButtonActionPerformed
        // 1. Validate Domain ID
        String domainIdString = domainIdTextField.getText();
        if (!InputValidationLib.isValidDomainId(domainIdString)) {
            // Invalid domain ID
            showMessageDialog(this, bundle.getString("ros2CommAdapterPanel.messageDialog.text"));
            return;
        }

        // 2. Disable Button
        connectButton.setEnabled(false);

        // 3. Establish connection
        int domainId = Integer.parseInt(domainIdString);
        sendCommand(new SetupRos2ConnectionCommand(domainId));
    }//GEN-LAST:event_connectButtonActionPerformed

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JPanel adapterStatusPanel;
    private javax.swing.JPanel bottomPanel;
    private javax.swing.JCheckBox chkBoxEnable;
    private javax.swing.JButton connectButton;
    private javax.swing.JPanel connectPanel;
    private javax.swing.JPanel connectionPropertiesPanel;
    private javax.swing.JLabel connectionStatusLabel;
    private javax.swing.JLabel domainIdLabel;
    private javax.swing.JTextField domainIdTextField;
    private javax.swing.JMenu jMenu1;
    private javax.swing.JMenu jMenu2;
    private javax.swing.JMenuBar jMenuBar1;
    private javax.swing.JPanel jPanel3;
    private javax.swing.JCheckBox lHDCheckbox;
    private javax.swing.JPanel loadDevicePanel;
    private javax.swing.JPanel mainPanel;
    private javax.swing.JLabel saxionLogoLabel;
    private javax.swing.JPanel testPanel;
    private javax.swing.JPanel topPanel;
    private javax.swing.JPanel topPanelLeft;
    private javax.swing.JPanel topPanelRight;
    // End of variables declaration//GEN-END:variables
    // CHECKSTYLE:ON
}
