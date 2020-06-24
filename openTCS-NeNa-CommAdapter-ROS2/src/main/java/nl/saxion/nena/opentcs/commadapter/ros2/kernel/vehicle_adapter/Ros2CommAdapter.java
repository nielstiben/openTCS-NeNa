package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter;

import com.google.inject.assistedinject.Assisted;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.constants.NodeRunningStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.ScaleCorrector;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.OperationAllowedLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.transport_order.ExecuteTransportOrderWorkflow;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.BasicVehicleCommAdapter;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.ExplainedBoolean;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;
import javax.inject.Inject;
import java.beans.PropertyChangeEvent;
import java.util.List;

import static org.opentcs.drivers.vehicle.VehicleProcessModel.Attribute.LOAD_HANDLING_DEVICES;

/**
 * Class containing override methods that every openTCS vehicle adapter is obliged to implement
 * These override methods are called by components in the Kernel.
 *
 * @author Niels Tiben
 */
public class Ros2CommAdapter extends BasicVehicleCommAdapter {
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapter.class);
    private final Ros2CommAdapterConfiguration configuration;

    // Variable for ExecuteTransportOrderWorkflow: only initiated when driver is enabled.
    private ExecuteTransportOrderWorkflow executeTransportOrderWorkflow;

    //================================================================================
    // Constructor
    //================================================================================

    @Inject
    public Ros2CommAdapter(@Nonnull Ros2CommAdapterConfiguration configuration, @Nonnull @Assisted Vehicle vehicle) {
        super(new Ros2ProcessModel(vehicle),
                2,
                1,
                "CHARGE");
        this.configuration = configuration;
    }

    //================================================================================
    // Override methods: Initiate, enable, disable, terminate.
    //================================================================================

    @Override
    public void initialize() {
        if (isInitialized()) {
            return; // Already initialised.
        }
        super.initialize();

        ScaleCorrector.getInstance().setScale(configuration.plantModelScale()); // Activate scalar
        getProcessModel().setVehicleState(Vehicle.State.UNAVAILABLE); // The robot cannot pickup any task until it has reached a familiar position (plant model point).
    }

    @Override
    public synchronized void enable() {
        if (isEnabled()) {
            return;
        }
        Ros2ProcessModel processModel = getProcessModel();
        processModel.onDriverEnable();
        this.executeTransportOrderWorkflow = new ExecuteTransportOrderWorkflow(processModel, getSentQueue());
        this.executeTransportOrderWorkflow.enableNavigationGoalListener();
        processModel.setExecuteOperationWorkflow(this.executeTransportOrderWorkflow.getExecuteOperationWorkflow());

        super.enable();
    }

    @Override
    public synchronized void disable() {
        if (!isEnabled()) {
            return;
        }

        getProcessModel().onDriverDisable(); // Let the vehicle instance know that we're disabled.
        this.executeTransportOrderWorkflow = null;

        super.disable();
    }

    @Override
    public void terminate() {
        if (!isInitialized()) {
            return;
        }
        super.terminate();
    }

    //================================================================================
    // Callback methods: Incoming commands or messages.
    //================================================================================

    @Override
    public synchronized void sendCommand(@Nonnull MovementCommand cmd) {
        LOG.info("Incoming command: {}", cmd.toString());
        executeTransportOrderWorkflow.processMovementCommand(cmd); // Forward to commandExecutor
    }

    @Override
    public void processMessage(Object message) {
        LOG.info("Incoming message: {}", message.toString());
        LOG.warn("Message handlers are not implemented.");
    }

    @Nonnull
    @Override
    public synchronized ExplainedBoolean canProcess(@Nonnull List<String> operations) {
        LOG.info("{}: Checking if the following operations can be processed: {}.", getName(), operations);
        ExplainedBoolean areAllOperationsAllowed = OperationAllowedLib.areAllOperationsAllowed(operations, this);

        if (areAllOperationsAllowed.getValue()) {
            LOG.info("We can process all operations, let's proceed.");
        } else {
            LOG.info("{}: Cannot process {}, reason: '{}'", getName(), operations, areAllOperationsAllowed.getReason());
        }

        return areAllOperationsAllowed;
    }

    //================================================================================
    // Callback methods: Vehicle related callbacks
    //================================================================================

    @Override
    public void propertyChange(PropertyChangeEvent evt) {
        super.propertyChange(evt);

        if (!((evt.getSource()) instanceof Ros2ProcessModel)) {
            return;
        }

        if (evt.getPropertyName().equals(LOAD_HANDLING_DEVICES.name())) {
            OperationAllowedLib.setLastKnownLoadState(getProcessModel().getVehicleLoadHandlingDevices());
        }
    }

    @Nonnull
    @Override
    public Ros2ProcessModel getProcessModel() {
        return (Ros2ProcessModel) super.getProcessModel();
    }

    @Override
    protected synchronized void connectVehicle() {
        // Not needed. Connection is automatically establish when enabling the driver.
    }

    @Override
    protected synchronized void disconnectVehicle() {
        // Not needed. Connection is automatically terminated when disabling the driver.
    }

    @Override
    protected synchronized boolean isVehicleConnected() {
        return getProcessModel().getNodeManager().getNodeRunningStatus().equals(NodeRunningStatus.ACTIVE);
    }

    @Override
    protected VehicleProcessModelTO createCustomTransferableProcessModel() {
        Ros2ProcessModelTO ros2ProcessModelTO = new Ros2ProcessModelTO();
        ros2ProcessModelTO.setNodeStatus(getProcessModel().getNodeManager().getNodeRunningStatus().name());
        ros2ProcessModelTO.setNavigationGoalTable(getProcessModel().getNavigationGoalTracker().toStringTable());
        ros2ProcessModelTO.setEstimatePosition(getProcessModel().getEstimatedPosition());

        return ros2ProcessModelTO;
    }
}
