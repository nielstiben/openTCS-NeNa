package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import com.google.inject.assistedinject.Assisted;
import lombok.Getter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.ScaleCorrector;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.OperationAllowedLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.task.ExecuteCommandWorkflow;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory.Ros2AdapterComponentsFactory;
import org.opentcs.common.LoopbackAdapterConstants;
import org.opentcs.customizations.kernel.KernelExecutor;
import org.opentcs.data.ObjectPropConstants;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.*;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.ExplainedBoolean;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;
import javax.inject.Inject;
import java.beans.PropertyChangeEvent;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ExecutorService;

import static java.util.Objects.requireNonNull;


public class Ros2CommAdapter extends BasicVehicleCommAdapter {
    public static final String LHD_NAME = "ros2";
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapter.class);
    private final Ros2CommAdapterConfiguration configuration;

    private final Vehicle vehicle;

    @Getter
    private LoadState loadState = LoadState.EMPTY;
    /**
     * Whether the loopback adapter is initialized or not.
     */
    private boolean initialized;
    private ExecuteCommandWorkflow executeCommandWorkflow; // Only initiated when driver is enabled.


    /**
     * Creates a new instance.
     *
     * @param componentsFactory The factory providing additional components for this adapter.
     * @param configuration     This class's configuration.
     * @param vehicle           The vehicle this adapter is associated with.
     * @param kernelExecutor    The kernel's executor.
     */
    @Inject
    public Ros2CommAdapter(
            @Nonnull Ros2CommAdapterConfiguration configuration,
            @Nonnull @Assisted Vehicle vehicle
    ) {
        super(
                new Ros2ProcessModel(vehicle),
                configuration.commandQueueCapacity(),
                1,
                configuration.rechargeOperation()
        );
        this.vehicle = vehicle;
        this.configuration = configuration;
    }

    @Override
    public void initialize() {
        if (isInitialized()) {
            return;
        }
        super.initialize();

        String initialPos = vehicle.getProperties().get(LoopbackAdapterConstants.PROPKEY_INITIAL_POSITION);
        if (initialPos == null) {
            @SuppressWarnings("deprecation")
            String deprecatedInitialPos = vehicle.getProperties().get(ObjectPropConstants.VEHICLE_INITIAL_POSITION);
            initialPos = deprecatedInitialPos;
        }

        // Set scale
        ScaleCorrector.getInstance().setScale(configuration.plantModelScale());

        getProcessModel().setVehicleState(Vehicle.State.IDLE);
        this.initialized = true;
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }

    @Override
    public void terminate() {
        if (!isInitialized()) {
            return;
        }
        super.terminate();
        initialized = false;
    }

    @Override
    public void propertyChange(PropertyChangeEvent evt) {
        super.propertyChange(evt);

        if (!((evt.getSource()) instanceof Ros2ProcessModel)) {
            return;
        }
        if (Objects.equals(evt.getPropertyName(),
                VehicleProcessModel.Attribute.LOAD_HANDLING_DEVICES.name())) {
            if (!getProcessModel().getVehicleLoadHandlingDevices().isEmpty()
                    && getProcessModel().getVehicleLoadHandlingDevices().get(0).isFull()) {
                loadState = LoadState.FULL;
            } else {
                loadState = LoadState.EMPTY;
            }
        }
    }

    /* --------------- Enable / Disable ---------------*/

    @Override
    public synchronized void enable() {
        if (isEnabled()) {
            return;
        }
        Ros2ProcessModel processModel = getProcessModel();
        processModel.onDriverEnable();
        this.executeCommandWorkflow = new ExecuteCommandWorkflow(processModel, getSentQueue());
        this.executeCommandWorkflow.enableNavigationGoalListener();
        processModel.setExecuteOperationWorkflow(this.executeCommandWorkflow.getExecuteOperationWorkflow());

        super.enable();
    }

    @Override
    public synchronized void disable() {
        if (!isEnabled()) {
            return;
        }

        getProcessModel().onDriverDisable(); // Vehicle instance
        this.executeCommandWorkflow = null; // Cyclic task for picking up tasks

        super.disable();
    }

    /* --------------- Misc ---------------*/

    @Nonnull
    @Override
    public Ros2ProcessModel getProcessModel() {
        return (Ros2ProcessModel) super.getProcessModel();
    }

    @Override
    public synchronized void sendCommand(@Nonnull MovementCommand cmd) {
        LOG.info("INCOMING COMMAND:");
        LOG.info(cmd.toString());

        LOG.info("1 Sent queue: " + getSentQueue().toString());
        LOG.info("1 Comm queue: " + getCommandQueue().toString());

        // Forward to commandExecutor
        executeCommandWorkflow.processMovementCommand(cmd);
    }

    @Override
    public void processMessage(Object message) {
        // Process LimitSpeeed message which might pause the vehicle.
        LOG.info("INCOMING MESSAGE:");
        LOG.info(message.toString());

//        if (message instanceof SetSpeedMultiplier) {
//            SetSpeedMultiplier lsMessage = (SetSpeedMultiplier) message;
//            int multiplier = lsMessage.getMultiplier();
//            getProcessModel().setVehiclePaused(multiplier == 0);
//        }
    }

    @Nonnull
    @Override
    public synchronized ExplainedBoolean canProcess(@Nonnull List<String> operations) {
        LOG.info("{}: Checking processability of {}...", getName(), operations);
        ExplainedBoolean areAllOperationsAllowed = OperationAllowedLib.areAllOperationsAllowed(operations, this);

        if (!areAllOperationsAllowed.getValue()) {
            LOG.info("{}: Cannot process {}, reason: '{}'", getName(), operations, areAllOperationsAllowed.getReason());
        } else {
            LOG.info("We can process all operations, let's proceed.");
        }

        return areAllOperationsAllowed;
    }

    @Override
    protected synchronized boolean canSendNextCommand() {
        return super.canSendNextCommand();
    }

    @Override
    protected synchronized void connectVehicle() {
    }

    @Override
    protected synchronized void disconnectVehicle() {
    }

    @Override
    protected synchronized boolean isVehicleConnected() {
        return true;
    }

    @Override
    protected VehicleProcessModelTO createCustomTransferableProcessModel() {
        Ros2ProcessModelTO ros2ProcessModelTO = new Ros2ProcessModelTO();
        ros2ProcessModelTO.setNodeStatus(getProcessModel().getNodeManager().getNodeRunningStatus().name());
        ros2ProcessModelTO.setNavigationGoalTable(getProcessModel().parseNavigationGoalTable());
        ros2ProcessModelTO.setEstimatePosition(getProcessModel().getEstimatePosition());

        return ros2ProcessModelTO;
    }

    /**
     * The vehicle's possible load states.
     */
    public enum LoadState {
        EMPTY,
        FULL;
    }
}