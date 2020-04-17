package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import com.google.inject.assistedinject.Assisted;
import lombok.Getter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.OperationLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory.Ros2AdapterComponentsFactory;
import org.opentcs.common.LoopbackAdapterConstants;
import org.opentcs.customizations.kernel.KernelExecutor;
import org.opentcs.data.ObjectPropConstants;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.*;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.drivers.vehicle.messages.SetSpeedMultiplier;
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
    private final ExecutorService kernelExecutor;
    private final Vehicle vehicle;

    @Getter
    private LoadState loadState = LoadState.EMPTY;
    /**
     * Whether the loopback adapter is initialized or not.
     */
    private boolean initialized;

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
            Ros2AdapterComponentsFactory componentsFactory,
            Ros2CommAdapterConfiguration configuration,
            @Assisted Vehicle vehicle,
            @KernelExecutor ExecutorService kernelExecutor
    ) {
        super(
                new Ros2ProcessModel(vehicle),
                configuration.commandQueueCapacity(),
                1,
                configuration.rechargeOperation()
        );

        this.vehicle = requireNonNull(vehicle, "vehicle");
        this.configuration = requireNonNull(configuration, "configuration");
        this.kernelExecutor = requireNonNull(kernelExecutor, "kernelExecutor");
    }

    @Override
    public void initialize() {
        if (isInitialized()) {
            return;
        }
        super.initialize();

        String initialPos
                = vehicle.getProperties().get(LoopbackAdapterConstants.PROPKEY_INITIAL_POSITION);
        if (initialPos == null) {
            @SuppressWarnings("deprecation")
            String deprecatedInitialPos
                    = vehicle.getProperties().get(ObjectPropConstants.VEHICLE_INITIAL_POSITION);
            initialPos = deprecatedInitialPos;
        }

        getProcessModel().setVehicleState(Vehicle.State.IDLE);
        initialized = true;
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

    @Override
    public synchronized void enable() {
        if (isEnabled()) {
            return;
        }
        super.enable();
        getProcessModel().startNode();
    }

    @Override
    public synchronized void disable() {
        if (!isEnabled()) {
            return;
        }
        super.disable();
        getProcessModel().getNodeManager().stop();
    }

    @Nonnull
    @Override
    public Ros2ProcessModel getProcessModel() {
        return (Ros2ProcessModel) super.getProcessModel();
    }

    @Override
    public synchronized void sendCommand(MovementCommand cmd) {
        requireNonNull(cmd, "cmd");
        LOG.info("INCOMING COMMAND:");
        LOG.info(cmd.toString());
    }

    @Override
    public void processMessage(Object message) {
        // Process LimitSpeeed message which might pause the vehicle.
        LOG.info("INCOMING MESSAGE:");
        LOG.info(message.toString());
        if (message instanceof SetSpeedMultiplier) {
            SetSpeedMultiplier lsMessage = (SetSpeedMultiplier) message;
            int multiplier = lsMessage.getMultiplier();
//            getProcessModel().setVehiclePaused(multiplier == 0);
        }
    }

    @Nonnull
    @Override
    public synchronized ExplainedBoolean canProcess(@Nonnull List<String> operations) {
        LOG.info("{}: Checking processability of {}...", getName(), operations);
        ExplainedBoolean areAllOperationsAllowed = OperationLib.areAllOperationsAllowed(operations, this);

        if (!areAllOperationsAllowed.getValue()) {
            LOG.info("{}: Cannot process {}, reason: '{}'", getName(), operations, areAllOperationsAllowed.getReason());
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

        ros2ProcessModelTO.setNodeStatus(getProcessModel().getNodeManager().getNodeStatus().name());
        ros2ProcessModelTO.setLoadOperation(getProcessModel().getLoadOperation());
        ros2ProcessModelTO.setOperatingTime(getProcessModel().getOperatingTime());
        ros2ProcessModelTO.setUnloadOperation(getProcessModel().getUnloadOperation());
        ros2ProcessModelTO.setNavigationGoalTable(getProcessModel().getNavigationGoalTracker().generateStringMatrix());

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