package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import action_msgs.msg.GoalStatusArray;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionController;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.NodeListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import nl.saxion.nena.opentcs.commadapter.ros2.virtual_vehicle.Parsers;
import org.opentcs.common.LoopbackAdapterConstants;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleProcessModel;

import javax.annotation.Nonnull;

public class Ros2ProcessModel extends VehicleProcessModel
        implements ConnectionListener, NodeListener {
    ConnectionController connectionController;
    GoalStatusArray goalStatusArray;

    /**
     * Indicates which operation is a loading operation.
     */
    private final String loadOperation;

    /**
     * Indicates which operation is an unloading operation.
     */
    private final String unloadOperation;
    /**
     * The time needed for executing operations.
     */
    private int operatingTime;

    public Ros2ProcessModel(Vehicle attachedVehicle) {
        super(attachedVehicle);
        this.connectionController = new ConnectionController(this, this);
        this.operatingTime = parseOperatingTime(attachedVehicle);
        this.loadOperation = extractLoadOperation(attachedVehicle);
        this.unloadOperation = extractUnloadOperation(attachedVehicle);
    }

    @Nonnull
    public ConnectionController getConnectionController() {
        return connectionController;
    }

    public String getLoadOperation() {
        return loadOperation;
    }

    public String getUnloadOperation() {
        return unloadOperation;
    }

    public synchronized void setupConnection(int domainId) {
        connectionController.connect(domainId);
    }

    /**
     * Returns the default operating time.
     *
     * @return The default operating time
     */
    public synchronized int getOperatingTime() {
        //TODO: Customize

        return operatingTime;
    }

    /**
     * Sets the default operating time.
     *        goalStatusArray.get

     * @param defaultOperatingTime The new default operating time
     */
    public synchronized void setOperatingTime(int defaultOperatingTime) {
        //TODO: Customize

        int oldValue = this.operatingTime;
        this.operatingTime = defaultOperatingTime;

        getPropertyChangeSupport().firePropertyChange(Attribute.OPERATING_TIME.name(),
                oldValue,
                defaultOperatingTime);
    }

    private int parseOperatingTime(Vehicle vehicle) {
        //TODO: Customize
        String opTime = vehicle.getProperty(LoopbackAdapterConstants.PROPKEY_OPERATING_TIME);
        // Ensure it's a positive value.
        return Math.max(Parsers.tryParseString(opTime, 5000), 1);
    }

    private static String extractLoadOperation(Vehicle attachedVehicle) {
        return OperationConstants.LOAD_CARGO;
    }

    private static String extractUnloadOperation(Vehicle attachedVehicle) {
        return OperationConstants.UNLOAD_CARGO;
    }

    @Override
    public void onConnectionStatusChange(ConnectionStatus connectionStatus) {
        getPropertyChangeSupport().firePropertyChange(Attribute.CONNECTION_STATUS.name(), null, connectionStatus);
    }

    @Override
    public void onNewGoalStatusArray(GoalStatusArray goalStatusArray) {
        this.goalStatusArray = goalStatusArray;
    }

    /**
     * Notification arguments to indicate some change.
     */
    public enum Attribute {
        OPERATING_TIME,
        CONNECTION_STATUS,
    }
}
