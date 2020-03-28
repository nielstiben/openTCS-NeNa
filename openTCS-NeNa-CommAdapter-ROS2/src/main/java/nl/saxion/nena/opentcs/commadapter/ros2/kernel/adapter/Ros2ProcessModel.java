package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionController;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import nl.saxion.nena.opentcs.commadapter.ros2.virtual_vehicle.Parsers;
import nl.saxion.nena.opentcs.commadapter.ros2.virtual_vehicle.VelocityHistory;
import org.opentcs.common.LoopbackAdapterConstants;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleProcessModel;

import javax.annotation.Nonnull;

/*
Single instance should be provided by every VehicleCommAdapter instance in which it keeps the relevant state of both the
evhicle and the comm adapter. This model instance is supposed to be updated to notify the kernel about relevant changes.
The comm adapter implementation should e.g. update the vehicle’s current position in the model when it receives that
information to allow the kernel and GUI frontends to use it. Likewise, other components may set values that influence
the comm adapter’s behaviour in the model, e.g. a time interval for periodic messages the comm adapter sends to the
vehicle. VehicleProcessModel may be used as it is, as it contains members for all the information the openTCS kernel
itself needs. However, developers may use driver-specific subclasses of VehicleProcessModel to have the comm adapter
and other components exchange more than the default set of attributes.
 */
public class Ros2ProcessModel extends VehicleProcessModel
        implements ConnectionListener {
    ConnectionController connectionController;
    /**
     * Indicates whether this communication adapter is in single step mode or not (i.e. in automatic
     * mode).
     */
    private boolean singleStepModeEnabled;
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

    /**
     * Keeps a log of recent velocity values.
     */
    private final VelocityHistory velocityHistory = new VelocityHistory(100, 10);

    public Ros2ProcessModel(Vehicle attachedVehicle) {
        //TODO: Customize

        super(attachedVehicle);
        this.connectionController = new ConnectionController(this);
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
     * Sets this communication adapter's <em>single step mode</em> flag.
     *
     * @param mode If <code>true</code>, sets this adapter to single step mode,
     *             otherwise sets this adapter to flow mode.
     */
    public synchronized void setSingleStepModeEnabled(final boolean mode) {
        //TODO: Customize

        boolean oldValue = singleStepModeEnabled;
        singleStepModeEnabled = mode;

        getPropertyChangeSupport().firePropertyChange(Attribute.SINGLE_STEP_MODE.name(),
                oldValue,
                mode);
    }

    /**
     * Returns this communication adapter's <em>single step mode</em> flag.
     *
     * @return <code>true</code> if, and only if, this adapter is currently in
     * single step mode.
     */
    public synchronized boolean isSingleStepModeEnabled() {
        //TODO: Customize

        return singleStepModeEnabled;
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
     *
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


    /**
     * Returns a log of recent velocity values of the vehicle.
     *
     * @return A log of recent velocity values.
     */
    @Nonnull
    public VelocityHistory getVelocityHistory() {
        //TODO: Customize
        return velocityHistory;
    }

    private int parseOperatingTime(Vehicle vehicle) {
        //TODO: Customize
        String opTime = vehicle.getProperty(LoopbackAdapterConstants.PROPKEY_OPERATING_TIME);
        // Ensure it's a positive value.
        return Math.max(Parsers.tryParseString(opTime, 5000), 1);
    }

    /**
     * Gets the maximum acceleration. If the user did not specify any, 1000(m/s²) is returned.
     *
     * @param vehicle the vehicle
     * @return the maximum acceleration.
     */
    private int parseAcceleration(Vehicle vehicle) {
        //TODO: Customize
        String acceleration = vehicle.getProperty(LoopbackAdapterConstants.PROPKEY_ACCELERATION);
        // Ensure it's a positive value.
        return Math.max(Parsers.tryParseString(acceleration, 500), 1);
    }

    /**
     * Gets the maximum decceleration. If the user did not specify any, 1000(m/s²) is returned.
     *
     * @param vehicle the vehicle
     * @return the maximum decceleration.
     */
    private int parseDeceleration(Vehicle vehicle) {
        //TODO: Customize
        String deceleration = vehicle.getProperty(LoopbackAdapterConstants.PROPKEY_DECELERATION);
        // Ensure it's a negative value.
        return Math.min(Parsers.tryParseString(deceleration, -500), -1);
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

    /**
     * Notification arguments to indicate some change.
     */
    public enum Attribute {
        /**
         * Indicates a change of the virtual vehicle's single step mode setting.
         */
        SINGLE_STEP_MODE,
        /**
         * Indicates a change of the virtual vehicle's default operating time.
         */
        OPERATING_TIME,
        /**
         * Indicates a change of the virtual vehicle's maximum acceleration.
         */
        ACCELERATION,
        /**
         * Indicates a change of the virtual vehicle's maximum deceleration.
         */
        DECELERATION,
        /**
         * Indicates a change of the virtual vehicle's maximum forward velocity.
         */
        MAX_FORWARD_VELOCITY,
        /**
         * Indicates a change of the virtual vehicle's maximum reverse velocity.
         */
        MAX_REVERSE_VELOCITY,
        /**
         * Indicates a change of the virtual vehicle's paused setting.
         */
        VEHICLE_PAUSED,
        /**
         * Indicates a change of the virtual vehicle's velocity history.
         */
        VELOCITY_HISTORY,
        CONNECTION_STATUS,
    }
}
