package nl.saxion.nena.opentcs.commadapter.ros2.virtual_vehicle;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.Ros2ProcessModel;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.notification.UserNotification;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.io.Serializable;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import static java.util.Objects.requireNonNull;

/**
 * A serializable representation of a {@link Ros2ProcessModel}.
 */
public class Ros2ProcessModelTO extends VehicleProcessModelTO {

    /**
     * Whether this communication adapter is in single step mode or not (i.e. in automatic mode).
     */
    private boolean singleStepModeEnabled;
    /**
     * Indicates which operation is a loading operation.
     */
    private String loadOperation;
    /**
     * Indicates which operation is an unloading operation.
     */
    private String unloadOperation;
    /**
     * The time needed for executing operations.
     */
    private int operatingTime;
    /**
     * The maximum acceleration.
     */
    private int maxAcceleration;
    /**
     * The maximum deceleration.
     */
    private int maxDeceleration;
    /**
     * The maximum forward velocity.
     */
    private int maxFwdVelocity;
    /**
     * The maximum reverse velocity.
     */
    private int maxRevVelocity;
    /**
     * Whether the vehicle is paused or not.
     */
    private boolean vehiclePaused;

    public boolean isSingleStepModeEnabled() {
        return singleStepModeEnabled;
    }

    public Ros2ProcessModelTO setSingleStepModeEnabled(boolean singleStepModeEnabled) {
        this.singleStepModeEnabled = singleStepModeEnabled;
        return this;
    }

    public String getLoadOperation() {
        return loadOperation;
    }

    public Ros2ProcessModelTO setLoadOperation(String loadOperation) {
        this.loadOperation = loadOperation;
        return this;
    }

    public String getUnloadOperation() {
        return unloadOperation;
    }

    public Ros2ProcessModelTO setUnloadOperation(String unloadOperation) {
        this.unloadOperation = unloadOperation;
        return this;
    }

    public int getOperatingTime() {
        return operatingTime;
    }

    public Ros2ProcessModelTO setOperatingTime(int operatingTime) {
        this.operatingTime = operatingTime;
        return this;
    }

    public int getMaxAcceleration() {
        return maxAcceleration;
    }

    public Ros2ProcessModelTO setMaxAcceleration(int maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        return this;
    }

    public int getMaxDeceleration() {
        return maxDeceleration;
    }

    public Ros2ProcessModelTO setMaxDeceleration(int maxDeceleration) {
        this.maxDeceleration = maxDeceleration;
        return this;
    }

    public int getMaxFwdVelocity() {
        return maxFwdVelocity;
    }

    public Ros2ProcessModelTO setMaxFwdVelocity(int maxFwdVelocity) {
        this.maxFwdVelocity = maxFwdVelocity;
        return this;
    }

    public int getMaxRevVelocity() {
        return maxRevVelocity;
    }

    public Ros2ProcessModelTO setMaxRevVelocity(int maxRevVelocity) {
        this.maxRevVelocity = maxRevVelocity;
        return this;
    }

    public boolean isVehiclePaused() {
        return vehiclePaused;
    }

    public Ros2ProcessModelTO setVehiclePaused(boolean vehiclePaused) {
        this.vehiclePaused = vehiclePaused;
        return this;
    }
}
