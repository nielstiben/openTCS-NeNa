package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import lombok.Getter;
import lombok.Setter;
import org.opentcs.data.model.Triple;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;

import static java.util.Objects.requireNonNull;

/**
 * A serializable representation of a {@link Ros2ProcessModel}.
 */
@Getter
@Setter
public class Ros2ProcessModelTO extends VehicleProcessModelTO {
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
     * Whether the ROS2 node is active or not.
     */
    private String nodeStatus;

    private int operatingTime;

    private Triple estimatePosition;
    /**
     * List of navigation goals
     */
    private String[][] navigationGoalTable;


}
