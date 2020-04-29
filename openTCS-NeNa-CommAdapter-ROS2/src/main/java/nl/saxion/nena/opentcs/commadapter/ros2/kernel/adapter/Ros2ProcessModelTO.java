package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import lombok.Getter;
import lombok.Setter;
import org.opentcs.data.model.Triple;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;

/**
 * A serializable representation of a {@link Ros2ProcessModel}.
 *
 * @author Niels Tiben
 */
@Getter
@Setter
public class Ros2ProcessModelTO extends VehicleProcessModelTO {
    private String nodeStatus;
    private Triple estimatePosition;
    private String[][] navigationGoalTable;
}
