/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter;

import lombok.Getter;
import lombok.NoArgsConstructor;
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
@NoArgsConstructor
public class Ros2ProcessModelTO extends VehicleProcessModelTO {
    private String nodeStatus;
    private Triple estimatePosition;
    private String[][] navigationGoalTable;
}
