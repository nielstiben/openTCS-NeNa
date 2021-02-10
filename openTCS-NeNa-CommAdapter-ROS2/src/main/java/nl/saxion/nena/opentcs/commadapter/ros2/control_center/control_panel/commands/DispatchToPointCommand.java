/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.commands;

import lombok.AllArgsConstructor;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2ProcessModel;
import org.opentcs.data.model.Point;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

import javax.annotation.Nonnull;

/**
 * Instruct the kernel to dispatch the vehicle to the location of a {@link Point}.
 *
 * @author Niels Tiben
 */
@AllArgsConstructor
public class DispatchToPointCommand implements AdapterCommand {
    private final Point destinationPoint;

    @Override
    public void execute(@Nonnull VehicleCommAdapter adapter) {
        Ros2ProcessModel ros2ProcessModel = (Ros2ProcessModel) adapter.getProcessModel();
        ros2ProcessModel.dispatchToPoint(this.destinationPoint);
    }
}
