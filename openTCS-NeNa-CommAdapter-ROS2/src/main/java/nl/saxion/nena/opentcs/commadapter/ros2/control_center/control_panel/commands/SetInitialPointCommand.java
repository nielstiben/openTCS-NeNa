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
 * Instruct the kernel to send an initial pose message to the vehicle. In RViz, this is also called “2D Pose Estimate”.
 *
 * @author Niels Tiben
 */
@AllArgsConstructor
public class SetInitialPointCommand implements AdapterCommand {
    private final Point initialPoint;

    @Override
    public void execute(@Nonnull VehicleCommAdapter adapter) {
        Ros2ProcessModel ros2ProcessModel = (Ros2ProcessModel) adapter.getProcessModel();
        ros2ProcessModel.setInitialPoint(this.initialPoint);
    }
}
