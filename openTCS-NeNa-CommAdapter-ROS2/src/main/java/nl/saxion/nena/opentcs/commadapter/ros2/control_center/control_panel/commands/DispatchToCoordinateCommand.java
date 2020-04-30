package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.commands;

import lombok.AllArgsConstructor;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2ProcessModel;
import org.opentcs.data.model.Triple;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

import javax.annotation.Nonnull;

/**
 * Instruct the kernel to dispatch the vehicle to a {@link Triple} coordinate.
 *
 * @author Niels Tiben
 */
@AllArgsConstructor
public class DispatchToCoordinateCommand implements AdapterCommand {
    private final Triple destinationCoordinate;

    @Override
    public void execute(@Nonnull VehicleCommAdapter adapter) {
        Ros2ProcessModel ros2ProcessModel = (Ros2ProcessModel) adapter.getProcessModel();
        ros2ProcessModel.dispatchToCoordinate(this.destinationCoordinate);
    }
}
