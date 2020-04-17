package nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands;

import lombok.AllArgsConstructor;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

import javax.annotation.Nonnull;

@AllArgsConstructor
public class SetInitialPositionCommand implements AdapterCommand {
    private final double x;
    private final double y;

    @Override
    public void execute(@Nonnull VehicleCommAdapter adapter) {
        Ros2ProcessModel ros2ProcessModel = (Ros2ProcessModel) adapter.getProcessModel();
        ros2ProcessModel.setInitialPosition(this.x, this.y);
    }
}
