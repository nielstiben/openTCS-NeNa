package nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands;

import lombok.AllArgsConstructor;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

@AllArgsConstructor
public class DispatchToCoordinateCommand implements AdapterCommand {
    private final double x;
    private final double y;

    @Override
    public void execute(VehicleCommAdapter adapter) {
        Ros2ProcessModel ros2ProcessModel = (Ros2ProcessModel) adapter.getProcessModel();
        ros2ProcessModel.dispatchToCoordinate(this.x, this.y);
    }
}
