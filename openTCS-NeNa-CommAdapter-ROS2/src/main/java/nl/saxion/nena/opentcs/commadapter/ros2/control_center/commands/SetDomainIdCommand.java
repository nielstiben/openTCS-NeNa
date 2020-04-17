package nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands;

import lombok.AllArgsConstructor;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

/**
 * Command to setup ROS2 connection with a given domain id.
 */
@AllArgsConstructor
public class SetDomainIdCommand implements AdapterCommand {
    private final int domainId;

    @Override
    public void execute(VehicleCommAdapter adapter) {
        Ros2ProcessModel ros2ProcessModel = (Ros2ProcessModel) adapter.getProcessModel();
        ros2ProcessModel.setDomainId(domainId);
    }
}
