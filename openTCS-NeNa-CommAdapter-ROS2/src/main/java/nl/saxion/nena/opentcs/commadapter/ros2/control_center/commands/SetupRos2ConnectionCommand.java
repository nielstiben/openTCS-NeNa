package nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

/**
 * Command to setup ROS2 connection with a given domain id.
 */
public class SetupRos2ConnectionCommand implements AdapterCommand {
    private final int domainId;

    public SetupRos2ConnectionCommand(int domainId) {
        this.domainId = domainId;
    }

    @Override
    public void execute(VehicleCommAdapter adapter) {
        Ros2ProcessModel ros2ProcessModel = (Ros2ProcessModel) adapter.getProcessModel();
        ros2ProcessModel.setupConnection(domainId);
    }
}
