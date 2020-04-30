package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.commands;

import lombok.AllArgsConstructor;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.Node;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

import javax.annotation.Nonnull;

/**
 * Instruct the kernel to use the given namespace when a {@link Node} is initiated.
 *
 * @author Niels Tiben
 */
@AllArgsConstructor
public class SetNamespaceCommand implements AdapterCommand {
    private final String namespace;

    @Override
    public void execute(@Nonnull VehicleCommAdapter adapter) {
        Ros2ProcessModel ros2ProcessModel = (Ros2ProcessModel) adapter.getProcessModel();
        ros2ProcessModel.setNamespace(this.namespace);
    }
}
