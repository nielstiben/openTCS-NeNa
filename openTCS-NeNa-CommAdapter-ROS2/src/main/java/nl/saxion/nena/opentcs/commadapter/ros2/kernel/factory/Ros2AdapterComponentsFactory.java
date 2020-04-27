package nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import org.opentcs.data.model.Vehicle;

/**
 * A factory for ROS2-vehicle instances specific to the comm adapter.
 *
 * @author Niels Tiben
 */
public interface Ros2AdapterComponentsFactory {
    Ros2CommAdapter createRos2CommAdapter(Vehicle vehicle);
}
