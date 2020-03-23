package nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.Ros2CommAdapter;
import org.opentcs.data.model.Vehicle;

/**
 * A factory for ROS2-vehicle instances specific to the comm adapter.
 *
 * @author Niels Tiben
 */
public interface Ros2AdapterComponentsFactory {
    /**
     * Creates a new Ros2CommAdapter for the given vehicle.
     *
     * @param vehicle The vehicle
     * @return A new Ros2CommAdapter
     */
    Ros2CommAdapter createRos2CommAdapter(Vehicle vehicle);
}
