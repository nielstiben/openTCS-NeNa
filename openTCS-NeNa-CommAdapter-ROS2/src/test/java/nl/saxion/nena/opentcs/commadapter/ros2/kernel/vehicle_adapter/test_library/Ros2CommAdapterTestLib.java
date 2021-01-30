package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2CommAdapterConfiguration;
import org.opentcs.data.model.Vehicle;

/**
 * Test library for providing an adapter instance used in tests.
 *
 * @author Niels Tiben
 */
public abstract class Ros2CommAdapterTestLib {
    public final static String DEFAULT_TESTING_NAMESPACE = "test";
    public final static int TIME_NEEDED_FOR_NODE_INITIALISATION = 600; // Maximum time needed for node to come online.

    public static Ros2CommAdapter generateAdapterForTesting() {
        Ros2CommAdapterConfiguration config = new Ros2CommAdapterConfiguration() {
            @Override
            public boolean enable() {
                return true;
            }

            @Override
            public double plantModelScale() {
                return 1;
            }
        };

        Vehicle vehicle = new Vehicle("TestVehicle");

        return new Ros2CommAdapter(config, vehicle, null);
    }
}
