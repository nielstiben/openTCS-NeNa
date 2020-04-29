package nl.saxion.nena.opentcs.commadapter.ros2.unit.transport_order.test_library;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapterConfiguration;
import org.opentcs.data.model.Vehicle;

/**
 * Test library for providing an adapter instance used in tests.
 *
 * @author Niels Tiben
 */
public class Ros2CommAdapterTestLib {
    public static Ros2CommAdapter generateAdapterForTesting() {
        Ros2CommAdapterConfiguration config = new Ros2CommAdapterConfiguration() {
            @Override
            public boolean enable() {
                return false;
            }

            @Override
            public double plantModelScale() {
                return 1;
            }

            @Override
            public int commandQueueCapacity() {
                return 1;
            }

            @Override
            public String rechargeOperation() {
                return "TEST_RECHARGE";
            }
        };

        Vehicle vehicle = new Vehicle("TestVehicle");

        return new Ros2CommAdapter(config, vehicle);
    }
}
