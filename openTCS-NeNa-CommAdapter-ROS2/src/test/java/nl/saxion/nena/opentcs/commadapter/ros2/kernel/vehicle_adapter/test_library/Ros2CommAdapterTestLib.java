package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2CommAdapterConfiguration;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.ScaleCorrector;
import org.opentcs.data.model.Vehicle;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Test library for providing an adapter instance used in tests.
 *
 * @author Niels Tiben
 */
public abstract class Ros2CommAdapterTestLib {
    public final static String DEFAULT_TESTING_NAMESPACE = "/test";
    public final static int DEFAULT_TESTING_DOMAIN_ID = 30;
    public final static int TIME_NEEDED_FOR_NODE_INITIALISATION = 2100; // Maximum time needed for node to come online.

    public static Ros2CommAdapter generateAdapterForTesting() {
        ScaleCorrector.getInstance().setScale(1);
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

        Ros2CommAdapter adapter = new Ros2CommAdapter(config, vehicle, Executors.newSingleThreadExecutor());
        adapter.getProcessModel().setNamespace(DEFAULT_TESTING_NAMESPACE);
        adapter.getProcessModel().setDomainId(DEFAULT_TESTING_DOMAIN_ID);

        return adapter;
    }
}
