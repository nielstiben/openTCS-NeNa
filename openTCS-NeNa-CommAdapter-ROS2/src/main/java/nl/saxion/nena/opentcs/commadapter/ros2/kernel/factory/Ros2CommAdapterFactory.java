package nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory;

import nl.saxion.nena.opentcs.commadapter.ros2.control_center.Ros2CommAdapterDescription;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.Ros2CommAdapter;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;
import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;
import org.opentcs.drivers.vehicle.VehicleCommAdapterFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import javax.inject.Inject;

import static java.util.Objects.requireNonNull;

/*
describes a factory for VehicleCommAdapter instances.
The kernel instantiates and uses one such factory per
vehicle driver to create instances of the respective
Ros2CommAdapter implementation on demand.
 */
public class Ros2CommAdapterFactory implements VehicleCommAdapterFactory {
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapterFactory.class);

    private final Ros2AdapterComponentsFactory componentsFactory;
    private boolean initialized;

    @Inject
    public Ros2CommAdapterFactory(Ros2AdapterComponentsFactory adapterFactory) {
        this.componentsFactory = requireNonNull(adapterFactory, "componentsFactory");
    }

    @Override
    public boolean providesAdapterFor(@Nonnull Vehicle vehicle) {
        // TODO: Perform vehicle checks
        return true;
    }

    @Nullable
    @Override
    public VehicleCommAdapter getAdapterFor(@Nonnull Vehicle vehicle) {
        if(!providesAdapterFor(vehicle)){
            return null; // Adapter not allowed
        }
        Ros2CommAdapter adapter = componentsFactory.createRos2CommAdapter(vehicle);
        // TODO: Set port, connection etcetera

        return adapter;
    }

    @Override
    public void initialize() {
        if (isInitialized()) {
            LOG.warn("The adapter is already initialized.");
            return;
        }
        initialized = true;
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }

    @Override
    public void terminate() {
        if (!isInitialized()) {
            LOG.warn("The adapter is not initialized, so it is already terminated.");
            return;
        }
        initialized = false;
    }

    @Override
    public VehicleCommAdapterDescription getDescription() {
        return new Ros2CommAdapterDescription();
    }
}
