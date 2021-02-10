/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory;

import nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.Ros2CommAdapterDescription;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2CommAdapter;
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

/**
 * Factory for providing {@link Ros2CommAdapter} instances.
 * The factory provides instances on demand, depending on the number of ROS2 vehicles in the model.
 *
 * @author Niels Tiben
 */
public class Ros2CommAdapterFactory implements VehicleCommAdapterFactory {
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapterFactory.class);
    private final Ros2AdapterComponentsFactory componentsFactory;

    // Variable for whether the adapter is initialised.
    private boolean initialised;

    //================================================================================
    // Constructor
    //================================================================================
    @Inject
    public Ros2CommAdapterFactory(Ros2AdapterComponentsFactory adapterFactory) {
        this.componentsFactory = requireNonNull(adapterFactory, "componentsFactory");
    }

    //================================================================================
    // Override methods
    //================================================================================

    @Override
    public boolean providesAdapterFor(@Nonnull Vehicle vehicle) {
        // Any vehicle in an openTCS model plant may be a ROS2 vehicle.
        return true;
    }

    @Nullable
    @Override
    public VehicleCommAdapter getAdapterFor(@Nonnull Vehicle vehicle) {
        if (!providesAdapterFor(vehicle)) {
            return null; // Adapter not allowed
        }

        return componentsFactory.createRos2CommAdapter(vehicle);
    }

    @Override
    public void initialize() {
        if (isInitialized()) {
            LOG.warn("The adapter is already initialized.");
            return;
        }
        initialised = true;
    }

    @Override
    public boolean isInitialized() {
        return initialised;
    }

    @Override
    public void terminate() {
        if (!isInitialized()) {
            LOG.warn("The adapter is not initialized, so it is already terminated.");
            return;
        }
        initialised = false;
    }

    @Override
    public VehicleCommAdapterDescription getDescription() {
        return new Ros2CommAdapterDescription();
    }
}
