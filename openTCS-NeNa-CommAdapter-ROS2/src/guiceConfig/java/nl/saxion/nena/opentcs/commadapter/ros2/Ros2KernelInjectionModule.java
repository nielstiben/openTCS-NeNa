/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import com.google.inject.assistedinject.FactoryModuleBuilder;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory.Ros2AdapterComponentsFactory;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2CommAdapterConfiguration;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory.Ros2CommAdapterFactory;
import org.opentcs.customizations.kernel.KernelInjectionModule;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * A Guice module for ROS2 Driver - Control Center configuration.
 *
 * @author Niels Tiben
 */
public class Ros2KernelInjectionModule extends KernelInjectionModule {
    private static final Logger LOG = LoggerFactory.getLogger(Ros2KernelInjectionModule.class);

    @Override
    protected void configure() {
        Ros2CommAdapterConfiguration configuration = getConfigBindingProvider()
                .get(Ros2CommAdapterConfiguration.PREFIX, Ros2CommAdapterConfiguration.class);

        if (!configuration.enable()) {
            LOG.info("NeNa ROS2 communication adapter disabled by configuration.");
            return;
        }

        LOG.info("NeNa ROS2 communication adapter ENABLED");
        bind(Ros2CommAdapterConfiguration.class).toInstance(configuration);

        install(new FactoryModuleBuilder().build(Ros2AdapterComponentsFactory.class));
        vehicleCommAdaptersBinder().addBinding().to(Ros2CommAdapterFactory.class);
    }
}
