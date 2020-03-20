package nl.saxion.nena.opentcs.commadapter.ros2;

import com.google.inject.assistedinject.FactoryModuleBuilder;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.Ros2AdapterComponentsFactory;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.Ros2CommAdapterConfiguration;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.Ros2CommAdapterFactory;
import org.opentcs.customizations.kernel.KernelInjectionModule;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Ros2KernelInjectionModule extends KernelInjectionModule {

    private static final Logger LOG = LoggerFactory.getLogger(Ros2KernelInjectionModule.class);

    @Override
    protected void configure() {
        Ros2CommAdapterConfiguration configuration = getConfigBindingProvider()
                .get(Ros2CommAdapterConfiguration.PREFIX, Ros2CommAdapterConfiguration.class);

        if (!configuration.enable()) {
            LOG.info("ROS2 communication adapter disabled by configuration.");
            return;
        }

        LOG.info("ROS2 communication adapter ENABLED");

        bind(Ros2CommAdapterConfiguration.class).toInstance(configuration); // Similar to LoopBackAdapter.


    install(new FactoryModuleBuilder().build(Ros2AdapterComponentsFactory.class));
    vehicleCommAdaptersBinder().addBinding().to(Ros2CommAdapterFactory.class);
    }
}
