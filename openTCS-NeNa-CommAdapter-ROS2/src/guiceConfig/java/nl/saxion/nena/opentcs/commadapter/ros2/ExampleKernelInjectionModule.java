/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import com.google.inject.assistedinject.FactoryModuleBuilder;
import org.opentcs.customizations.kernel.KernelInjectionModule;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ExampleKernelInjectionModule
    extends KernelInjectionModule {
  
  private static final Logger LOG = LoggerFactory.getLogger(ExampleKernelInjectionModule.class);

  @Override
  protected void configure() {
    
    ExampleCommAdapterConfiguration configuration
        = getConfigBindingProvider().get(ExampleCommAdapterConfiguration.PREFIX,
                                         ExampleCommAdapterConfiguration.class);
    
    if (!configuration.enable()) {
      LOG.info("Example communication adapter disabled by configuration.");
      return;
    }
    
    install(new FactoryModuleBuilder().build(ExampleAdapterComponentsFactory.class));
    vehicleCommAdaptersBinder().addBinding().to(ExampleCommAdapterFactory.class);
  }
}
