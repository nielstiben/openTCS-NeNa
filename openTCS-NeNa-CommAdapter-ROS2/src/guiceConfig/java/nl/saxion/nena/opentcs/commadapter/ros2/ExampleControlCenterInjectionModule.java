/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import com.google.inject.assistedinject.FactoryModuleBuilder;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.AdapterPanelComponentsFactory;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.ExampleCommAdapterPanelFactory;
import org.opentcs.customizations.controlcenter.ControlCenterInjectionModule;

/**
 * A custom Guice module for project-specific configuration.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class ExampleControlCenterInjectionModule
    extends ControlCenterInjectionModule {

  @Override
  protected void configure() {
    install(new FactoryModuleBuilder().build(AdapterPanelComponentsFactory.class));

    commAdapterPanelFactoryBinder().addBinding().to(ExampleCommAdapterPanelFactory.class);
  }
}
