/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import com.google.inject.assistedinject.FactoryModuleBuilder;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.factory.AdapterPanelComponentsFactory;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.factory.Ros2CommAdapterPanelFactory;
import org.opentcs.customizations.controlcenter.ControlCenterInjectionModule;

/**
 * A custom Guice module for project-specific configuration.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class Ros2ControlCenterInjectionModule
    extends ControlCenterInjectionModule {

  @Override
  protected void configure() {
    install(new FactoryModuleBuilder().build(AdapterPanelComponentsFactory.class));

    commAdapterPanelFactoryBinder().addBinding().to(Ros2CommAdapterPanelFactory.class);
  }
}
