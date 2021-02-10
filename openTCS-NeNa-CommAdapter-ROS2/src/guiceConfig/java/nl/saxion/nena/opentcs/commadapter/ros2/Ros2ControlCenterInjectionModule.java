/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import com.google.inject.assistedinject.FactoryModuleBuilder;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.factory.AdapterPanelComponentsFactory;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.factory.Ros2CommAdapterPanelFactory;
import org.opentcs.customizations.controlcenter.ControlCenterInjectionModule;

/**
 * A Guice module for ROS2 Driver - Control Center configuration.
 *
 * @author Niels Tiben
 */
public class Ros2ControlCenterInjectionModule extends ControlCenterInjectionModule {

  @Override
  protected void configure() {
    install(new FactoryModuleBuilder().build(AdapterPanelComponentsFactory.class));
    commAdapterPanelFactoryBinder().addBinding().to(Ros2CommAdapterPanelFactory.class);
  }
}
