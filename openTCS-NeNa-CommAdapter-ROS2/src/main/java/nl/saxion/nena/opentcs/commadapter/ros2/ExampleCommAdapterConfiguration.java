/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import org.opentcs.configuration.ConfigurationEntry;
import org.opentcs.configuration.ConfigurationPrefix;

/**
 * Provides methods to configure the {@link ExampleCommAdapter}.
 *
 * @author Leonard Schuengel (Fraunhofer IML)
 */
@ConfigurationPrefix(ExampleCommAdapterConfiguration.PREFIX)
public interface ExampleCommAdapterConfiguration {

  /**
   * This configuration's prefix.
   */
  String PREFIX = "example.commadapter";

  @ConfigurationEntry(
      type = "Boolean",
      description = "Whether to register/enable the example communication adapter.",
      orderKey = "0_enable")
  boolean enable();

}
