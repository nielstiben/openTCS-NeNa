/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands;

import nl.saxion.nena.opentcs.commadapter.ros2.ExampleCommAdapter;
import static java.util.Objects.requireNonNull;

import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

/**
 * A command to set the vehicle's host.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class SetVehicleHostCommand
    implements AdapterCommand {

  /**
   * The host to set.
   */
  private final String host;

  /**
   * Creates a new instnace.
   *
   * @param host The host to set.
   */
  public SetVehicleHostCommand(String host) {
    this.host = requireNonNull(host, "host");
  }

  @Override
  public void execute(VehicleCommAdapter adapter) {
    if (!(adapter instanceof ExampleCommAdapter)) {
      return;
    }

    ExampleCommAdapter exampleAdapter = (ExampleCommAdapter) adapter;
    exampleAdapter.getProcessModel().setVehicleHost(host);
  }
}
