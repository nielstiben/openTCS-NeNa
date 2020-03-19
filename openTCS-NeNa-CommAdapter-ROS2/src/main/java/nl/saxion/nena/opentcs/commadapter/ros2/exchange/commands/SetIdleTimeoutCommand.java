/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands;

import nl.saxion.nena.opentcs.commadapter.ros2.ExampleCommAdapter;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

/**
 * A command to set the adapters's idle timeout.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class SetIdleTimeoutCommand
    implements AdapterCommand {

  /**
   * The idle timeout to set.
   */
  private final int timeout;

  /**
   * Creates a new instance.
   *
   * @param timeout The idle timeout to set.
   */
  public SetIdleTimeoutCommand(int timeout) {
    this.timeout = timeout;
  }

  @Override
  public void execute(VehicleCommAdapter adapter) {
    if (!(adapter instanceof ExampleCommAdapter)) {
      return;
    }

    ExampleCommAdapter exampleAdapter = (ExampleCommAdapter) adapter;
    exampleAdapter.getProcessModel().setVehicleIdleTimeout(timeout);
  }
}
