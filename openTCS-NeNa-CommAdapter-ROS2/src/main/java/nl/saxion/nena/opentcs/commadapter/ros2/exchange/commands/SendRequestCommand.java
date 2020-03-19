/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.exchange.commands;

import nl.saxion.nena.opentcs.commadapter.ros2.ExampleCommAdapter;
import nl.saxion.nena.opentcs.common.telegrams.Request;
import static java.util.Objects.requireNonNull;

import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

/**
 * A command for sending a telegram to the actual vehicle.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class SendRequestCommand
    implements AdapterCommand {

  /**
   * The request to send.
   */
  private final Request request;

  /**
   * Creates a new instance.
   *
   * @param request The request to send.
   */
  public SendRequestCommand(Request request) {
    this.request = requireNonNull(request, "request");
  }

  @Override
  public void execute(VehicleCommAdapter adapter) {
    if (!(adapter instanceof ExampleCommAdapter)) {
      return;
    }

    ExampleCommAdapter exampleAdapter = (ExampleCommAdapter) adapter;
    exampleAdapter.getRequestResponseMatcher().enqueueRequest(request);
  }
}
