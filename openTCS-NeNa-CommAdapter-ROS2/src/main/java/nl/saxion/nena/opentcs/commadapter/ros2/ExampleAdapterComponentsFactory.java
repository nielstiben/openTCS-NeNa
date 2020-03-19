/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import nl.saxion.nena.opentcs.common.telegrams.RequestResponseMatcher;
import nl.saxion.nena.opentcs.common.telegrams.StateRequesterTask;
import nl.saxion.nena.opentcs.common.telegrams.TelegramSender;
import java.awt.event.ActionListener;
import org.opentcs.data.model.Vehicle;

/**
 * A factory for various instances specific to the comm adapter.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public interface ExampleAdapterComponentsFactory {

  /**
   * Creates a new ExampleCommAdapter for the given vehicle.
   *
   * @param vehicle The vehicle
   * @return A new ExampleCommAdapter for the given vehicle
   */
  ExampleCommAdapter createExampleCommAdapter(Vehicle vehicle);

  /**
   * Creates a new {@link RequestResponseMatcher}.
   *
   * @param telegramSender Sends telegrams/requests.
   * @return The created {@link RequestResponseMatcher}.
   */
  RequestResponseMatcher createRequestResponseMatcher(TelegramSender telegramSender);

  /**
   * Creates a new {@link StateRequesterTask}.
   *
   * @param stateRequestAction The actual action to be performed to enqueue requests.
   * @return The created {@link StateRequesterTask}.
   */
  StateRequesterTask createStateRequesterTask(ActionListener stateRequestAction);
}
