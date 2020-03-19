/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderAction;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderRequest;
import nl.saxion.nena.opentcs.common.telegrams.BoundedCounter;
import static nl.saxion.nena.opentcs.common.telegrams.BoundedCounter.UINT16_MAX_VALUE;
import nl.saxion.nena.opentcs.common.telegrams.Telegram;
import org.opentcs.data.model.Point;
import org.opentcs.drivers.vehicle.MovementCommand;

/**
 * Maps {@link MovementCommand}s from openTCS to a telegram sent to the vehicle.
 *
 * @author Mats Wilhelm (Fraunhofer IML)
 */
public class OrderMapper {

  /**
   * Counts the order id's sent to the vehicle.
   */
  private final BoundedCounter orderIdCounter = new BoundedCounter(1, UINT16_MAX_VALUE);

  /**
   * Creates a new instance.
   */
  public OrderMapper() {
  }

  /**
   * Maps the given command to an order request that can be sent to the vehicle.
   *
   * @param command The command to be mapped.
   * @return The order request to be sent.
   * @throws IllegalArgumentException If the movement command could not be mapped properly.
   */
  public OrderRequest mapToOrder(MovementCommand command)
      throws IllegalArgumentException {
    return new OrderRequest(Telegram.ID_DEFAULT,
                            orderIdCounter.getAndIncrement(),
                            extractDestinationId(command.getStep().getDestinationPoint()),
                            OrderAction.stringToAction(command.getFinalOperation()));
  }

  private static int extractDestinationId(Point point)
      throws IllegalArgumentException {
    try {
      return Integer.parseInt(point.getName());
    }
    catch (NumberFormatException e) {
      throw new IllegalArgumentException("Cannot parse point name: " + point.getName(), e);
    }
  }
}
