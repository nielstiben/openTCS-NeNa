/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.telegrams;

import nl.saxion.nena.opentcs.common.dispatching.LoadAction;

/**
 * Defines all actions that a vehicle can execute as part of an order.
 *
 * @author Mats Wilhelm (Fraunhofer IML)
 */
public enum OrderAction {
  /**
   * No action.
   */
  NONE('N'),
  /**
   * Action to load an object.
   */
  LOAD('L'),
  /**
   * Action to unload an object.
   */
  UNLOAD('U'),
  /**
   * Charge vehicle.
   */
  CHARGE('C');

  /**
   * The actual byte to put into the telegram to the vehicle.
   */
  private final byte actionByte;

  /**
   * Creates a new Action.
   *
   * @param action The actual byte to put into the telegram to the vehicle.
   */
  OrderAction(char action) {
    this.actionByte = (byte) action;
  }

  /**
   * Returns the actual byte to put into the telegram to the vehicle.
   *
   * @return The actual byte to put into the telegram to the vehicle.
   */
  public byte getActionByte() {
    return actionByte;
  }

  /**
   * Maps the given <code>actionString</code> to an Action.
   *
   * @param actionString
   * @return The Action associated with the <code>actionString</code>. Returns
   * <code>Action.NONE</code> if there isn't any Action associated with the <code>actionString</code>.
   */
  public static OrderAction stringToAction(String actionString) {
    OrderAction action = NONE;
    if (actionString.equals(LoadAction.LOAD)) {
      action = LOAD;
    }
    if (actionString.equals(LoadAction.UNLOAD)) {
      action = UNLOAD;
    }
    if (actionString.equals(LoadAction.CHARGE)) {
      action = CHARGE;
    }
    return action;
  }
}
