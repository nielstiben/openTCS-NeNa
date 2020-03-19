/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.common.dispatching;

import org.opentcs.data.order.DriveOrder;

/**
 * Defines (configurable) strings for loading and unloading that can be used for vehicle actions in 
 * the kernel's model.
 *
 * @author Stefan Walter (Fraunhofer IML)
 */
public interface LoadAction {

  public static final String NONE = DriveOrder.Destination.OP_NOP;
  /**
   * A constant for adding load.
   */
  public static final String LOAD = "Load cargo";
  /**
   * A constant for removing load.
   */
  public static final String UNLOAD = "Unload cargo";
  /**
   * A constant for charging the battery.
   */
  public static final String CHARGE = "Charge";
}
