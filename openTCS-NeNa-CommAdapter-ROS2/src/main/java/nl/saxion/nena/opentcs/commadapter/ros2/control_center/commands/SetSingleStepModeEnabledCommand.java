/**
 * Copyright (c) The openTCS Authors.
 *
 * This program is free software and subject to the MIT license. (For details,
 * see the licensing information (LICENSE.txt) you should have received with
 * this copy of the software.)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.Ros2CommAdapter;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

/**
 * A command to enable/disable the comm adapter's single step mode.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class SetSingleStepModeEnabledCommand
    implements AdapterCommand {

  /**
   * Whether to enable/disable single step mode.
   */
  private final boolean enabled;

  /**
   * Creates a new instance.
   *
   * @param enabled Whether to enable/disable single step mode.
   */
  public SetSingleStepModeEnabledCommand(boolean enabled) {
    this.enabled = enabled;
  }

  @Override
  public void execute(VehicleCommAdapter adapter) {
    if (!(adapter instanceof Ros2CommAdapter)) {
      return;
    }

    Ros2CommAdapter loopbackAdapter = (Ros2CommAdapter) adapter;
    loopbackAdapter.getProcessModel().setSingleStepModeEnabled(enabled);
  }

}
