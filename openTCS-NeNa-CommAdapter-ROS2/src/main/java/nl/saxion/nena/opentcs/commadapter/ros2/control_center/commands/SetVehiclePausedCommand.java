/**
 * Copyright (c) The openTCS Authors.
 *
 * This program is free software and subject to the MIT license. (For details,
 * see the licensing information (LICENSE.txt) you should have received with
 * this copy of the software.)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

/**
 * A command to pause/unpause the vehicle.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class SetVehiclePausedCommand
    implements AdapterCommand {

  /**
   * Whether to pause/unpause the vehicle.
   */
  private final boolean paused;

  /**
   * Creates a new instance.
   *
   * @param paused Whether to pause/unpause the vehicle.
   */
  public SetVehiclePausedCommand(boolean paused) {
    this.paused = paused;
  }

  @Override
  public void execute(VehicleCommAdapter adapter) {
    if (!(adapter instanceof Ros2CommAdapter)) {
      return;
    }

    Ros2CommAdapter loopbackAdapter = (Ros2CommAdapter) adapter;
    loopbackAdapter.getProcessModel().setVehiclePaused(paused);
  }

}
