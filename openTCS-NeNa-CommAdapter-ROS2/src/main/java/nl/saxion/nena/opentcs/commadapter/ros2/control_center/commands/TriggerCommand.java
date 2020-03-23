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
 * A command to trigger the comm adapter in single step mode.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class TriggerCommand
    implements AdapterCommand {

  @Override
  public void execute(VehicleCommAdapter adapter) {
    if (!(adapter instanceof Ros2CommAdapter)) {
      return;
    }

    Ros2CommAdapter loopbackAdapter = (Ros2CommAdapter) adapter;
    loopbackAdapter.trigger();
  }
}
