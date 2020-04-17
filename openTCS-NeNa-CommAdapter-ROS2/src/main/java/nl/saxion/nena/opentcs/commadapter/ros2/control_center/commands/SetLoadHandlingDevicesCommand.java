package nl.saxion.nena.opentcs.commadapter.ros2.control_center.commands;

import lombok.AllArgsConstructor;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

import javax.annotation.Nonnull;
import java.util.List;

import static java.util.Objects.requireNonNull;

/**
 * A command to set the {@link LoadHandlingDevice}s attached to a vehicle.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
@AllArgsConstructor
public class SetLoadHandlingDevicesCommand  implements AdapterCommand {

  /**
   * The list of load handling devices.
   */
  private final List<LoadHandlingDevice> devices;

  @Override
  public void execute(VehicleCommAdapter adapter) {
    adapter.getProcessModel().setVehicleLoadHandlingDevices(devices);
  }
}
