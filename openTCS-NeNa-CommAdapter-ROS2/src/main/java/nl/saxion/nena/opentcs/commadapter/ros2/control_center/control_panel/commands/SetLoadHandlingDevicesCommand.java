/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel.commands;

import lombok.AllArgsConstructor;
import org.opentcs.drivers.vehicle.AdapterCommand;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;
import org.opentcs.drivers.vehicle.VehicleCommAdapter;

import javax.annotation.Nonnull;
import java.util.List;

/**
 * Instruct the kernel to set the vehicle its {@link LoadHandlingDevice}.
 *
 * @author Niels Tiben
 */
@AllArgsConstructor
public class SetLoadHandlingDevicesCommand implements AdapterCommand {
    private final List<LoadHandlingDevice> loadHandlingDevices;

    @Override
    public void execute(@Nonnull VehicleCommAdapter adapter) {
        adapter.getProcessModel().setVehicleLoadHandlingDevices(this.loadHandlingDevices);
    }
}
