package nl.saxion.nena.opentcs.commadapter.ros2.control_center;

import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;

import java.util.ResourceBundle;

import static nl.saxion.nena.opentcs.commadapter.ros2.virtual_vehicle.I18nLoopbackCommAdapter.BUNDLE_PATH;


public class Ros2CommAdapterDescription extends VehicleCommAdapterDescription {

    @Override
    public String getDescription() {
        return ResourceBundle.getBundle(BUNDLE_PATH)
                .getString("ros2CommAdapterDescription.description");
    }

    @Override
    public boolean isSimVehicleCommAdapter() {
        return true;
    }

}
