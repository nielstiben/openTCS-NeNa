package nl.saxion.nena.opentcs.commadapter.ros2.control_center;

import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;

import java.util.ResourceBundle;

import static nl.saxion.nena.opentcs.commadapter.ros2.common.I18nROS2CommAdapter.BUNDLE_PATH;

public class Ros2CommAdapterDescription extends VehicleCommAdapterDescription {

    @Override
    public String getDescription() {
        return ResourceBundle.getBundle(BUNDLE_PATH)
                .getString("ros2CommAdapterDescription.description");
    }

    @Override
    public boolean isSimVehicleCommAdapter() {
        return false; // No, because we are a ROS2 comm adapter.
    }

}
