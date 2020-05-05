package nl.saxion.nena.opentcs.commadapter.ros2.control_center.control_panel;

import lombok.NoArgsConstructor;
import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;

import java.util.ResourceBundle;

import static nl.saxion.nena.opentcs.commadapter.ros2.I18nROS2CommAdapter.BUNDLE_PATH;

/**
 * Class for providing a description of the ROS2 adapter.
 *
 * @author Niels Tiben
 */
@NoArgsConstructor
public class Ros2CommAdapterDescription extends VehicleCommAdapterDescription {

    @Override
    public String getDescription() {
        return ResourceBundle.getBundle(BUNDLE_PATH).getString("ros2CommAdapterDescription.description");
    }
}
