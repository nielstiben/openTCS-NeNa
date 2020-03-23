package nl.saxion.nena.opentcs.commadapter.ros2.control_center;

import nl.saxion.nena.opentcs.commadapter.ros2.virtual_vehicle.Ros2ProcessModelTO;
import org.opentcs.components.kernel.services.VehicleService;

public interface AdapterPanelComponentsFactory {

    /**
     * Creates a {@link Ros2CommAdapterPanel} representing the given process model's content.
     *
     * @param processModel The process model to represent.
     * @param vehicleService The vehicle service used for interaction with the comm adapter.
     * @return The comm adapter panel.
     */
    Ros2CommAdapterPanel createPanel(Ros2ProcessModelTO processModel,
                                     VehicleService vehicleService);
}
