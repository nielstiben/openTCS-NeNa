package nl.saxion.nena.opentcs.commadapter.ros2.control_center.factory;

import nl.saxion.nena.opentcs.commadapter.ros2.control_center.Ros2CommAdapterPanel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModelTO;
import org.opentcs.components.kernel.services.VehicleService;

import javax.annotation.Nonnull;

/**
 * An interface for allowing the injection module to create an instance of the factory.
 *
 * @author Niels Tiben
 */
public interface AdapterPanelComponentsFactory {

    /**
     * Creates a {@link Ros2CommAdapterPanel} representing the given process model's content.
     *
     * @param processModel The process model to represent.
     * @param vehicleService The vehicle service used for interaction with the comm adapter.
     * @return The comm adapter panel.
     */
    Ros2CommAdapterPanel createPanel(@Nonnull Ros2ProcessModelTO processModel, @Nonnull VehicleService vehicleService);
}
