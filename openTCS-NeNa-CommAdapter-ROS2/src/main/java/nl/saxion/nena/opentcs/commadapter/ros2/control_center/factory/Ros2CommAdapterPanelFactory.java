package nl.saxion.nena.opentcs.commadapter.ros2.control_center.factory;

import lombok.Getter;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.Ros2CommAdapterDescription;
import nl.saxion.nena.opentcs.commadapter.ros2.control_center.Ros2CommAdapterPanel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModelTO;
import org.opentcs.access.KernelServicePortal;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleCommAdapterDescription;
import org.opentcs.drivers.vehicle.management.VehicleCommAdapterPanel;
import org.opentcs.drivers.vehicle.management.VehicleCommAdapterPanelFactory;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;
import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

/**
 * A factory for creating {@link Ros2CommAdapterPanel} instances on demand.
 *
 * @author Niels Tiben
 */
public class Ros2CommAdapterPanelFactory implements VehicleCommAdapterPanelFactory {
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapterPanelFactory.class);
    private final KernelServicePortal servicePortal;
    private final AdapterPanelComponentsFactory componentsFactory;

    @Getter
    private boolean initialized;

    //================================================================================
    // Constructor
    //================================================================================
    @Inject
    public Ros2CommAdapterPanelFactory(@Nonnull KernelServicePortal servicePortal,
                                       @Nonnull AdapterPanelComponentsFactory componentsFactory) {
        this.servicePortal = servicePortal;
        this.componentsFactory = componentsFactory;
    }

    //================================================================================
    // Override methods
    //================================================================================
    @Override
    public void initialize() {
        if (isInitialized()) {
            return;
        }
        LOG.info("Initialising ROS2 Comm adapter panel.");
        this.initialized = true;
    }

    @Override
    public void terminate() {
        if (!isInitialized()) {
            return;
        }
        this.initialized = false;
    }

    @Override
    public List<VehicleCommAdapterPanel> getPanelsFor(
            @Nonnull VehicleCommAdapterDescription description,
            @Nonnull TCSObjectReference<Vehicle> vehicle,
            @Nonnull VehicleProcessModelTO processModel) {

        if (canProvidePanelForDescriptionAndProcessModel(description, processModel)) {
            // The factory is able to provide a comm adapter panel for this description and/or process model.
            List<VehicleCommAdapterPanel> panels = new ArrayList<>();
            panels.add(
                    this.componentsFactory.createPanel(((Ros2ProcessModelTO) processModel),
                    this.servicePortal.getVehicleService())
            );

            return panels;
        } else {
            // The factory cannot provide a comm adapter panel for this description and/or process model.
            LOG.debug("Cannot provide panels for '{}' with '{}'.", description, processModel);

            return new ArrayList<>();
        }
    }

    //================================================================================
    // Private functions
    //================================================================================
    /**
     * Checks whether this factory can provide comm adapter panels for the given description and the
     * given type of process model.
     *
     * @param description  The description to check for.
     * @param processModel The process model.
     * @return {@code true} if, and only if, this factory can provide comm adapter panels for the
     * given description and the given type of process model.
     */
    private boolean canProvidePanelForDescriptionAndProcessModel(@Nonnull VehicleCommAdapterDescription description,
                                                                 @Nonnull VehicleProcessModelTO processModel) {
        return (description instanceof Ros2CommAdapterDescription)
                && (processModel instanceof Ros2ProcessModelTO);
    }
}
