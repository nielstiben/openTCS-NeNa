/**
 * Copyright (c) The openTCS Authors.
 *
 * This program is free software and subject to the MIT license. (For details,
 * see the licensing information (LICENSE.txt) you should have received with
 * this copy of the software.)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.factory;

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

import static java.util.Objects.requireNonNull;

/**
 * A factory for creating {@link Ros2CommAdapterPanel} instances.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class Ros2CommAdapterPanelFactory
    implements VehicleCommAdapterPanelFactory {

  /**
   * This class's logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapterPanelFactory.class);
  /**
   * The service portal.
   */
  private final KernelServicePortal servicePortal;
  /**
   * The components factory.
   */
  private final AdapterPanelComponentsFactory componentsFactory;
  /**
   * Whether this factory is initialized or not.
   */
  private boolean initialized;

  /**
   * Creates a new instance.
   *
   * @param servicePortal The service portal.
   * @param componentsFactory The components factory.
   */
  @Inject
  public Ros2CommAdapterPanelFactory(KernelServicePortal servicePortal,
                                     AdapterPanelComponentsFactory componentsFactory) {
    this.servicePortal = requireNonNull(servicePortal, "servicePortal");
    this.componentsFactory = requireNonNull(componentsFactory, "componentsFactory");
    LOG.info("====================== ROS2 Panel injected ======================");
  }

  @Override
  public void initialize() {
    if (isInitialized()) {
      return;
    }

    initialized = true;
  }

  @Override
  public boolean isInitialized() {
    return initialized;
  }

  @Override
  public void terminate() {
    if (!isInitialized()) {
      return;
    }

    initialized = false;
  }

  @Override
  public List<VehicleCommAdapterPanel> getPanelsFor(
      @Nonnull VehicleCommAdapterDescription description,
      @Nonnull TCSObjectReference<Vehicle> vehicle,
      @Nonnull VehicleProcessModelTO processModel) {
    requireNonNull(description, "description");
    requireNonNull(vehicle, "vehicle");
    requireNonNull(processModel, "processModel");

    if (!providesPanelsFor(description, processModel)) {
      return new ArrayList<>();
    }

    List<VehicleCommAdapterPanel> panels = new ArrayList<>();
    panels.add(componentsFactory.createPanel(((Ros2ProcessModelTO) processModel),
                                             servicePortal.getVehicleService()));
    return panels;
  }

  /**
   * Checks whether this factory can provide comm adapter panels for the given description and the
   * given type of process model.
   *
   * @param description The description to check for.
   * @param processModel The process model.
   * @return {@code true} if, and only if, this factory can provide comm adapter panels for the
   * given description and the given type of process model.
   */
  private boolean providesPanelsFor(VehicleCommAdapterDescription description,
                                    VehicleProcessModelTO processModel) {
    return (description instanceof Ros2CommAdapterDescription)
        && (processModel instanceof Ros2ProcessModelTO);
  }
}
