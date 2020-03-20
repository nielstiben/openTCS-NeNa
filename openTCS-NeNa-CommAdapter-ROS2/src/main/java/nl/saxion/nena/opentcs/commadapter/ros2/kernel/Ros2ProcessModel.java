package nl.saxion.nena.opentcs.commadapter.ros2.kernel;

import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleProcessModel;

import javax.annotation.Nonnull;

/*
Single instance should be provided by every VehicleCommAdapter instance in which it keeps the relevant state of both the
evhicle and the comm adapter. This model instance is supposed to be updated to notify the kernel about relevant changes.
The comm adapter implementation should e.g. update the vehicle’s current position in the model when it receives that
information to allow the kernel and GUI frontends to use it. Likewise, other components may set values that influence
the comm adapter’s behaviour in the model, e.g. a time interval for periodic messages the comm adapter sends to the
vehicle. VehicleProcessModel may be used as it is, as it contains members for all the information the openTCS kernel
itself needs. However, developers may use driver-specific subclasses of VehicleProcessModel to have the comm adapter
and other components exchange more than the default set of attributes.
 */
public class Ros2ProcessModel extends VehicleProcessModel {
    public Ros2ProcessModel(@Nonnull Vehicle attachedVehicle) {
        super(attachedVehicle);
    }
}
