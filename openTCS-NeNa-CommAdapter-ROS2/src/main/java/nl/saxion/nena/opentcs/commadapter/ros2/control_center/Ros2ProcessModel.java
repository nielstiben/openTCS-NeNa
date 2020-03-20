package nl.saxion.nena.opentcs.commadapter.ros2.control_center;
/*
VehicleProcessModelTO instances should be provided by every VehicleCommAdapter instance according to the current state
of its VehicleProcessModel. Instances of this model are supposed to be used in a comm adapter’s VehicleCommAdapterPanel
instances for updating their contents only. Note that VehicleProcessModelTO is basically a serializable representation
of a comm adapter’s VehicleProcessModel. Developers should keep that in mind when creating driver-specific subclasses
of VehicleProcessModelTO.
 */
public class Ros2ProcessModel {
}
