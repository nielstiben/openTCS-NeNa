/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.operation.constants;

/**
 * Constants with operation names of all operations that are allowed to be carried out by the vehicle driver.
 *
 * @author Niels Tiben
 */
public interface OperationConstants {
    String MOVE = "MOVE";
    String NOP = "NOP";
    String LOAD_CARGO = "Load cargo";
    String UNLOAD_CARGO = "Unload cargo";
}