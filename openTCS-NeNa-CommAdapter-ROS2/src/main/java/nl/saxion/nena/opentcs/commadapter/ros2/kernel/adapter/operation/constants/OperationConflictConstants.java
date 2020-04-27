package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.OperationAllowedLib;

/**
 * Constants for operation initiation conflicts that are raised in {@link OperationAllowedLib}.
 *
 * @author Niels Tiben
 */
public interface OperationConflictConstants {
    String LOAD_OPERATION_CONFLICT = "cannotLoadWhenLoaded";
    String UNLOAD_OPERATION_CONFLICT = "cannotUnloadWhenNotLoaded";
}
