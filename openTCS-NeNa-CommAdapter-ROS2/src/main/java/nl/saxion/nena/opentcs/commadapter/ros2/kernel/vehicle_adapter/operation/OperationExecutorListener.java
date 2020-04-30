package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.operation;

/**
 * Listener for notifying about an update about the execution of an operation.
 *
 * @author Niels Tiben
 */
public interface OperationExecutorListener {
    void onOperationExecutionSucceeded();

    void onOperationExecutionFailed(String reason);
}
