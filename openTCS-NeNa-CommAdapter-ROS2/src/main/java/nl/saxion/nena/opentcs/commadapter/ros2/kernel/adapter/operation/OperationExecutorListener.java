package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation;

/**
 * @author Niels Tiben
 */
public interface OperationExecutorListener {
    void onOperationExecutionSucceeded();
    void onOperationExecutionFailed(String reason);
}
