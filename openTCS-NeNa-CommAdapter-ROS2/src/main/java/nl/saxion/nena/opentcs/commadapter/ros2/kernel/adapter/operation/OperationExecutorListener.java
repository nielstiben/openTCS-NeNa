package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation;

/**
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public interface OperationExecutorListener {
    void onOperationExecutionSucceeded();
    void onOperationExecutionFailed(String reason);
}
