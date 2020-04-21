package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.task;

/**
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public class TaskNotImplementedException extends Exception {
    public TaskNotImplementedException(String taskType) {
        super(String.format("The requested task type is not implemented: %s", taskType));
    }
}
