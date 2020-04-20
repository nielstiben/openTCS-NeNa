package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.task;

/**
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public class TaskUnknownException extends Exception{
    public TaskUnknownException(String taskType) {
        super(String.format("The requested task type is not supported: %s", taskType));
    }
}
