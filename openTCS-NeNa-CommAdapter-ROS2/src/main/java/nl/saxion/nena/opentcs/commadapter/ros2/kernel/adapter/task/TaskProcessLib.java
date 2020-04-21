package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.task;

import org.opentcs.drivers.vehicle.MovementCommand;

public class TaskProcessLib {
    public static void executeTaskByMessage(Object message) throws TaskNotImplementedException {
        if (message instanceof MovementCommand){
            processMovementCommand((MovementCommand) message);
        } else {
            // Unsupported task
            throw new TaskNotImplementedException(message.getClass().toString());
        }
    }

    private static void processMovementCommand(MovementCommand movementCommand){
        //todo
    }
}
