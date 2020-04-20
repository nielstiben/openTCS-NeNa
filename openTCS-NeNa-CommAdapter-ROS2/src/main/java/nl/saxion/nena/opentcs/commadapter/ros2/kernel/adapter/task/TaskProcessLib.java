package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.task;

import org.opentcs.drivers.vehicle.MovementCommand;

public class TaskLib {
    public static void executeTaskByMessage(Object message){
        if (message instanceof MovementCommand){

        } else {
            // Unsupported task
            throw new
        }
    }
}
