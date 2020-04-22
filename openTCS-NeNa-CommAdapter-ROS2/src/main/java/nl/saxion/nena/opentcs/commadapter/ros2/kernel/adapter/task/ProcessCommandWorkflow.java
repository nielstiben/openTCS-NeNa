package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.task;

import geometry_msgs.msg.PoseStamped;
import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.MessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.NavigationGoalListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.NavigationGoalTracker;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import org.opentcs.data.model.Point;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;
import java.util.Queue;

/**
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public class ProcessCommandWorkflow implements NavigationGoalListener {
    private static final Logger LOG = LoggerFactory.getLogger(ProcessCommandWorkflow.class);
    private final Ros2ProcessModel processModelInstance;
    private final Queue<MovementCommand> sentQueue;
    private final Queue<MovementCommand> commandsQueue;

    private MovementCommand currentCommand;
    private boolean isCommandExecutorActive = false;

    /* --------------- 0: Construct and enable ---------------*/

    public ProcessCommandWorkflow(
            @Nonnull Ros2ProcessModel processModelInstance,
            @Nonnull Queue<MovementCommand> sentQueue,
            @Nonnull Queue<MovementCommand> commandsQueue
    ) {
        this.processModelInstance = processModelInstance;
        this.sentQueue = sentQueue;
        this.commandsQueue = commandsQueue;
    }

    public void enableNavigationGoalListener() {
        NavigationGoalTracker goalTracker = processModelInstance.getNavigationGoalTracker();

        assert goalTracker != null;
        goalTracker.setCommandExecutorListener(this);
    }

    /* --------------- 0: Workflow validation ---------------*/

    public void processMovementCommandFromMovementCommand(MovementCommand movementCommand) {
        assert !this.isCommandExecutorActive;
        this.currentCommand = movementCommand;
        processMovementCommand();
    }

    /* --------------- 1: Create Movement Command ---------------*/
//    private void processMovementCommandFromQueue(){
//        this.currentCommand = this.sentQueue.peek();
//        processMovementCommand();
//    }

    private void processMovementCommand(){
        // Set active label
        this.isCommandExecutorActive = true;

        // Get Destination
        assert this.currentCommand != null;
        Point destinationPoint = this.currentCommand.getStep().getDestinationPoint();

        // Generate message.
        LOG.info("Command Execution: Dispatching vehicle to point '{}'", destinationPoint.getName());
        PoseStamped message = MessageLib.generateNavigationMessageByPoint(destinationPoint);

        // Notify NavigationGoalTracker that we expect a new goal.
        processModelInstance.getNavigationGoalTracker().setDestinationPointIncomingGoal(destinationPoint);

        // Give dispatch order.
        processModelInstance.getNodeManager().getNode().getGoalPublisher().publish(message);

        // Next step (2: Current Movement Command Active) is activated by callback.
    }

    /* --------------- 2: Current Movement Command Active ---------------*/

    @Override
    public void onNavigationGoalActive(Point point) {
        // Callback received that a new navigation goal has been activated.

        if (this.isCommandExecutorActive) {
            // Let's check if its point equals our destination point.
            Point currentDestination = this.currentCommand.getStep().getDestinationPoint();
            System.out.println("--Intended-- " + currentDestination.toString());
            System.out.println("--Found-- " + point.toString());
            assert currentDestination.equals(point);
        } else {
            // Not intended for CommandExecutor so we don't care, just proceed.
        }

        // Next step (3: Current Movement Command Succeeded) is activated by callback.
    }

    /* --------------- 3: Current Movement Command Active ---------------*/

    @Override
    public void onNavigationGoalSucceeded(Point point) {
        if (this.isCommandExecutorActive) {
            Point currentDestination = this.currentCommand.getStep().getDestinationPoint();

            // Callback that a new navigation goal has been activated: let's check if its point equals our destination point.
            if (currentDestination.equals(point)) {
                // Vehicle reached its intended destination
                LOG.info("Processing MovementCommand Succeeded: Vehicle reached its intended destination! =>" + currentDestination.toString());
            } else {
                // Vehicle did not reach its intended destination, but a different destination.
                LOG.info("Processing MovementCommand Failed: Vehicle did not reach its intended destination, but a different destination ( " + point.toString() + " -instead of- " + currentDestination.toString() + " )");
                return;
            }

            executeOperationIfNeeded();
        } else {
            // Not intended for CommandExecutor so we don't care, just proceed.
        }

    }

    /* --------------- 4: Execute Operation (if needed)---------------*/

    @SneakyThrows
    private void executeOperationIfNeeded() {
        if (this.currentCommand.isWithoutOperation()) {
            // No operation in this command => do nothing.
        } else {
            String operation = this.currentCommand.getOperation();
            switch (operation) {
                case OperationConstants.LOAD_CARGO:
                    LOG.info("CommandExecutor: Task Load Cargo not yet implemented");
                    break;
                case OperationConstants.UNLOAD_CARGO:
                    LOG.info("CommandExecutor: Task Unload Cargo not yet implemented");
                    break;
                default:
                    // TODO: Command Failed:
//                    this.processModelInstance.commandFailed(this.currentCommand);
                    throw new TaskNotImplementedException(operation);
            }
        }

        processNewCommandIfNeeded();
    }

    /* --------------- 5: Check if there are already new ---------------*/
    private void processNewCommandIfNeeded() {
        LOG.info("2 Sent queue: " + this.sentQueue.toString());
        LOG.info("2 Comm queue: " + this.commandsQueue.toString());

        // Remove it from the queue
        MovementCommand movementCommandOnQueue = sentQueue.poll();
        assert movementCommandOnQueue != null && movementCommandOnQueue.equals(currentCommand);

        this.isCommandExecutorActive = false;
        // Notify the vehicle manager that current command has finished.
        this.processModelInstance.commandExecuted(this.currentCommand);

        LOG.info("3 Sent queue: " + this.sentQueue.toString());
        LOG.info("3 Comm queue: " + this.commandsQueue.toString());

        // TODO:
        // If vehicle reached its final destination
//        this.currentCommand.getFinalDestination().equals(currentPosition);

        // Then set vehicle state to IDLE, because we don't expect a new task.
//        this.processModelInstance.setVehicleState(Vehicle.State.IDLE);



//        // If the command queue was cleared in the meantime, the kernel
//        // might be surprised to hear we executed a command we shouldn't
//        // have, so we only peek() at the beginning of this method and
//        // poll() here. If sentCmd is null, the queue was probably cleared
//        // and we shouldn't report anything back.
//        this.sentQueue.poll();
//
//        if (isCommandQueueEmpty()) {
//            // Finish: there is nothing left to process.
//            System.out.println("No more commands found");
//            this.processModelInstance.setVehicleState(Vehicle.State.IDLE);
//        } else {
//            // There are more commands: repeat steps 1 till 5.
//            System.out.println("More commands found:");
//            System.out.println(sentQueue.toString());
//            System.out.println(commandsQueue.toString());
//            processMovementCommandFromQueue();
//        }
    }

//    private boolean isCommandQueueEmpty() {
//        return this.sentQueue.size() <= 1 && this.commandsQueue.isEmpty();
//    }
}
