package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.task;

import geometry_msgs.msg.PoseStamped;
import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.MessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.NavigationGoalListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.NavigationGoalTracker;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;
import java.util.Queue;

/**
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public class CommandExecutor implements NavigationGoalListener {
    private static final Logger LOG = LoggerFactory.getLogger(CommandExecutor.class);
    private final Ros2ProcessModel processModelInstance;
    private final Queue<MovementCommand> movementSentQueue;
    private final Queue<MovementCommand> movementCommandsQueue;

    private MovementCommand currentMovementCommand;
    private boolean isCommandExecutorActive = false;

    /* --------------- 0: Construct and enable ---------------*/
    public CommandExecutor(
            @Nonnull Ros2ProcessModel processModelInstance,
            @Nonnull Queue<MovementCommand> movementSentQueue,
            @Nonnull Queue<MovementCommand> movementCommandsQueue
    ) {
        this.processModelInstance = processModelInstance;
        this.movementSentQueue = movementSentQueue;
        this.movementCommandsQueue = movementCommandsQueue;
    }

    public void enableNavigationGoalListener() {
        NavigationGoalTracker goalTracker = processModelInstance.getNavigationGoalTracker();
        assert goalTracker != null;
        goalTracker.setCommandExecutorListener(this);


    }

    /* --------------- 1: Create Movement Command ---------------*/
    public void executeMovementCommand(@Nonnull MovementCommand movementCommand) {
        // Set active label
        this.currentMovementCommand = movementCommand;
        this.isCommandExecutorActive = true;

        // Get Destination
        Point destinationPoint = this.currentMovementCommand.getStep().getDestinationPoint();

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
            Point currentDestination = this.currentMovementCommand.getStep().getDestinationPoint();
            assert currentDestination.equals(point);
        } else {
            // We don't care, just proceed.
        }

        // Next step (3: Current Movement Command Succeeded) is activated by callback.
    }

    /* --------------- 3: Current Movement Command Active ---------------*/

    @Override
    public void onNavigationGoalSucceeded(Point point) {
        if (this.isCommandExecutorActive) {
            Point currentDestination = this.currentMovementCommand.getStep().getDestinationPoint();

            // Callback that a new navigation goal has been activated: let's check if its point equals our destination point.
            if (currentDestination.equals(point)) {
                // Vehicle reached its intended destination
                LOG.info("Processing MovementCommand Succeeded: Vehicle reached its intended destination!");
            } else {
                // Vehicle did not reach its intended destination, but a different destination.
                LOG.info("Processing MovementCommand Failed: Vehicle did not reach its intended destination, but a different destination");
                return;
            }

            executeOperationIfNeeded();
        } else {
            // We don't care, just proceed.
        }

    }

    /* --------------- 4: Execute Operation (if needed)---------------*/

    @SneakyThrows
    private void executeOperationIfNeeded() {
        if (this.currentMovementCommand.isWithoutOperation()) {
            // No operation in this command => do nothing.
        } else {
            String operation = this.currentMovementCommand.getOperation();
            switch (operation) {
                case OperationConstants.LOAD_CARGO:
                    LOG.info("CommandExecutor: Task Load Cargo not yet implemented");
                    break;
                case OperationConstants.UNLOAD_CARGO:
                    LOG.info("CommandExecutor: Task Unload Cargo not yet implemented");
                    break;
                default:
                    throw new TaskNotImplementedException(operation);
            }
        }

        processNewCommand();
    }

    /* --------------- 5: Check if there are already new ---------------*/
    private void processNewCommand() {
        this.processModelInstance.setVehicleState(Vehicle.State.IDLE);
        this.isCommandExecutorActive = false;

//        if (movementSentQueue.size() <= 1 && this.movementCommandsQueue.isEmpty()) {
//            this.processModelInstance.setVehicleState(Vehicle.State.IDLE);
//        } else {
//            // TODO: do more!!
//        }
    }
}
