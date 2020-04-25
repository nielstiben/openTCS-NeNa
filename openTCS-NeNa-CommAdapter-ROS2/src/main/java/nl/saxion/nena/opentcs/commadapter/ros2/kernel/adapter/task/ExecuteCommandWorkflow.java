package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.task;

import geometry_msgs.msg.PoseStamped;
import lombok.Getter;
import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.ExecuteOperationWorkflow;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.OperationExecutorListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.OutgoingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.NavigationGoalListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.NavigationGoalTracker;
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
public class ExecuteCommandWorkflow implements NavigationGoalListener, OperationExecutorListener {
    private static final Logger LOG = LoggerFactory.getLogger(ExecuteCommandWorkflow.class);
    private final Ros2ProcessModel processModelInstance;
    private final Queue<MovementCommand> sentQueue;

    @Getter
    private final ExecuteOperationWorkflow executeOperationWorkflow;

    private MovementCommand currentCommand;

    @Getter
    private boolean isCommandExecutorActive = false;

    /* --------------- 0: Construct and enable ---------------*/

    public ExecuteCommandWorkflow(
            @Nonnull Ros2ProcessModel processModelInstance,
            @Nonnull Queue<MovementCommand> sentQueue
    ) {
        this.processModelInstance = processModelInstance;
        this.sentQueue = sentQueue;
        this.executeOperationWorkflow = new ExecuteOperationWorkflow(this.processModelInstance, this);
    }

    public void enableNavigationGoalListener() {
        NavigationGoalTracker goalTracker = processModelInstance.getNavigationGoalTracker();

        assert goalTracker != null;

        goalTracker.setCommandExecutorListener(this);
        this.processModelInstance.setExecuteCommandWorkflow(this);
    }

    /* --------------- 1: Create Movement Command ---------------*/

    public void processMovementCommand(MovementCommand movementCommand) {
        assert !this.isCommandExecutorActive;
        this.currentCommand = movementCommand;

        // Set active label
        this.isCommandExecutorActive = true;

        // Set vehicle state
        processModelInstance.setVehicleState(Vehicle.State.EXECUTING);

        // Get Destination
        assert this.currentCommand != null;
        Point destinationPoint = this.currentCommand.getStep().getDestinationPoint();

        // Generate message.
        LOG.info("Command Execution: Dispatching vehicle to point '{}'", destinationPoint.getName());
        PoseStamped message = OutgoingMessageLib.generateScaledNavigationMessageByPoint(destinationPoint);

        // Notify NavigationGoalTracker that we expect a new goal.
        processModelInstance.getNavigationGoalTracker().setDestinationPointIncomingGoal(destinationPoint);

        // Give dispatch order.
        processModelInstance.getNodeManager().getNode().getGoalPublisher().publish(message);

        // Next step (2: Current Movement Command Active) is activated by callback.
    }

    private void processMovementCommand1() {

    }

    /* --------------- 2: Current Movement Command Active ---------------*/

    @Override
    public void onNavigationGoalActive(@Nonnull Point point) {
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
    public void onNavigationGoalSucceeded(@Nonnull Point point) {
        if (this.isCommandExecutorActive) {
            Point currentDestination = this.currentCommand.getStep().getDestinationPoint();

            // Callback that a new navigation goal has been activated: let's check if its point equals our destination point.
            if (currentDestination.equals(point)) {
                // Vehicle reached its intended destination
                LOG.info("Processing MovementCommand Succeeded: Vehicle reached its intended destination! =>" + currentDestination.toString());
                executeOperationIfNeeded();
            } else {
                // Vehicle did not reach its intended destination, but a different destination.
                String reason = String.format("Processing MovementCommand Failed: Vehicle did not reach its intended destination, but a different destination ('%s' instead of '%s')", point.toString(), currentDestination.toString());
                onOperationExecutionFailed(reason);
            }
        } else {
            // Not intended for CommandExecutor so we don't care, just proceed.
        }

    }

    /* --------------- 4: Execute Operation (if needed)---------------*/

    @SneakyThrows
    private void executeOperationIfNeeded() {
        if (this.currentCommand.isWithoutOperation()) {
            // No operation in this command => skip this step.
            setCommandWorkflowSucceeded();
        } else {
            this.executeOperationWorkflow.executeActionByName(this.currentCommand.getOperation());
        }
    }

    /* --------------- 4a: Operation has finished ---------------*/

    @Override
    public void onOperationExecutionSucceeded() {
        LOG.info("Operation succeeded!");
        setCommandWorkflowSucceeded();
    }

    /* --------------- 4b: Operation has failed ---------------*/

    @Override
    public void onOperationExecutionFailed(String reason) {
        LOG.info(String.format("Operation failed: %s", reason));
        setCommandWorkflowFailed();
    }

    /* --------------- 5: Check if there are already new ---------------*/
    private void setCommandWorkflowSucceeded() {
        removeExecutedCommandFromSentQueueAndSetWorkflowInactive();

        this.processModelInstance.commandExecuted(this.currentCommand);

        if (hasVehicleReachedFinalDestination()) {
            // We reached our final destination and executed all operations.
            this.processModelInstance.setVehicleState(Vehicle.State.IDLE);
        } else {
            // We are expecting more commands to come, so let's keep the state on EXECUTING.
            assert this.processModelInstance.getVehicleState().equals(Vehicle.State.EXECUTING);
        }
    }

    private void setCommandWorkflowFailed() {
        removeExecutedCommandFromSentQueueAndSetWorkflowInactive();

        this.processModelInstance.commandFailed(this.currentCommand);
        this.processModelInstance.setVehicleState(Vehicle.State.ERROR);
    }

    private boolean hasVehicleReachedFinalDestination() {
        Point finalDestination = this.currentCommand.getFinalDestination();
        Point currentStepDestination = this.currentCommand.getStep().getDestinationPoint();

        return currentStepDestination.equals(finalDestination);
    }

    private void removeExecutedCommandFromSentQueueAndSetWorkflowInactive() {
        // Remove current command from the queue
        MovementCommand movementCommandOnQueue = this.sentQueue.poll();
        assert movementCommandOnQueue != null && movementCommandOnQueue.equals(this.currentCommand);

        // Release the workflow for other commands.
        this.isCommandExecutorActive = false;
    }
}
