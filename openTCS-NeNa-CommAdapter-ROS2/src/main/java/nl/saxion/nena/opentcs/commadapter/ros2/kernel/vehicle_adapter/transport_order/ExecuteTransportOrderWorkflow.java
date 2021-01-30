package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.transport_order;

import geometry_msgs.msg.dds.PoseStamped;
import lombok.Getter;
import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.operation.ExecuteOperationWorkflow;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.operation.OperationExecutorListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.OutgoingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.NavigationGoalListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.NavigationGoalTracker;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;
import java.util.Queue;

/**
 * A workflow class used to sequentially execute a Transport Order.
 * Each function in this class belongs to a certain step in the workflow of executing a Transport Order.
 * TODO: Create a Activity diagram for this workflow.
 *
 * @author Niels Tiben
 */
public class ExecuteTransportOrderWorkflow implements NavigationGoalListener, OperationExecutorListener {
    private static final Logger LOG = LoggerFactory.getLogger(ExecuteTransportOrderWorkflow.class);
    private final Ros2ProcessModel processModelInstance;
    private final Queue<MovementCommand> sentQueue;

    //================================================================================
    // Workflow variables
    //================================================================================

    @Getter
    private boolean isCommandExecutorActive = false;
    @Getter
    private final ExecuteOperationWorkflow executeOperationWorkflow;
    private MovementCommand currentCommand;

    //================================================================================
    // 0: Construct and enable
    //================================================================================

    public ExecuteTransportOrderWorkflow(
            @Nonnull Ros2ProcessModel processModelInstance,
            @Nonnull Queue<MovementCommand> sentQueue
    ) {
        this.processModelInstance = processModelInstance;
        this.sentQueue = sentQueue;
        this.executeOperationWorkflow = new ExecuteOperationWorkflow(this.processModelInstance, this);
    }

    public void enableNavigationGoalListener() {
        NavigationGoalTracker goalTracker = this.processModelInstance.getNavigationGoalTracker();

        assert goalTracker != null;

        goalTracker.setCommandExecutorListener(this);
        this.processModelInstance.setExecuteTransportOrderWorkflow(this);
    }
    // Next step (1) is activated on demand (when there is a new transport order).

    //================================================================================
    // 1: Initiate Movement Command.
    //================================================================================

    @SneakyThrows
    public void processMovementCommand(MovementCommand movementCommand) {
        assert !this.isCommandExecutorActive;
        this.currentCommand = movementCommand;

        // Set active label
        this.isCommandExecutorActive = true;

        // Set vehicle state
        this.processModelInstance.setVehicleState(Vehicle.State.EXECUTING);

        // Get Destination
        assert this.currentCommand != null;
        Point destinationPoint = this.currentCommand.getStep().getDestinationPoint();

        // Generate message.
        LOG.info("Command Execution: Dispatching vehicle to point '{}'", destinationPoint.getName());
        PoseStamped message = OutgoingMessageLib.generateScaledNavigationMessageByPoint(destinationPoint);

        // Notify NavigationGoalTracker that we expect a new goal.
        this.processModelInstance.getNavigationGoalTracker().setDestinationPointIncomingGoal(destinationPoint);

        // Give dispatch order.
        this.processModelInstance.getNodeManager().getNode().getGoalPublisher().publish(message);
    }
    // Next step (2) is activated by a callback.

    //================================================================================
    // 2: Received callback from NavigationGoalTracker that Movement Command has been effectively picked up.
    //================================================================================

    @Override
    public void onNavigationGoalActive(@Nonnull Point incomingDestination) {
        // Callback received that a new navigation goal has been activated.

        if (this.isCommandExecutorActive) {
            // Let's check if its point equals our destination point.
            Point currentDestination = this.currentCommand.getStep().getDestinationPoint();
            assert currentDestination.equals(incomingDestination);
        } else {
            // Not intended for CommandExecutor so we don't care, just proceed.
        }
    }
    // Next step (3) is activated by callback.

    //================================================================================
    // 3: Received callback from NavigationGoalTracker that Movement Command has been finished.
    //================================================================================

    @Override
    public void onNavigationGoalSucceeded(@Nonnull Point arrivedPoint) {
        if (this.isCommandExecutorActive) {
            Point currentDestination = this.currentCommand.getStep().getDestinationPoint();

            // Callback that a new navigation goal has been activated: let's check if its point equals our destination point.
            if (currentDestination.equals(arrivedPoint)) {
                // Vehicle reached its intended destination
                LOG.info("Processing MovementCommand Succeeded: Vehicle reached its intended destination! =>" + currentDestination.toString());
                executeOperationIfNeeded();
            } else {
                // Vehicle did not reach its intended destination, but a different destination.
                String reason = String.format("Processing MovementCommand Failed: Vehicle did not reach its intended destination, but a different destination ('%s' instead of '%s')", arrivedPoint.toString(), currentDestination.toString());
                onOperationExecutionFailed(reason);
            }
        } else {
            // Not intended for CommandExecutor so we don't care, just proceed.
        }

    }
    // Next step (4) is activated by step 3 on success.

    //================================================================================
    // 4: Execute Operation (if needed)
    //================================================================================

    @SneakyThrows
    private void executeOperationIfNeeded() {
        if (this.currentCommand.isWithoutOperation()) {
            // No operation in this command => skip this step.
            setCommandWorkflowSucceeded();
        } else {
            this.executeOperationWorkflow.executeOperationByName(this.currentCommand.getOperation());
        }
    }
    // Next step (5) is activated by callback. Step 5 is skipped when there are no operations.

    //================================================================================
    // 5: Received feedback that operation has been finished (if needed)
    //================================================================================

    /* --------------- 5a: Operation has succeeded ---------------*/
    @Override
    public void onOperationExecutionSucceeded() {
        LOG.info("Operation succeeded!");
        setCommandWorkflowSucceeded();
    }

    /* --------------- 5b: Operation has failed ---------------*/
    @Override
    public void onOperationExecutionFailed(String reason) {
        LOG.info(String.format("Operation failed: %s", reason));
        setCommandWorkflowFailed();
    }
    // Next step (6) is activated by step 5a..

    //================================================================================
    // 6: Succeed Transport Order Workflow
    //================================================================================

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

    private boolean hasVehicleReachedFinalDestination() {
        Point finalDestination = this.currentCommand.getFinalDestination();
        Point currentStepDestination = this.currentCommand.getStep().getDestinationPoint();

        return currentStepDestination.equals(finalDestination);
    }

    //================================================================================
    // F: On Transport Order Workflow Failed
    //================================================================================
    private void setCommandWorkflowFailed() {
        removeExecutedCommandFromSentQueueAndSetWorkflowInactive();

        this.processModelInstance.commandFailed(this.currentCommand);
        this.processModelInstance.setVehicleState(Vehicle.State.ERROR);
    }

    //================================================================================
    // Finally
    //================================================================================
    private void removeExecutedCommandFromSentQueueAndSetWorkflowInactive() {
        // Remove current command from the queue
        MovementCommand movementCommandOnQueue = this.sentQueue.poll();

        assert movementCommandOnQueue != null;
        assert movementCommandOnQueue.equals(this.currentCommand);

        // Release the workflow for other commands.
        this.isCommandExecutorActive = false;
    }
}
