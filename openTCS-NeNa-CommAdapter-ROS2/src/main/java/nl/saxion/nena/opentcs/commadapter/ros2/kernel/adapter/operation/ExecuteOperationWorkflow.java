package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation;

import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.Node;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;

import javax.annotation.Nonnull;
import java.util.Collections;

/**
 * A workflow class used to sequentially execute a operation on a vehicle.
 * Each function in this class belongs to a certain step in the workflow of executing an operation.
 * TODO: Create a sequence diagram for this workflow.
 *
 * @author Niels Tiben
 */
public class ExecuteOperationWorkflow {
    public static final String LOAD_HANDLING_DEVICE_NAME = "nena-lhd";
    private final Ros2ProcessModel processModel;
    private final OperationExecutorListener operationExecutorListener;

    //================================================================================
    // Workflow variables
    //================================================================================

    private boolean isOperationBeingExecuted = false;

    //================================================================================
    // 0: Construct and enable
    //================================================================================

    public ExecuteOperationWorkflow(Ros2ProcessModel processModel, OperationExecutorListener operationExecutorListener) {
        this.processModel = processModel;
        this.operationExecutorListener = operationExecutorListener;
    }
    // Next step (1) is activated by parent workflow.

    //================================================================================
    // 1: Define which operation to execute.
    //================================================================================

    @SneakyThrows
    public void executeOperationByName(@Nonnull String actionName) {
        // Only one action at a time can be executed.
        assert !this.isOperationBeingExecuted;

        switch (actionName) {
            case OperationConstants.LOAD_CARGO:
                executeLoadCargo();
                break;
            case OperationConstants.UNLOAD_CARGO:
                executeUnloadCargo();
                break;
            default:
                String reason = String.format("Action not implemented in %s", this.getClass().getName());
                onOperationExecutionFailed(reason);
        }
    }
    // Next step (2) is activated by step 1.

    //================================================================================
    // 2: Execute the operation.
    //================================================================================

    /* --------------- 2a: Execute load cargo ---------------*/
    private void executeLoadCargo() {
        Node opentcsNode = this.processModel.getNodeManager().getNode();
        System.out.println("LOADING CARGO...");

        // TODO: Implement executor for LOAD CARGO operation.

        // Mark the vehicle as loaded.
        this.processModel.setVehicleLoadHandlingDevices(Collections.singletonList(new LoadHandlingDevice(LOAD_HANDLING_DEVICE_NAME, true)));
        onOperationExecutionFinished(); // Fake that the operation was successful.
    }

    /* --------------- 2b: Execute unload cargo ---------------*/
    private void executeUnloadCargo() {
        Node opentcsNode = this.processModel.getNodeManager().getNode();
        System.out.println("UNLOADING CARGO...");

        // TODO: Implement executor for UNLOAD CARGO operation.

        // Mark the vehicle as unloaded
        this.processModel.setVehicleLoadHandlingDevices(Collections.singletonList(new LoadHandlingDevice(LOAD_HANDLING_DEVICE_NAME, false)));
        onOperationExecutionFinished(); // Fake that the operation was successful.

    }
    // Next step (3) is activated by callback

    //================================================================================
    // 3: Handle operation feedback.
    //================================================================================

    /* --------------- 3a: Handle load cargo feedback ---------------*/
    public void onExecuteLoadCargoFeedback() {
        // TODO: Implement feedback handler for LOAD CARGO operation.
        // Success
        onOperationExecutionFinished();

        // Failed
        //onOperationExecutionFailed("our vehicle finds the load too heavy");
    }

    /* --------------- 3b: Handle unload cargo feedback ---------------*/
    public void onExecuteUnloadCargoFeedback() {
        // TODO: Implement feedback handler for UNLOAD CARGO operation.
        // Success
        onOperationExecutionFinished();

        // Failed
        //onOperationExecutionFailed("our vehicle cannot find a nice spot to drop the load");
    }
    // Next step (4) is activated by step 3.

    //================================================================================
    // 4: Callback to parent workflow (CommandWorkflow)
    //================================================================================

    /* --------------- 4a: On succeeded ---------------*/
    private void onOperationExecutionFinished() {
        this.isOperationBeingExecuted = false;
        this.operationExecutorListener.onOperationExecutionSucceeded();
    }

    /* --------------- 4b: On failure ---------------*/
    private void onOperationExecutionFailed(String reason) {
        this.isOperationBeingExecuted = false;
        this.operationExecutorListener.onOperationExecutionFailed(reason);
    }
}
