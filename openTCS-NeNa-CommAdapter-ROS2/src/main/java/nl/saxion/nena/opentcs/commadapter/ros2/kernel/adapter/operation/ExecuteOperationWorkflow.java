package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation;

import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.Node;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import org.opentcs.drivers.vehicle.LoadHandlingDevice;

import javax.annotation.Nonnull;
import java.util.Collections;


/**
 * @author Niels Tiben
 */
public class ExecuteOperationWorkflow {
    public static final String LOAD_HANDLING_DEVICE_NAME = "nena-lhd";

    private boolean isOperationBeingExecuted = false;
    private final Ros2ProcessModel processModel;
    private final OperationExecutorListener operationExecutorListener;
    private Node opentcsNode;

    /* --------------- 1: Construct / Start ---------------*/

    public ExecuteOperationWorkflow(Ros2ProcessModel processModel, OperationExecutorListener operationExecutorListener) {
        this.processModel = processModel;
        this.operationExecutorListener = operationExecutorListener;
    }

    @SneakyThrows
    public void executeActionByName(@Nonnull String actionName) {
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

    /* --------------- 2: Execute operation ---------------*/

    private void executeLoadCargo() {
        Node opentcsNode = processModel.getNodeManager().getNode();

        System.out.println("LOADING CARGO...");
        // TODO: Implement executor for LOAD CARGO operation.

        // Set as full
        processModel.setVehicleLoadHandlingDevices(
                Collections.singletonList(new LoadHandlingDevice(LOAD_HANDLING_DEVICE_NAME, true)));

        // Fake that the operation was successful.
        onOperationExecutionFinished();
    }

    private void executeUnloadCargo() {
        // TODO: Implement executor for UNLOAD CARGO operation.
        Node opentcsNode = processModel.getNodeManager().getNode();
        System.out.println("UNLOADING CARGO...");

        // Set as not-full
        processModel.setVehicleLoadHandlingDevices(
                Collections.singletonList(new LoadHandlingDevice(LOAD_HANDLING_DEVICE_NAME, false)));

        // Fake that the operation was successful.
        onOperationExecutionFinished();
    }

    /* --------------- 3: Handle operation feedback ---------------*/
    public void onExecuteLoadCargoFeedback() {
        // TODO: Implement feedback handler for LOAD CARGO operation.
        // Success
        onOperationExecutionFinished();

        // Failed
        //onOperationExecutionFailed("our vehicle finds the load too heavy");
    }

    public void onExecuteUnloadCargoFeedback() {
        // TODO: Implement feedback handler for UNLOAD CARGO operation.
        // Success
        onOperationExecutionFinished();

        // Failed
        //onOperationExecutionFailed("our vehicle cannot find a nice spot to drop the load");
    }

    /* --------------- 4: Callback to CommandWorkflow ---------------*/

    private void onOperationExecutionFinished() {
        this.isOperationBeingExecuted = false;
        this.operationExecutorListener.onOperationExecutionSucceeded();
    }

    private void onOperationExecutionFailed(String reason) {
        this.isOperationBeingExecuted = false;
        this.operationExecutorListener.onOperationExecutionFailed(reason);
    }
}
