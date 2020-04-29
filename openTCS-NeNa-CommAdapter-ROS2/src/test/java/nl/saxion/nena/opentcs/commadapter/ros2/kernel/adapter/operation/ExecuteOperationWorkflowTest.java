package nl.saxion.nena.opentcs.commadapter.ros2.unit.transport_order;

import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.ExecuteOperationWorkflow;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.OperationExecutorListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import nl.saxion.nena.opentcs.commadapter.ros2.unit.transport_order.test_library.Ros2CommAdapterTestLib;
import org.junit.Before;
import org.junit.Test;

/**
 * Unit test to cover {@link ExecuteOperationWorkflow}.
 *
 * @author Niels Tiben
 */
public class ExecuteOperationWorkflowTest {
    private OperationExecutorListener operationExecutorListener;

    //================================================================================
    // Callback variables.
    //================================================================================
    boolean isOperationExecutionSucceededCallbackCalled = false;
    boolean isOperationExecutionFailedCallbackCalled = false;

    //================================================================================
    // Pre operation
    //================================================================================

    @Before
    public void setupListener() {
        this.operationExecutorListener = new OperationExecutorListener() {
            @Override
            public void onOperationExecutionSucceeded() {
                isOperationExecutionSucceededCallbackCalled = true;
            }

            @Override
            public void onOperationExecutionFailed(String reason) {
                isOperationExecutionFailedCallbackCalled = true;
            }
        };
    }

    //================================================================================
    // Tests
    //================================================================================

    @Test
    @SneakyThrows
    public void testValidLoadOperationWorkflowLoadCargo() {
        Ros2CommAdapter adapter = Ros2CommAdapterTestLib.generateAdapterForTesting();
        ExecuteOperationWorkflow workflow = new ExecuteOperationWorkflow(
                adapter.getProcessModel(),
                this.operationExecutorListener
        );
        workflow.executeOperationByName(OperationConstants.LOAD_CARGO);
        Thread.sleep(1); // The maximum time an operation may take.

        // By now, we should have received feedback that our task has succeeded
        assert this.isOperationExecutionSucceededCallbackCalled;
        assert !this.isOperationExecutionFailedCallbackCalled;
    }

    @Test
    @SneakyThrows
    public void testValidLoadOperationWorkflowUnloadCargo() {
        Ros2CommAdapter adapter = Ros2CommAdapterTestLib.generateAdapterForTesting();
        ExecuteOperationWorkflow workflow = new ExecuteOperationWorkflow(
                adapter.getProcessModel(),
                this.operationExecutorListener
        );
        workflow.executeOperationByName(OperationConstants.UNLOAD_CARGO);
        Thread.sleep(1); // The maximum time an operation may take.

        // By now, we should have received feedback that our task has succeeded
        assert this.isOperationExecutionSucceededCallbackCalled;
        assert !this.isOperationExecutionFailedCallbackCalled;
    }

}
