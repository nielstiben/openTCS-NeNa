package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.Ros2CommAdapterTestLib;
import org.junit.Test;
import org.opentcs.util.ExplainedBoolean;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConflictConstants.UNKNOWN_OPERATION;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants.LOAD_CARGO;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants.UNLOAD_CARGO;

/**
 * Unit test to cover {@link OperationAllowedLib}.
 *
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public class OperationAllowedLibTest {
    @Test
    public void testValidLoadOperationAllowed() {
        Ros2CommAdapter commAdapter = Ros2CommAdapterTestLib.generateAdapterForTesting();

        List<String> operationsToValidate = new ArrayList<>(Arrays.asList(LOAD_CARGO, UNLOAD_CARGO));
        ExplainedBoolean areAllOperationsAllowed = OperationAllowedLib
                .areAllOperationsAllowed(operationsToValidate, commAdapter);

        System.out.println(areAllOperationsAllowed.getReason());
        assert areAllOperationsAllowed.getValue();
    }

    @Test
    public void testValidLoadOperationForbidden() {
        Ros2CommAdapter commAdapter = Ros2CommAdapterTestLib.generateAdapterForTesting();

        // Unloading cargo before Loading cargo shouldn't be allowed.
        List<String> operationsToValidate = new ArrayList<>(Arrays.asList(UNLOAD_CARGO, LOAD_CARGO));
        ExplainedBoolean areAllOperationsAllowed = OperationAllowedLib
                .areAllOperationsAllowed(operationsToValidate, commAdapter);

        assert !areAllOperationsAllowed.getValue();
    }

    @Test
    public void testUnknownOperation(){
        Ros2CommAdapter commAdapter = Ros2CommAdapterTestLib.generateAdapterForTesting();

        List<String> operationsToValidate = new ArrayList<>(Arrays.asList(LOAD_CARGO, UNLOAD_CARGO, "Do a barrel roll"));
        ExplainedBoolean areAllOperationsAllowed = OperationAllowedLib
                .areAllOperationsAllowed(operationsToValidate, commAdapter);

        assert !areAllOperationsAllowed.getValue();
        assert areAllOperationsAllowed.getReason().equals(UNKNOWN_OPERATION);
    }
}
