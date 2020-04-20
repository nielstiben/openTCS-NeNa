package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter.LoadState;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConflictConstants;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import org.opentcs.util.ExplainedBoolean;

import javax.annotation.Nonnull;
import java.util.Iterator;
import java.util.List;

/**
 * Library for meant validating incoming operations.
 */
public class OperationLib {
    private static LoadState lastKnownLoadState = null;

    /**
     * Check if a list of operations is allowed.
     *
     * @param operations      The operation list.
     * @param adapterInstance An instance of the current adapter.
     * @return Whether all operations in the list are allowed.
     */
    public static ExplainedBoolean areAllOperationsAllowed(
            @Nonnull List<String> operations,
            @Nonnull Ros2CommAdapter adapterInstance
    ) {
        lastKnownLoadState = adapterInstance.getLoadState();

        ExplainedBoolean isLastOperationAllowed = allowed();

        Iterator<String> operationIterator = operations.iterator();
        while (operationIterator.hasNext() && isLastOperationAllowed.getValue()) {
            final String nextOperation = operationIterator.next();

            isLastOperationAllowed = isOperationAllowed(nextOperation);
        }

        // All operations are checked, thus allowed.
        return isLastOperationAllowed;
    }

    /**
     * Check if a single operation is allowed.
     * @param operation the operation.
     * @return Whether the operation is allowed.
     */
    public static ExplainedBoolean isOperationAllowed(@Nonnull String operation) {
        switch (operation) {
            case OperationConstants.LOAD_CARGO:
                return isLoadCargoAllowed();
            case OperationConstants.UNLOAD_CARGO:
                return isUnloadCargoAllowed();
            default:
                return getUnkownOperation();
        }
    }

    private static ExplainedBoolean isLoadCargoAllowed() {
        boolean isVehicleLoaded = isVehicleLoadedByLoadState(lastKnownLoadState);

        if (isVehicleLoaded) {
            // Vehicle already loaded
            return notAllowed(OperationConflictConstants.LOAD_OPERATION_CONFLICT);
        } else {
            // Allowed
            lastKnownLoadState = LoadState.FULL; // After loading, our vehicle will be FULL.
            return allowed();
        }
    }

    private static ExplainedBoolean isUnloadCargoAllowed() {
        boolean isVehicleLoaded = isVehicleLoadedByLoadState(lastKnownLoadState);

        if (isVehicleLoaded) {
            // Allowed
            return allowed();
        } else {
            // Vehicle already unloaded
            lastKnownLoadState = LoadState.EMPTY; // After unloading, our vehicle will be EMPTY.
            return notAllowed(OperationConflictConstants.UNLOAD_OPERATION_CONFLICT);
        }
    }

    private static ExplainedBoolean getUnkownOperation() {
        return new ExplainedBoolean(false, "unknownOperation");
    }

    private static boolean isVehicleLoadedByLoadState(LoadState loadState) {

        return loadState == LoadState.FULL;
    }

    private static ExplainedBoolean allowed() {
        return new ExplainedBoolean(true, "");
    }

    private static ExplainedBoolean notAllowed(String reason) {
        return new ExplainedBoolean(false, reason);
    }
}
