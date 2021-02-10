/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter;

import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.Ros2CommAdapterTestLib;
import org.junit.Test;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.ExplainedBoolean;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.constants.NodeRunningStatus.NOT_ACTIVE;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.operation.constants.OperationConstants.LOAD_CARGO;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.operation.constants.OperationConstants.UNLOAD_CARGO;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.Ros2CommAdapterTestLib.*;

/**
 * Unit test to cover {@link Ros2CommAdapter}.
 *
 * @author Niels Tiben
 */
public class Ros2CommAdapterTest {
    @Test
    @SneakyThrows
    public void testRos2CommAdapterLifecycle(){
        Ros2CommAdapter adapter = Ros2CommAdapterTestLib.generateAdapterForTesting();

        assert !adapter.isInitialized();
        adapter.initialize();
        assert adapter.isInitialized();

        assert !adapter.isEnabled();
        adapter.enable();
        Thread.sleep(TIME_NEEDED_FOR_NODE_INITIALISATION);
        assert adapter.isEnabled();
        Thread.sleep(TIME_NEEDED_FOR_NODE_INITIALISATION);
        adapter.disable();
        assert !adapter.isEnabled();

        adapter.terminate();
        assert !adapter.isInitialized();
    }

    @Test
    public void testCreateTransferableProcessModel(){
        Ros2CommAdapter adapter = Ros2CommAdapterTestLib.generateAdapterForTesting();
        VehicleProcessModelTO vehicleProcessModelTO = adapter.createCustomTransferableProcessModel();

        assert vehicleProcessModelTO instanceof Ros2ProcessModelTO;
        assert ((Ros2ProcessModelTO) vehicleProcessModelTO).getNodeStatus().equals(NOT_ACTIVE.name());
    }

    @Test
    public void testCanProcessValidList(){
        Ros2CommAdapter adapter = Ros2CommAdapterTestLib.generateAdapterForTesting();
        List<String > operations = new ArrayList<>(Arrays.asList(LOAD_CARGO, UNLOAD_CARGO));
        ExplainedBoolean canProcess = adapter.canProcess(operations);
        assert canProcess.getValue();
    }

    @Test
    public void testCanProcessInvalidList(){
        Ros2CommAdapter adapter = Ros2CommAdapterTestLib.generateAdapterForTesting();
        List<String > operations = new ArrayList<>(Arrays.asList(UNLOAD_CARGO, UNLOAD_CARGO));
        ExplainedBoolean canProcess = adapter.canProcess(operations);
        assert !canProcess.getValue();
    }
}
