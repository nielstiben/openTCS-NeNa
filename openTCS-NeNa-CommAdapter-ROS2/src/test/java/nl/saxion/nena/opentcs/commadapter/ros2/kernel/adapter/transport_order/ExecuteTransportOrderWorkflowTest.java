package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.transport_order;

import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.PointTestLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.Ros2CommAdapterTestLib;
import org.junit.Test;
import org.opentcs.data.model.*;
import org.opentcs.data.order.Route;
import org.opentcs.drivers.vehicle.MovementCommand;

import java.util.HashMap;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.Ros2CommAdapterTestLib.DEFAULT_TESTING_NAMESPACE;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.Ros2CommAdapterTestLib.TIME_NEEDED_FOR_NODE_INITIALISATION;

/**
 * Unit test to cover {@link ExecuteTransportOrderWorkflow}.
 * This test is creates an instance of the vehicle adapter.
 * The vehicle states are validated at each step of the transport order execution.
 *
 * @author Niels Tiben
 */
public class ExecuteTransportOrderWorkflowTest {

    //================================================================================
    // Tests
    //================================================================================

    @Test
    @SneakyThrows
    public void testExecuteValidTransportOrder() {
        // Setup vehicle
        Ros2CommAdapter adapter = Ros2CommAdapterTestLib.generateAdapterForTesting();
        adapter.getProcessModel().setNamespace(DEFAULT_TESTING_NAMESPACE);
        adapter.enable();
        Thread.sleep(TIME_NEEDED_FOR_NODE_INITIALISATION); // Wait for the node to become active

        // Setup start and end point
        Point startPoint = PointTestLib.generatePointByNameAndCoordinate("TestPoint1", new Triple(0, 0, 0));
        Point destinationPoint = PointTestLib.generatePointByNameAndCoordinate("TestPoint2", new Triple(1, 1, 0));

        // Fake that the vehicle has reached the start location, so it's available to receive transport orders.
        adapter.getProcessModel().onNavigationGoalSucceeded(startPoint);

        // Initiate workflow
        ExecuteTransportOrderWorkflow workflow = new ExecuteTransportOrderWorkflow(
                adapter.getProcessModel(),
                adapter.getSentQueue()
        );

        // Assert the vehicle is idling.
        assert adapter.getProcessModel().getVehicleState().equals(Vehicle.State.IDLE);
        assert !workflow.isCommandExecutorActive();

        // Start workflow
        MovementCommand dummyMovementCommand = generateDummyMovementCommand(startPoint, destinationPoint);
        adapter.enqueueCommand(dummyMovementCommand);
        workflow.processMovementCommand(dummyMovementCommand);

        // Vehicle should be busy now
        assert adapter.getProcessModel().getVehicleState().equals(Vehicle.State.EXECUTING);
        assert workflow.isCommandExecutorActive();

        // Fake that that the vehicle reached its destination.
        // There is no operation to process here, so the transport order should be finished.
        workflow.onNavigationGoalSucceeded(destinationPoint);
        assert adapter.getProcessModel().getVehicleState().equals(Vehicle.State.IDLE);
        assert !workflow.isCommandExecutorActive();
    }

    //================================================================================
    // Helping methods
    //================================================================================

    private MovementCommand generateDummyMovementCommand(Point startPoint, Point destinationPoint) {
        return new MovementCommand(
                generateDummyStep(startPoint, destinationPoint),
                OperationConstants.NOP,
                generateLocationByPoint(0),
                true,
                generateLocationByPoint(1),
                destinationPoint,
                OperationConstants.NOP,
                new HashMap<>()
        );
    }

    private Route.Step generateDummyStep(Point startPoint, Point destinationPoint) {
        Path path = new Path("test_path", startPoint.getReference(), destinationPoint.getReference());

        return new Route.Step(
                path,
                startPoint,
                destinationPoint,
                Vehicle.Orientation.FORWARD,
                0
        );
    }

    private Location generateLocationByPoint(int index) {
        LocationType locationType = new LocationType("test_location_type");
        String locationName = String.format("test_location_%d", index);

        return new Location(locationName, locationType.getReference());
    }
}
