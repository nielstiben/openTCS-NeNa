/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.transport_order;

import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2CommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.operation.constants.OperationConstants;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.PointTestLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.Ros2CommAdapterTestLib;
import org.junit.Test;
import org.opentcs.data.model.*;
import org.opentcs.data.order.Route;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.kernel.vehicles.MovementCommandImpl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.Ros2CommAdapterTestLib.*;

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
    public void testExecuteValidTransportOrder() {
        Ros2CommAdapter adapter = generateRos2CommAdapter();

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
        adapter.getSentQueue().add(dummyMovementCommand);
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

        // Stop Node
        adapter.getProcessModel().getNodeManager().stop();
    }

    @SneakyThrows
    @Test
    public void testExecuteUnreachableTransportOrder() {
        Ros2CommAdapter adapter = generateRos2CommAdapter();

        // Setup start and end point
        Point startPoint = PointTestLib.generatePointByNameAndCoordinate("TestPoint1", new Triple(0, 0, 0));
        Point destinationPoint = PointTestLib.generatePointByNameAndCoordinate("TestPointUnreachable", new Triple(-50, -50, 0));

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
        adapter.getSentQueue().add(dummyMovementCommand);
        adapter.enqueueCommand(dummyMovementCommand);
        workflow.processMovementCommand(dummyMovementCommand);

        // Vehicle should be busy now
        assert adapter.getProcessModel().getVehicleState().equals(Vehicle.State.EXECUTING);
        assert workflow.isCommandExecutorActive();

        // Fake that that the vehicle reached its destination.
        // There is no operation to process here, so the transport order should be finished.
        workflow.onNavigationGoalRejected(destinationPoint);
        assert adapter.getProcessModel().getVehicleState().equals(Vehicle.State.ERROR);
        assert !workflow.isCommandExecutorActive();

        // Stop Node
        adapter.getProcessModel().getNodeManager().stop();
        Thread.sleep(200);
    }


    //================================================================================
    // Helping methods
    //================================================================================
    @SneakyThrows
    private Ros2CommAdapter generateRos2CommAdapter(){
        // Setup vehicle
        Ros2CommAdapter adapter = Ros2CommAdapterTestLib.generateAdapterForTesting();
        adapter.enable();
        Thread.sleep(TIME_NEEDED_FOR_NODE_INITIALISATION); // Wait for the node to become active

        return adapter;
    }

    private MovementCommand generateDummyMovementCommand(Point startPoint, Point destinationPoint) {
        return new MovementCommandImpl(
                generateDummyRoute(startPoint, destinationPoint),
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

    private Route generateDummyRoute(Point startPoint, Point destinationPoint) {
        List<Route.Step> routeSteps = new ArrayList<>();
        routeSteps.add(generateDummyStep(startPoint, destinationPoint));
        return new Route(routeSteps, 0);
    }

    private Route.Step generateDummyStep(Point startPoint, Point destinationPoint) {
        return new Route.Step(
                generatePath(startPoint, destinationPoint),
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

    private Path generatePath(Point startPoint, Point destinationPoint) {
        return new Path("test_path", startPoint.getReference(), destinationPoint.getReference());
    }
}
