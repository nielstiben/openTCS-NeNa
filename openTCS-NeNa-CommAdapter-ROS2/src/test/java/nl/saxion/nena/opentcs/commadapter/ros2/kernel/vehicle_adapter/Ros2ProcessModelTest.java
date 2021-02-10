/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter;

import action_msgs.msg.dds.GoalStatusArray;
import geometry_msgs.msg.dds.PoseWithCovarianceStamped;
import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.OutgoingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.ScaleCorrector;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.NavigationGoalTestLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.point.CoordinatePoint;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.PointTestLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.Ros2CommAdapterTestLib;
import org.junit.Test;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import us.ihmc.euclid.tuple4D.Quaternion;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.Ros2CommAdapterTestLib.*;

/**
 * Unit test to cover {@link Ros2ProcessModel}.
 *
 * @author Niels Tiben
 */
public class Ros2ProcessModelTest {
    private final static byte NAV_ACTIVE = 2;

    //================================================================================
    // Tests
    //================================================================================

    @Test
    @SneakyThrows
    public void testInitiateRos2ProcessModelInstance() {
        Vehicle vehicle = new Vehicle("test_vehicle");

        // Create instance
        Ros2ProcessModel ros2ProcessModel = new Ros2ProcessModel(vehicle);
        assert ros2ProcessModel.getVehicleState().equals(Vehicle.State.UNKNOWN);

        // Enable process model
        ros2ProcessModel.setNamespace(DEFAULT_TESTING_NAMESPACE);
        ros2ProcessModel.setDomainId(DEFAULT_TESTING_DOMAIN_ID);
        ros2ProcessModel.onDriverEnable();

//        assert ros2ProcessModel.getVehicleState().equals(Vehicle.State.UNAVAILABLE);
    }

    @Test
    public void testNewAmclPose() {
        Ros2ProcessModel processModel = generateRos2ProcessModelEnabled(0);

        // All position variables should be unknown, because the vehicle never sent its amclPose.
        assert processModel.getVehiclePrecisePosition() == null;
        assert processModel.getVehiclePosition() == null;

        // Supply process model with a new amclPose, which contains the position and orientation of ros2 vehicle.
        PoseWithCovarianceStamped amclPoseMessage = generateDummyAmclPose();
        processModel.onNewAmclPose(amclPoseMessage);

        // The precise position and orientation should now be known.
        assert processModel.getVehiclePrecisePosition() != null;

        // The dummy amclPose should contain an orientation towards the west.
        double orientationAngleTowardsWest = 180;
        assert processModel.getVehicleOrientationAngle() == orientationAngleTowardsWest;

        // Position should still be unknown, because the amclPose is not (necessarily) a point on the Plant Model.
        assert processModel.getVehiclePosition() == null;
    }

    @Test
    public void testNewGoalStatusArray() {
        Ros2ProcessModel processModel = generateRos2ProcessModelEnabled(1);

        // Generate GoalStatusArray with one entry
        GoalStatusArray goalStatusArray = NavigationGoalTestLib.createDummyGoalStatusArraySingleEntry(NAV_ACTIVE);

        // Update process model
        assert processModel.getNavigationGoalTracker().toStringTable() == null;
        processModel.onNewGoalStatusArray(goalStatusArray);
        assert processModel.getNavigationGoalTracker().toStringTable() != null;
    }

    @Test
    @SneakyThrows
    public void testDispatchToPoint() {
        Ros2ProcessModel processModel = generateRos2ProcessModelEnabled(1);
        Thread.sleep(TIME_NEEDED_FOR_NODE_INITIALISATION);

        processModel.dispatchToPoint(new CoordinatePoint(new Triple(0,0,0)));
        processModel.onDriverDisable();
    }

    @Test
    @SneakyThrows
    public void testDispatchToCoordinate() {
        Ros2ProcessModel processModel = generateRos2ProcessModelEnabled(1);
        Thread.sleep(TIME_NEEDED_FOR_NODE_INITIALISATION);

        processModel.dispatchToCoordinate(new Triple(1,1,0));
        processModel.onDriverDisable();
    }

    //================================================================================
    // Helper methods
    //================================================================================

    private Ros2ProcessModel generateRos2ProcessModelEnabled(int index) {
        ScaleCorrector.getInstance().setScale(1);
        Vehicle vehicle = new Vehicle(String.format("test_vehicle_%d", index));
        Ros2ProcessModel ros2ProcessModel = new Ros2ProcessModel(vehicle);
        ros2ProcessModel.setDomainId(Ros2CommAdapterTestLib.DEFAULT_TESTING_DOMAIN_ID);
        ros2ProcessModel.setNamespace(DEFAULT_TESTING_NAMESPACE);
        ros2ProcessModel.onDriverEnable();

        return ros2ProcessModel;
    }

    private PoseWithCovarianceStamped generateDummyAmclPose() {
        Point pointForAmclPose = PointTestLib.generatePointByNameAndCoordinate("TestPoint1", new Triple(0, 0, 0));
        // The message type for 'outgoing' initial pose message, is exactly the same as the 'incoming'
        // amcl pose message, So we use the OutGoingMessageLib for creating a dummy incoming amcl message.
        PoseWithCovarianceStamped pose = OutgoingMessageLib.generateInitialPoseMessageByPoint(pointForAmclPose);

        // Set orientation too.
        pose.getPose().getPose().getOrientation().set(getQuaternionTowardsWest());

        return pose;
    }

    private Quaternion getQuaternionTowardsWest() {
        return new Quaternion(0,0,1,0);
    }
}
