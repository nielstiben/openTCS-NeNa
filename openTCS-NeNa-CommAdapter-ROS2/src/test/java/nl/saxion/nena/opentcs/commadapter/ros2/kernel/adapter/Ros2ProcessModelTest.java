package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.PoseWithCovarianceStamped;
import geometry_msgs.msg.Quaternion;
import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.OutgoingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.test_library.NavigationGoalTestLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.point.CoordinatePoint;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.PointTestLib;
import org.junit.Test;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.Ros2CommAdapterTestLib.DEFAULT_TESTING_NAMESPACE;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.Ros2CommAdapterTestLib.TIME_NEEDED_FOR_NODE_INITIALISATION;

/**
 * Unit test to cover {@link Ros2ProcessModel}.
 *
 * @author Niels Tiben
 */
public class Ros2ProcessModelTest {

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
        byte NAV_ACTIVE = 2;
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
        Vehicle vehicle = new Vehicle(String.format("test_vehicle_%d", index));
        Ros2ProcessModel ros2ProcessModel = new Ros2ProcessModel(vehicle);
        ros2ProcessModel.setNamespace("");
        ros2ProcessModel.onDriverEnable();

        return ros2ProcessModel;
    }

    private PoseWithCovarianceStamped generateDummyAmclPose() {
        Point pointForAmclPose = PointTestLib.generatePointByNameAndCoordinate("TestPoint1", new Triple(0, 0, 0));
        // The message type for 'outgoing' initial pose message, is exactly the same as the 'incoming'
        // amcl pose message, So we use the OutGoingMessageLib for creating a dummy incoming amcl message.
        PoseWithCovarianceStamped pose = OutgoingMessageLib.generateInitialPoseMessageByPoint(pointForAmclPose);

        // Set orientation too.
        pose.getPose().getPose().setOrientation(getQuaternionTowardsWest());

        return pose;
    }

    private Quaternion getQuaternionTowardsWest() {
        return new Quaternion().setX(0).setY(0).setZ(1).setW(0);
    }
}
