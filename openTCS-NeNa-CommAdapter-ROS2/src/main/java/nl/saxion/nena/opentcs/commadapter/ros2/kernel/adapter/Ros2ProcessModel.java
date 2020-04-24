package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.PoseStamped;
import geometry_msgs.msg.PoseWithCovarianceStamped;
import geometry_msgs.msg.Quaternion;
import lombok.Getter;
import lombok.Setter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.NodeManager;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.NodeMessageListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.NodeRunningStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.NodeRunningStatusListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.IncomingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.OutgoingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.UnitConverterLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.NavigationGoalListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.NavigationGoalTracker;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.ExecuteOperationWorkflow;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.point.CoordinatePoint;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;

/**
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public class Ros2ProcessModel extends VehicleProcessModel implements
        NodeRunningStatusListener,
        NodeMessageListener,
        NavigationGoalListener {
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapter.class);
    private final String loadOperation;
    private final String unloadOperation;

    @Setter
    private String namespace = "";
    @Getter
    private NavigationGoalTracker navigationGoalTracker;
    @Getter
    private NodeManager nodeManager;
    @Getter
    private int operatingTime;
    @Getter
    private Triple estimatePosition;
    @Setter
    private ExecuteOperationWorkflow executeOperationWorkflow;

    public Ros2ProcessModel(Vehicle attachedVehicle) {
        super(attachedVehicle);
        this.loadOperation = extractLoadOperation(attachedVehicle);
        this.unloadOperation = extractUnloadOperation(attachedVehicle);
        this.operatingTime = parseOperatingTime(attachedVehicle);
//        this.navigationGoalTracker = new NavigationGoalTracker(this); // TODO: do we want to construct here??


        this.nodeManager = new NodeManager();
    }

    /* --------------- Navigation ---------------*/

    /**
     * Sends the initial position to help a ROS2 node finding its current location on the map.
     *
     * @param x The X coordinate in meters.
     * @param y The Y coordinate in meters.
     */
    public void setInitialPosition(double x, double y) {
        final PoseWithCovarianceStamped message = OutgoingMessageLib.generateInitialPoseMessageByCoordinate(x, y);
        // Todo: set current location too.
        nodeManager.getOpentcsNode().getInitialPosePublisher().publish(message);
    }

    /**
     * Dispatches the connected ROS2 node to the given coordinates.
     *
     * @param coordinate The coordinates.
     */
    public void dispatchToCoordinate(@Nonnull Triple coordinate) {
        final CoordinatePoint coordinatePoint = new CoordinatePoint(coordinate);
        dispatchToPoint(coordinatePoint);
    }

    /**
     * Dispatches the connected ROS2 node to the given point.
     *
     * @param point The point containing the physical location of the destination.
     */
    public void dispatchToPoint(@Nonnull Point point) {
        LOG.info("Dispatching vehicle to point '{}'", point.getName());
        PoseStamped message = OutgoingMessageLib.generateScaledNavigationMessageByPoint(point);

        this.navigationGoalTracker.setDestinationPointIncomingGoal(point); // Notify NavigationGoalTracker that we expect a new goal
        this.nodeManager.getOpentcsNode().getGoalPublisher().publish(message);
    }

    /* --------------- Operation ---------------*/
    private static String extractLoadOperation(Vehicle attachedVehicle) {
        return OperationConstants.LOAD_CARGO;
    }

    private static String extractUnloadOperation(Vehicle attachedVehicle) {
        return OperationConstants.UNLOAD_CARGO;
    }

    public String getLoadOperation() {
        return loadOperation;
    }

    public String getUnloadOperation() {
        return unloadOperation;
    }

    /* --------------- Misc ---------------*/

    public synchronized void setOperatingTime(int defaultOperatingTime) {
        //TODO: Customize

        int oldValue = this.operatingTime;
        this.operatingTime = defaultOperatingTime;


        getPropertyChangeSupport().firePropertyChange(Attribute.OPERATING_TIME.name(),
                oldValue,
                defaultOperatingTime);
    }

    private int parseOperatingTime(Vehicle vehicle) {
        //TODO: Customize
        return 1;
    }

    public String[][] parseNavigationGoalTable() {
        // todo: fix code smell
        if (navigationGoalTracker == null) {
            return null;
        } else {
            return navigationGoalTracker.toStringTable();
        }
    }

    /* --------------- Enable / Disable ---------------*/
    public void onDriverEnable() {
        nodeManager.start(this, this, this.namespace);
        this.navigationGoalTracker = new NavigationGoalTracker(this); // Start navigation goal tracker

    }

    public void onDriverDisable() {
        this.nodeManager.stop();
        this.navigationGoalTracker = null;
        getPropertyChangeSupport().firePropertyChange(Attribute.NAVIGATION_GOALS.name(), null, null);
    }

    /* --------------- Node message callback methods ---------------*/
    @Override
    public void onNewGoalStatusArray(GoalStatusArray goalStatusArray) {
        Object[][] oldValue = navigationGoalTracker.toStringTable();
        navigationGoalTracker.updateByGoalStatusArray(goalStatusArray);
        Object[][] newValue = navigationGoalTracker.toStringTable();

        getPropertyChangeSupport().firePropertyChange(Attribute.NAVIGATION_GOALS.name(), oldValue, newValue);
    }

    /**
     * Update Estimate Position
     *
     * @param amclPose
     */
    @Override
    public void onNewAmclPose(PoseWithCovarianceStamped amclPose) {
        Triple oldEstimatePosition = this.estimatePosition;
        this.estimatePosition = IncomingMessageLib.generateTripleByAmclPose(amclPose);

        // Set precise position
        setVehiclePrecisePosition(this.estimatePosition);
        getPropertyChangeSupport().firePropertyChange(Attribute.POSITION_ESTIMATE.name(), oldEstimatePosition, this.estimatePosition);

        // Set orientation angle
        Quaternion orientationQuaternion = amclPose.getPose().getPose().getOrientation();
        double orientationDegrees = UnitConverterLib.quaternionToAngleDegree(orientationQuaternion);
        setVehicleOrientationAngle(orientationDegrees);
    }

    /* Operations */
    @Override
    public void onOperationLoadCargoFeedback(String feedback) {
        this.executeOperationWorkflow.onExecuteLoadCargoFeedback();
    }

    @Override
    public void onOperationUnloadCargoFeedback(String feedback) {
        this.executeOperationWorkflow.onExecuteUnloadCargoFeedback();
    }

    @Override
    public void onNodeStatusChange(NodeRunningStatus newNodeRunningStatus) {
        getPropertyChangeSupport().firePropertyChange(Attribute.NODE_STATUS.name(), null, newNodeRunningStatus);
    }

    /**
     * Update Precise position
     *
     * @param point
     */
    @Override
    public void onNavigationGoalSucceeded(Point point) {
        if (point instanceof CoordinatePoint) {
            // The vehicle reached a coordinate point.
            // Since this is a fictional point, the vehicle position should not be modified.
        } else {
            setVehiclePosition(point.getName());
            setVehiclePrecisePosition(point.getPosition());
        }
    }

    @Override
    public void onNavigationGoalActive(Point point) {
        // Todo:
    }

    /**
     * Notification arguments to indicate some change.
     */
    public enum Attribute {
        OPERATING_TIME,
        NODE_STATUS,
        NAVIGATION_GOALS,
        POSITION_ESTIMATE,
    }
}
