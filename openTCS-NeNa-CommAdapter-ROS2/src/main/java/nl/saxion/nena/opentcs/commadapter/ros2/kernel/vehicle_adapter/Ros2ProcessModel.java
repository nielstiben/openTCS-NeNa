package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.PoseStamped;
import geometry_msgs.msg.PoseWithCovarianceStamped;
import geometry_msgs.msg.Quaternion;
import lombok.Getter;
import lombok.Setter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.NodeManager;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.NodeMessageListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.constants.NodeRunningStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.NodeRunningStatusListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.IncomingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.OutgoingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library.UnitConverterLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.ExternalNavigationGoalListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.NavigationGoalListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.NavigationGoalTracker;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.operation.ExecuteOperationWorkflow;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.point.CoordinatePoint;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.transport_order.ExecuteTransportOrderWorkflow;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.annotation.Nonnull;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2ProcessModelAttribute.*;

/**
 * Class for holding data and functionality regarding the connected ROS 2 vehicle.
 * An instance of the {@link Ros2CommAdapter} holds one instance of the {@link Ros2ProcessModel}.
 *
 * @author Niels Tiben
 */
public class Ros2ProcessModel extends VehicleProcessModel implements
        NodeRunningStatusListener,
        NodeMessageListener,
        NavigationGoalListener,
        ExternalNavigationGoalListener {
    private static final Logger LOG = LoggerFactory.getLogger(Ros2CommAdapter.class);

    //================================================================================
    // Class variables.
    //================================================================================

    @Getter
    private NodeManager nodeManager;
    @Getter
    private NavigationGoalTracker navigationGoalTracker;
    @Getter
    private Triple estimatedPosition;
    @Setter
    private String namespace = "";
    @Setter
    private ExecuteTransportOrderWorkflow executeTransportOrderWorkflow;
    @Setter
    private ExecuteOperationWorkflow executeOperationWorkflow;

    //================================================================================
    // Constructor.
    //================================================================================

    public Ros2ProcessModel(@Nonnull Vehicle attachedVehicle) {
        super(attachedVehicle);
        this.nodeManager = new NodeManager();
        this.navigationGoalTracker = new NavigationGoalTracker(this, this);
    }

    //================================================================================
    // Startup / Shutdown methods.
    //================================================================================

    public void onDriverEnable() {
        this.nodeManager.start(this, this, this.namespace);
    }

    public void onDriverDisable() {
        this.navigationGoalTracker.reset();
        getPropertyChangeSupport().firePropertyChange(NAVIGATION_GOALS.name(), null, null);
        this.nodeManager.stop();
    }

    //================================================================================
    // Navigation methods.
    //================================================================================

    public void setInitialPoint(@Nonnull Point initialPoint) {
        final PoseWithCovarianceStamped message = OutgoingMessageLib.generateInitialPoseMessageByPoint(initialPoint);
        this.nodeManager.getNode().getInitialPosePublisher().publish(message);
    }

    public void dispatchToCoordinate(@Nonnull Triple coordinate) {
        final CoordinatePoint coordinatePoint = new CoordinatePoint(coordinate);
        dispatchToPoint(coordinatePoint);
    }

    public void dispatchToPoint(@Nonnull Point point) {
        LOG.info("Dispatching vehicle to point '{}'", point.getName());
        PoseStamped message = OutgoingMessageLib.generateScaledNavigationMessageByPoint(point);

        this.navigationGoalTracker.setDestinationPointIncomingGoal(point); // Notify NavigationGoalTracker that we expect a new goal
        this.nodeManager.getNode().getGoalPublisher().publish(message);
    }

    //================================================================================
    // Navigation callback methods.
    //================================================================================

    /* --------------- Navigation goals initiated by OpenTCS ---------------*/
    @Override
    public void onNavigationGoalActive(@Nonnull Point point) {
        if (executeTransportOrderWorkflow != null && executeTransportOrderWorkflow.isCommandExecutorActive()) {
            // Vehicle state is set by the ExecuteCommandWorkflow
        } else {
            setVehicleState(Vehicle.State.EXECUTING);
        }
    }

    @Override
    public void onNavigationGoalSucceeded(@Nonnull Point point) {
        if (point instanceof CoordinatePoint) {
            // The vehicle reached a coordinate point.
            // Since this is a fictional point, the vehicle position should be unknown.
            setVehiclePosition(null);
            setVehicleState(Vehicle.State.UNAVAILABLE);
        } else {
            setVehiclePosition(point.getName());
            setVehiclePrecisePosition(point.getPosition());

            if (executeTransportOrderWorkflow != null && executeTransportOrderWorkflow.isCommandExecutorActive()) {
                // Vehicle state is set by the ExecuteCommandWorkflow
            } else {
                setVehicleState(Vehicle.State.IDLE);
            }
        }
    }

    /* --------------- Navigation goals initiated an external application ---------------*/
    @Override
    public void onExternalNavigationGoalActive() {
        setVehiclePosition(null); // Because we are at a given coordinate, which is an unknown position for our plant.
        setVehicleState(Vehicle.State.EXECUTING);
    }

    @Override
    public void onExternalNavigationGoalSucceeded() {
        setVehiclePosition(null); // Because we are at a given coordinate, which is an unknown position for our plant.
        setVehicleState(Vehicle.State.UNAVAILABLE);
    }

    //================================================================================
    // Node message callback methods.
    //================================================================================

    /* --------------- Position update callbacks ---------------*/
    @Override
    public void onNewGoalStatusArray(@Nonnull GoalStatusArray goalStatusArray) {
        Object[][] oldValue = navigationGoalTracker.toStringTable();
        this.navigationGoalTracker.updateByGoalStatusArray(goalStatusArray);
        Object[][] newValue = navigationGoalTracker.toStringTable();

        getPropertyChangeSupport().firePropertyChange(NAVIGATION_GOALS.name(), oldValue, newValue);
    }

    @Override
    public void onNewAmclPose(@Nonnull PoseWithCovarianceStamped amclPose) {
        Triple oldEstimatePosition = this.estimatedPosition;
        this.estimatedPosition = IncomingMessageLib.generateTripleByAmclPose(amclPose);

        // Set precise position
        setVehiclePrecisePosition(this.estimatedPosition);
        getPropertyChangeSupport().firePropertyChange(POSITION_ESTIMATE.name(), oldEstimatePosition, this.estimatedPosition);

        // Set orientation angle
        Quaternion orientationQuaternion = amclPose.getPose().getPose().getOrientation();
        double orientationDegrees = UnitConverterLib.quaternionToAngleDegree(orientationQuaternion);
        setVehicleOrientationAngle(orientationDegrees);
    }

    /* --------------- Operation update callbacks (to be forwarded to OperationWorkflow) ---------------*/
    @Override
    public void onOperationLoadCargoFeedback(@Nonnull String feedback) {
        this.executeOperationWorkflow.onExecuteLoadCargoFeedback();
    }

    @Override
    public void onOperationUnloadCargoFeedback(@Nonnull String feedback) {
        this.executeOperationWorkflow.onExecuteUnloadCargoFeedback();
    }

    //================================================================================
    // Node running status callback method.
    //================================================================================

    @Override
    public void onNodeRunningStatusUpdate(@Nonnull NodeRunningStatus newNodeRunningStatus) {
        getPropertyChangeSupport().firePropertyChange(NODE_STATUS.name(), null, newNodeRunningStatus);
    }
}
