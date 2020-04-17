package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.PoseStamped;
import lombok.Getter;
import lombok.Setter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.*;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.NavigationGoalTracker;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.operation.constants.OperationConstants;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleProcessModel;

public class Ros2ProcessModel extends VehicleProcessModel implements NodeStatusChangeListener, NodeListener {
    private final String loadOperation;
    private final String unloadOperation;
    @Getter
    private final NavigationGoalTracker navigationGoalTracker;
    @Setter
    private int domainId = 0;
    @Getter
    private NodeManager nodeManager;

    @Getter
    private int operatingTime;

    public Ros2ProcessModel(Vehicle attachedVehicle) {
        super(attachedVehicle);
        this.loadOperation = extractLoadOperation(attachedVehicle);
        this.unloadOperation = extractUnloadOperation(attachedVehicle);
        this.navigationGoalTracker = new NavigationGoalTracker();
        this.operatingTime = parseOperatingTime(attachedVehicle);
        this.nodeManager = new NodeManager();
    }

    public void startNode() {
        nodeManager.start(this, this, domainId);
    }

    /* --------------- Navigation ---------------*/
    public void dispatchToCoordinate(double x, double y) {
        PoseStamped message = MessageLib.generatePoseStampedByCoordinate(x,y);
        nodeManager.getNode().getGoalPublisher().publish(message);
    }

    /* --------------- Operation ---------------*/


    public String getLoadOperation() {
        return loadOperation;
    }

    public String getUnloadOperation() {
        return unloadOperation;
    }

    /**
     * Sets the default operating time.
     * goalStatusArray.get
     *
     * @param defaultOperatingTime The new default operating time
     */
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

    private static String extractLoadOperation(Vehicle attachedVehicle) {
        return OperationConstants.LOAD_CARGO;
    }

    private static String extractUnloadOperation(Vehicle attachedVehicle) {
        return OperationConstants.UNLOAD_CARGO;
    }


    @Override
    public void onNewGoalStatusArray(GoalStatusArray goalStatusArray) {
        Object[][] oldValue = navigationGoalTracker.generateStringMatrix();
        navigationGoalTracker.updateByGoalStatusArray(goalStatusArray);
        Object[][] newValue = navigationGoalTracker.generateStringMatrix();

        getPropertyChangeSupport().firePropertyChange(Attribute.NAVIGATION_GOALS.name(), oldValue, newValue);
    }

    @Override
    public void onNodeStatusChange(NodeStatus newNodeStatus) {
        getPropertyChangeSupport().firePropertyChange(Attribute.NODE_STATUS.name(), null, newNodeStatus);
    }

    /**
     * Notification arguments to indicate some change.
     */
    public enum Attribute {
        OPERATING_TIME,
        NODE_STATUS,
        NAVIGATION_GOALS
    }
}
