package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication;

import action_msgs.msg.dds.GoalStatusArray;
import geometry_msgs.msg.dds.PoseWithCovarianceStamped;

/**
 * Listener for notifying about (new) incoming messages from the Node's subscriptions.
 *
 * @author Niels Tiben
 */
public interface NodeMessageListener {
    void onNewGoalStatusArray(GoalStatusArray goalStatusArray);
    void onNewAmclPose(PoseWithCovarianceStamped amclPose);
    void onOperationLoadCargoFeedback(String feedback);
    void onOperationUnloadCargoFeedback(String feedback);
}
