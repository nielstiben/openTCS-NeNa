package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.PoseWithCovarianceStamped;

public interface NodeMessageListener {
    void onNewGoalStatusArray(GoalStatusArray goalStatusArray);
    void onNewAmclPose(PoseWithCovarianceStamped amclPose);
}
