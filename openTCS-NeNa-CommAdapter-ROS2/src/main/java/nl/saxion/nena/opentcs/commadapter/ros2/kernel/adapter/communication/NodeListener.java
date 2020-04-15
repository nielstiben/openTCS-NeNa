package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import action_msgs.msg.GoalStatusArray;

public interface NodeListener {
    void onNewGoalStatusArray(GoalStatusArray goalStatusArray);
}
