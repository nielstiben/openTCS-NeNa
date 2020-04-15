package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import action_msgs.msg.GoalStatus;
import action_msgs.msg.GoalStatusArray;

import java.util.List;

public class SubscriptionMessageParserLib {
    public static GoalStatus parseLatestGoalStatusByGoalStatusArray(GoalStatusArray goalStatusArray) {
        List<GoalStatus> goalStatusList = goalStatusArray.getStatusList();

        return goalStatusList.get((goalStatusList.size() - 1));
    }
}
