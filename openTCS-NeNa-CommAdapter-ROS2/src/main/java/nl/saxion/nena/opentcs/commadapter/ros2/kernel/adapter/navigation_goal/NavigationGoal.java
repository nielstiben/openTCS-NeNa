package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal;

import action_msgs.msg.GoalInfo;
import action_msgs.msg.GoalStatus;
import builtin_interfaces.msg.Time;
import lombok.Getter;
import lombok.Setter;

import java.time.Instant;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.List;

@Getter
public class NavigationGoal implements Comparable<NavigationGoal> {
    private List<Byte> uuid;

    @Setter
    private ZonedDateTime lastUpdated;

    @Setter
    private NavigationGoalStatus navigationGoalStatus;


    public NavigationGoal(GoalStatus goalStatus) {
        GoalInfo goalInfo = goalStatus.getGoalInfo();
        Time stamp = goalInfo.getStamp();

        uuid = goalInfo.getGoalId().getUuid();
        lastUpdated = Instant
                .ofEpochSecond(stamp.getSec(), stamp.getNanosec())
                .atZone(ZoneId.systemDefault());
        navigationGoalStatus = NavigationGoalStatus.getByStatusCode(goalStatus.getStatus());
    }

    @Override
    public int compareTo(NavigationGoal otherNavigationGoal) {
        return this.lastUpdated.compareTo(otherNavigationGoal.lastUpdated);
    }
}
