package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal;

import action_msgs.msg.GoalStatus;
import builtin_interfaces.msg.Time;
import lombok.Getter;
import lombok.Setter;
import org.opentcs.data.model.Point;

import javax.annotation.Nonnull;
import java.time.Instant;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.List;

@Getter
public class NavigationGoal implements Comparable<NavigationGoal> {
    private final List<Byte> uuid;
    private ZonedDateTime lastUpdated;
    private NavigationGoalStatus navigationGoalStatus;

    @Setter
    private Point destinationPoint;

    public NavigationGoal(@Nonnull GoalStatus goalStatus) {
        this.uuid = goalStatus.getGoalInfo().getGoalId().getUuid();
        this.setLastUpdatedByGoalStatus(goalStatus);
        this.setNavigationGoalStatusByGoalStatus(goalStatus);
    }

    public void setLastUpdatedByGoalStatus(@Nonnull GoalStatus goalStatus) {
        Time stamp = goalStatus.getGoalInfo().getStamp();
        lastUpdated = Instant
                .ofEpochSecond(stamp.getSec(), stamp.getNanosec())
                .atZone(ZoneId.systemDefault());
    }

    public void setNavigationGoalStatusByGoalStatus(@Nonnull GoalStatus goalStatus) {
        this.navigationGoalStatus = NavigationGoalStatus.getByStatusCode(goalStatus.getStatus());
    }

    @Override
    public int compareTo(NavigationGoal otherNavigationGoal) {
        return this.lastUpdated.compareTo(otherNavigationGoal.lastUpdated);
    }
}
