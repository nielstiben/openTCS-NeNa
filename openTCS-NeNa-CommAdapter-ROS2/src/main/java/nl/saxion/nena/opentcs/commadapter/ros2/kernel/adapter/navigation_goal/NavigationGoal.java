package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal;

import action_msgs.msg.GoalStatus;
import builtin_interfaces.msg.Time;
import lombok.Getter;
import org.opentcs.data.model.Point;

import javax.annotation.Nonnull;
import java.time.Instant;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.List;

@Getter
public class NavigationGoal implements Comparable<NavigationGoal> {
    private final List<Byte> uuid;
    private final ZonedDateTime created;
    private ZonedDateTime lastUpdated;
    private NavigationGoalStatus navigationGoalStatus;
    private final Point destinationPoint;

    public NavigationGoal(@Nonnull GoalStatus goalStatus, Point destinationPoint) {
        this.uuid = goalStatus.getGoalInfo().getGoalId().getUuid();
        this.created = parseTimestampFromGoalStatus(goalStatus);
        this.lastUpdated = parseTimestampFromGoalStatus(goalStatus);
        this.navigationGoalStatus = NavigationGoalStatus.getByStatusCode(goalStatus.getStatus());
        this.destinationPoint = destinationPoint;
    }

    public void setLastUpdatedByGoalStatus(@Nonnull GoalStatus goalStatus) {
        this.lastUpdated = parseTimestampFromGoalStatus(goalStatus);
    }

    public void setNavigationGoalStatusByGoalStatus(@Nonnull GoalStatus goalStatus) {
        this.navigationGoalStatus = NavigationGoalStatus.getByStatusCode(goalStatus.getStatus());
    }

    @Nonnull
    private ZonedDateTime parseTimestampFromGoalStatus(GoalStatus goalStatus) {
        Time stamp = goalStatus.getGoalInfo().getStamp();
        return Instant
                .ofEpochSecond(stamp.getSec(), stamp.getNanosec())
                .atZone(ZoneId.systemDefault());
    }

    @Override
    public int compareTo(NavigationGoal otherNavigationGoal) {
        return this.lastUpdated.compareTo(otherNavigationGoal.lastUpdated);
    }
}
