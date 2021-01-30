package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal;

import action_msgs.msg.dds.GoalStatus;
import builtin_interfaces.msg.dds.Time;
import lombok.Getter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.constants.NavigationGoalStatus;
import org.opentcs.data.model.Point;

import javax.annotation.Nonnull;
import java.time.Instant;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.List;

/**
 * Class for holding a single Navigation Goal, which consist of
 * ROS2 (rcljava)-provided data (such as the UUID and parsed status)
 * and OpenTCS-provided data (destination point)
 *
 * @author Niels Tiben
 */
@Getter
public class NavigationGoal implements Comparable<NavigationGoal> {
    private final byte[] uuid;
    private final ZonedDateTime created;
    private final Point destinationPoint;
    private ZonedDateTime lastUpdated;
    private NavigationGoalStatus navigationGoalStatus;

    public NavigationGoal(@Nonnull GoalStatus goalStatus, Point destinationPoint) {
        this.uuid = goalStatus.getGoalInfo().getGoalId().getUuid();
        this.created = parseTimestampFromGoalStatus(goalStatus);
        this.lastUpdated = parseTimestampFromGoalStatus(goalStatus);
        this.navigationGoalStatus = NavigationGoalStatus.getByStatusCodeNumber(goalStatus.getStatus());
        this.destinationPoint = destinationPoint;
    }

    public void setLastUpdatedByGoalStatus(@Nonnull GoalStatus goalStatus) {
        this.lastUpdated = parseTimestampFromGoalStatus(goalStatus);
    }

    public void setNavigationGoalStatusByGoalStatus(@Nonnull GoalStatus goalStatus) {
        this.navigationGoalStatus = NavigationGoalStatus.getByStatusCodeNumber(goalStatus.getStatus());
    }

    private ZonedDateTime parseTimestampFromGoalStatus(@Nonnull GoalStatus goalStatus) {
        Time stamp = goalStatus.getGoalInfo().getStamp();
        return Instant
                .ofEpochSecond(stamp.getSec(), stamp.getNanosec())
                .atZone(ZoneId.systemDefault());
    }

    @Override
    public int compareTo(NavigationGoal otherNavigationGoal) {
        return this.lastUpdated.compareTo(otherNavigationGoal.lastUpdated); // Newest first.
    }
}
