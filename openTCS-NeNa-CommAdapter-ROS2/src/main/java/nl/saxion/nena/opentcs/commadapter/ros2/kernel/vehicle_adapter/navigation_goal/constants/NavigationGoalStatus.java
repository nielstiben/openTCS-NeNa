package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.constants;

import lombok.AllArgsConstructor;
import lombok.Getter;

/**
 * Enum representing the Navigation Goal Status.
 * A status can be parsed using ROS2 status codes numbers, that is provided with each navigation Goal.
 *
 * @author Niels Tiben
 */
@AllArgsConstructor
public enum NavigationGoalStatus {
    PENDING("The goal has yet to be processed by the action server."),
    ACTIVE("The goal is currently being processed by the action server."),
    PREEMPTED("The goal received a cancel request after it started executing and has since completed its execution (Terminal State)."),
    SUCCEEDED(" The goal was achieved successfully by the action server (Terminal State)."),
    ABORTED("The goal was aborted during execution by the action server due to some failure (Terminal State)."),
    REJECTED("The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)."),
    PREEMPTING("The goal received a cancel request after it started executing and has not yet completed execution."),
    RECALLING("The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled."),
    RECALLED("The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)."),
    LOST("An action client can determine that a goal is LOST. This should not be sent over the wire by an action server."),
    UNKNOWN("The provided status code is not supported");

    @Getter
    private final String description;

    public static NavigationGoalStatus getByStatusCodeNumber(byte statusCodeNumber) {
        switch (statusCodeNumber) {
            case 1:
                return PENDING;
            case 2:
                return ACTIVE;
            case 3:
                return PREEMPTED;
            case 4:
                return SUCCEEDED;
            case 5:
                return ABORTED;
            case 6:
                return REJECTED;
            case 7:
                return PREEMPTING;
            case 8:
                return RECALLING;
            case 9:
                return RECALLED;
            case 10:
                return LOST;
            default:
                return UNKNOWN;
        }
    }
}
