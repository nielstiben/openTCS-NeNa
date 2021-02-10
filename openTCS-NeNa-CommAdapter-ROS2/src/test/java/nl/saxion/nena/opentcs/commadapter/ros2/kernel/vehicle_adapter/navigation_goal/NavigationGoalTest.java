/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal;

import action_msgs.msg.dds.GoalStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.constants.NavigationGoalStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.NavigationGoalTestLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.point.CoordinatePoint;
import org.junit.Test;
import org.opentcs.data.model.Triple;

/**
 * Unit test to cover {@link NavigationGoal}.
 *
 * @author Niels Tiben
 */
public class NavigationGoalTest {
    private static final byte UUID_LAST_BYTE = 1;
    private static final byte NAV_ACTIVE = 2;
    private static final byte NAV_SUCCEEDED = 4;

    @Test
    public void testParseGoalStatus() {
        // Get dummy goal status
        GoalStatus dummyGoalStatus = NavigationGoalTestLib.createDummyGoalStatus(UUID_LAST_BYTE, NAV_ACTIVE);
        NavigationGoal dummyNavigationGoal = new NavigationGoal(dummyGoalStatus, null);

        // Assert correct status
        assert dummyNavigationGoal.getNavigationGoalStatus().equals(NavigationGoalStatus.ACTIVE);

        // Assert destination point is still unknown
        assert dummyNavigationGoal.getDestinationPoint() == null;

        // Assert UUID matches.
        assert dummyNavigationGoal.getUuid()[15] == UUID_LAST_BYTE;
    }

    @Test
    public void testUpdateGoalStatus() {
        // Get dummy goal status
        GoalStatus initialGoalStatus = NavigationGoalTestLib.createDummyGoalStatus(UUID_LAST_BYTE, NAV_ACTIVE);
        NavigationGoal dummyNavigationGoal = new NavigationGoal(initialGoalStatus, null);

        // Update navigation goal status
        GoalStatus updatedGoalStatus = NavigationGoalTestLib.createDummyGoalStatus(UUID_LAST_BYTE, NAV_SUCCEEDED);
        dummyNavigationGoal.setNavigationGoalStatusByGoalStatus(updatedGoalStatus);

        assert dummyNavigationGoal.getNavigationGoalStatus().equals(NavigationGoalStatus.SUCCEEDED);
    }

    @Test
    public void testParseGoalStatusWithDestination() {
        // Get dummy goal status
        GoalStatus dummyGoalStatus = NavigationGoalTestLib.createDummyGoalStatus(UUID_LAST_BYTE, NAV_ACTIVE);
        CoordinatePoint dummyDestination = new CoordinatePoint(new Triple(0, 0, 0));
        NavigationGoal dummyNavigationGoal = new NavigationGoal(dummyGoalStatus, dummyDestination);

        // Assert correct status
        assert dummyNavigationGoal.getNavigationGoalStatus().equals(NavigationGoalStatus.ACTIVE);

        // Assert destination point is valid
        assert dummyNavigationGoal.getDestinationPoint() != null;

        // Assert UUID matches.
        assert dummyNavigationGoal.getUuid()[15] == UUID_LAST_BYTE;
    }
}
