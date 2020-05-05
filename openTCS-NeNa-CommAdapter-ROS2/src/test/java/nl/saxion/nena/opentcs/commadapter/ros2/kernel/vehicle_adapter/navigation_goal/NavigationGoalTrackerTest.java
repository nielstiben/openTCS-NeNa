package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal;

import action_msgs.msg.GoalStatusArray;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.Node;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.constants.NavigationGoalStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library.NavigationGoalTestLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.point.CoordinatePoint;
import org.junit.Before;
import org.junit.Test;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;

import javax.annotation.Nonnull;
import java.util.Arrays;

/**
 * Unit test to cover {@link NavigationGoalTracker}.
 * Dummy messages are used for filling the navigation goal tracker.
 *
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public class NavigationGoalTrackerTest {
    private NavigationGoalListener navigationGoalListener;
    private ExternalNavigationGoalListener externalNavigationGoalListener;

    private final byte NAV_ACTIVE = 2;
    private final byte NAV_SUCCEEDED = 4;

    //================================================================================
    // Callback variables.
    //================================================================================
    private Point latestNavigationGoalActivePoint;
    private Point latestNavigationGoalSucceededPoint;

    private boolean externalNavigationGoalActiveCallbackIsCalled;
    private boolean externalNavigationGoalSucceededCallbackIsCalled;

    //================================================================================
    // Pre operation
    //================================================================================
    @Before
    public void setUpListeners() {
        this.navigationGoalListener = new NavigationGoalListener() {
            @Override
            public void onNavigationGoalActive(@Nonnull Point point) {
                latestNavigationGoalActivePoint = point;
            }

            @Override
            public void onNavigationGoalSucceeded(@Nonnull Point point) {
                latestNavigationGoalSucceededPoint = point;
            }
        };
        this.externalNavigationGoalListener = new ExternalNavigationGoalListener() {
            @Override
            public void onExternalNavigationGoalActive() {
                externalNavigationGoalActiveCallbackIsCalled = true;
            }

            @Override
            public void onExternalNavigationGoalSucceeded() {
                externalNavigationGoalSucceededCallbackIsCalled = true;
            }
        };
    }

    //================================================================================
    // Tests
    //================================================================================

    @Test
    public void testNavigationGoalTrackerOpenTcsGoal() {
        NavigationGoalTracker navigationGoalTracker = new NavigationGoalTracker(
                this.navigationGoalListener,
                this.externalNavigationGoalListener
        );
        // Also redirect callbacks intended for the command executor to this test.
        navigationGoalTracker.setCommandExecutorListener(this.navigationGoalListener);

        // Create an OpenTCS point, and let the Navigation Goal Tracker know that we expect an update
        CoordinatePoint destinationActiveGoal = new CoordinatePoint(new Triple(100, -150, 0));
        navigationGoalTracker.setDestinationPointIncomingGoal(destinationActiveGoal);

        // Fill the navigation goal tracker with dummy data.
        navigationGoalTracker.updateByGoalStatusArray(createDummyGoalStatusArray());

        // The newest entry of the dummy data had status ACTIVE => check if callback has been made
        assert this.latestNavigationGoalActivePoint != null;
    }

    @Test
    public void testNavigationGoalTrackerOpenTcsGoalUpdate() {
        NavigationGoalTracker navigationGoalTracker = new NavigationGoalTracker(
                this.navigationGoalListener,
                this.externalNavigationGoalListener
        );
        // Also redirect callbacks intended for the command executor to this test.
        navigationGoalTracker.setCommandExecutorListener(this.navigationGoalListener);

        // Create an OpenTCS point, and let the Navigation Goal Tracker know that we expect an update.
        CoordinatePoint destinationActiveGoal = new CoordinatePoint(new Triple(100, -150, 0));
        navigationGoalTracker.setDestinationPointIncomingGoal(destinationActiveGoal);

        // Add a single entry (status ACTIVE) to the Goal Navigation Tracker.
        GoalStatusArray initialGoalStatusArray = NavigationGoalTestLib.createDummyGoalStatusArraySingleEntry(NAV_ACTIVE);
        navigationGoalTracker.updateByGoalStatusArray(initialGoalStatusArray);
        assert this.latestNavigationGoalActivePoint != null;
        assert this.latestNavigationGoalSucceededPoint == null; // The goal has not succeeded yet!

        // Update the same entry from 'ACTIVE' to 'SUCCEEDED'
        GoalStatusArray updatedGoalStatusArray = NavigationGoalTestLib.createDummyGoalStatusArraySingleEntry(NAV_SUCCEEDED);
        navigationGoalTracker.updateByGoalStatusArray(updatedGoalStatusArray);

        // The newest entry of the dummy data had status ACTIVE => check if callback has been made
        assert this.latestNavigationGoalSucceededPoint != null; // The goal has now been succeeded.
    }

    @Test
    public void testNavigationGoalTrackerExternalGoalUpdate() {
        NavigationGoalTracker navigationGoalTracker = new NavigationGoalTracker(
                this.navigationGoalListener,
                this.externalNavigationGoalListener
        );

        // Add a single entry (status ACTIVE) to the Goal Navigation Tracker.
        GoalStatusArray initialGoalStatusArray = NavigationGoalTestLib.createDummyGoalStatusArraySingleEntry(NAV_ACTIVE);
        navigationGoalTracker.updateByGoalStatusArray(initialGoalStatusArray);
        assert this.externalNavigationGoalActiveCallbackIsCalled;
        assert !this.externalNavigationGoalSucceededCallbackIsCalled; // The goal has not succeeded yet!

        // Update the same entry from 'ACTIVE' to 'SUCCEEDED'
        GoalStatusArray updatedGoalStatusArray = NavigationGoalTestLib.createDummyGoalStatusArraySingleEntry(NAV_SUCCEEDED);
        navigationGoalTracker.updateByGoalStatusArray(updatedGoalStatusArray);

        // The newest entry of the dummy data had status ACTIVE => check if callback has been made
        assert this.externalNavigationGoalSucceededCallbackIsCalled;
        ; // The goal has now been succeeded.
    }

    @Test
    public void testNavigationGoalTrackerExternalGoal() {
        NavigationGoalTracker navigationGoalTracker = new NavigationGoalTracker(
                this.navigationGoalListener,
                this.externalNavigationGoalListener
        );
        // Fill the navigation goal tracker with dummy data.
        navigationGoalTracker.updateByGoalStatusArray(createDummyGoalStatusArray());

        // The newest entry of the dummy data had status ACTIVE => check if callback has been made
        assert externalNavigationGoalActiveCallbackIsCalled;
    }

    @Test
    public void testStringifyTable() {
        NavigationGoalTracker navigationGoalTracker = new NavigationGoalTracker(
                this.navigationGoalListener,
                this.externalNavigationGoalListener
        );
        String[][] table = navigationGoalTracker.toStringTable();
        assert table == null; // The navigation goal tracker has never been updated.

        navigationGoalTracker.updateByGoalStatusArray(createDummyGoalStatusArray());
        table = navigationGoalTracker.toStringTable();

        assert table != null; // Does table contain fields?
        assert table.length == 3; // The dummy goal status array has three entries.
        assert table[0].length == 4; // The dummy goal status array has four columns.
        assert table[0][3].equals(NavigationGoalStatus.ACTIVE.name()); // The newest goal should have status active.
    }

    //================================================================================
    // Helper methods
    //================================================================================

    /**
     * Mocks a {@link GoalStatusArray} that could come from the {@link Node} amcl_pose subscriber.
     * The NAV status codes correspond to {@link NavigationGoalStatus}
     *
     * @return A GoalStatusArray holding three goal statuses of which the 'newest' is currently being active.
     */
    private GoalStatusArray createDummyGoalStatusArray() {
        GoalStatusArray dummyGoalStatusArray = new GoalStatusArray();
        dummyGoalStatusArray.setStatusList(Arrays.asList(
                NavigationGoalTestLib.createDummyGoalStatus((byte) 2, NAV_SUCCEEDED),
                NavigationGoalTestLib.createDummyGoalStatus((byte) 3, NAV_SUCCEEDED),
                NavigationGoalTestLib.createDummyGoalStatus((byte) 1, NAV_ACTIVE)
        ));

        return dummyGoalStatusArray;
    }
}
