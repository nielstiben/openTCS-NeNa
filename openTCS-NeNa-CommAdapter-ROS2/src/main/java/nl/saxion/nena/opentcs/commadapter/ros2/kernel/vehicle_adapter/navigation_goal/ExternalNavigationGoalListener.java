package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal;

/**
 * Listener for notifying about an updated EXTERNAL navigation goal.
 * The particular navigation goal should NOT have been initiated by openTCS.
 *
 * @author Niels Tiben
 */
public interface ExternalNavigationGoalListener {
    void onExternalNavigationGoalActive();

    void onExternalNavigationGoalSucceeded();
}
