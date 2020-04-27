package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal;

/**
 * @author Niels Tiben
 */
public interface ExternalNavigationGoalListener {
    void onExternalNavigationGoalActive();
    void onExternalNavigationGoalSucceeded();
}
