package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal;

import org.opentcs.data.model.Point;

import javax.annotation.Nonnull;

/**
 * Listener for notifying about an updated navigation goal.
 * The particular navigation goal has initiated by openTCS.
 *
 * @author Niels Tiben
 */
public interface NavigationGoalListener {
    void onNavigationGoalSucceeded(@Nonnull Point point);
    void onNavigationGoalActive(@Nonnull Point point);
}
