package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal;

import org.opentcs.data.model.Point;

/**
 * @author Niels Tiben <nielstiben@outlook.com>
 */
public interface NavigationGoalListener {
    void onNavigationGoalSucceeded(Point point);
}
