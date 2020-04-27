package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import nl.saxion.nena.opentcs.commadapter.ros2.control_center.Ros2CommAdapterPanel;

/**
 * Attributes used as flag to indicate model change to control center panel {@link Ros2CommAdapterPanel}.
 *
 * @author Niels Tiben
 */
public enum Ros2ProcessModelAttribute {
    NODE_STATUS,
    NAVIGATION_GOALS,
    POSITION_ESTIMATE,
}
