package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.constants.NodeRunningStatus;

/**
 * Listener for notifying about a (new) node running status.
 *
 * @author Niels Tiben
 */
public interface NodeRunningStatusListener {
    void onNodeRunningStatusUpdate(NodeRunningStatus nodeRunningStatus);
}
