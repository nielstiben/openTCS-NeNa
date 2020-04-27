package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

/**
 * Listener for notifying when a node has started or has stopped.
 *
 * @author Niels Tiben
 */
public interface NodeRunnableListener {
    void onNodeStarted(NodeManager.NodeRunnable nodeRunnable) throws InterruptedException;
    void onNodeStopped();
}
