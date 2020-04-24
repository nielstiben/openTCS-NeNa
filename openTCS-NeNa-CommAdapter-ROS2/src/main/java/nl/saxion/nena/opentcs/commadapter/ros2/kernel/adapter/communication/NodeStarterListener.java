package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

public interface NodeStarterListener {
    void onNodeStarted(NodeManager.NodeRunnable nodeRunnable) throws InterruptedException;
    void onNodeStopped();
}
