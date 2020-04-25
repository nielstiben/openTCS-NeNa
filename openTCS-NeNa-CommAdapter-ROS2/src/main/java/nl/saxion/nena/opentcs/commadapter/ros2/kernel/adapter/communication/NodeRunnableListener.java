package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

public interface NodeRunnableListener {
    void onNodeStarted(NodeManager.NodeRunnable nodeRunnable) throws InterruptedException;
    void onNodeStopped();
}
