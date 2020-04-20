package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

public interface NodeRunningStatusListener {
    void onNodeStatusChange(NodeRunningStatus nodeRunningStatus);
}
