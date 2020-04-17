package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

public interface NodeStatusChangeListener {
    void onNodeStatusChange(NodeStatus nodeStatus);
}
