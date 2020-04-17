package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

public interface NodeStarterListener {
    void onNodeInitiated(Node node) throws InterruptedException;
}
