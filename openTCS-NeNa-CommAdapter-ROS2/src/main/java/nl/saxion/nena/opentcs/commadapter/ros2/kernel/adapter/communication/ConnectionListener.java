package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

public interface ConnectionListener {
    void onConnectionStatusChange(ConnectionStatus connectionStatus);
}
