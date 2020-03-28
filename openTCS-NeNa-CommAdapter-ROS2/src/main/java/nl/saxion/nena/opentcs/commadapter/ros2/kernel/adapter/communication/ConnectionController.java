package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import lombok.Getter;
import lombok.NonNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;
import us.ihmc.ros2.Ros2Subscription;

import java.io.IOException;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionStatus.*;

public class ConnectionController {
    private static final Logger LOG = LoggerFactory.getLogger(ConnectionController.class);
    private ConnectionListener connectionListener;

    @Getter
    private ConnectionStatus connectionStatus;

    @Getter
    private int domainId;

    private Ros2Node node;
    private Ros2Publisher publisher;
    private Ros2Subscription subscriber;

    public ConnectionController(ConnectionListener connectionListener) {
        this.connectionListener = connectionListener;
        setConnectionStatus(DISCONNECTED);
    }

    public void connect(int domainId) {
        LOG.info("Connecting ROS2 node...");
        this.domainId = domainId;

        setConnectionStatus(ESTABLISHING_CONNECTION);

        // Node
        try {
            node = new Ros2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "nenaOpenTCS", "/nl/saxion", domainId);
        } catch (IOException e) {
            e.printStackTrace();
            setConnectionStatus(ERROR);
            return;
        }
        setConnectionStatus(CONNECTED);
    }

    private void setConnectionStatus(ConnectionStatus connectionStatus) {
        this.connectionStatus = connectionStatus;
        connectionListener.onConnectionStatusChange(connectionStatus);
    }

    public void disconnect(){
        if (node == null){
            LOG.error("Could not disconnect because the node is not connected");
        } else {
            node.destroy();
            node = null;
            setConnectionStatus(DISCONNECTED);
        }
    }
}
