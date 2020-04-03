package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import lombok.Getter;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


import java.io.IOException;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionStatus.*;

public class ConnectionController {
    private static final Logger LOG = LoggerFactory.getLogger(ConnectionController.class);
    private ConnectionListener connectionListener;

    @Getter
    private ConnectionStatus connectionStatus;

    @Getter
    private int domainId;

    public ConnectionController(ConnectionListener connectionListener) {
        this.connectionListener = connectionListener;
        setConnectionStatus(DISCONNECTED);
    }

    public void connect(int domainId) {
        LOG.info("Connecting ROS2 node...");
        this.domainId = domainId;

        setConnectionStatus(ESTABLISHING_CONNECTION);

//        try {
//            PublisherLambda.main(null);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        setConnectionStatus(CONNECTED);
    }

    private void setConnectionStatus(ConnectionStatus connectionStatus) {
        this.connectionStatus = connectionStatus;
        connectionListener.onConnectionStatusChange(connectionStatus);
    }

    public void disconnect(){
        // TODO: destroy node
    }
}
