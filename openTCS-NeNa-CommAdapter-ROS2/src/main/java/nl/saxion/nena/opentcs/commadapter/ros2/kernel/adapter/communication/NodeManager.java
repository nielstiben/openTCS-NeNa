package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.SneakyThrows;
import org.ros2.rcljava.RCLJava;

import javax.annotation.Nonnull;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.NodeRunningStatus.*;

/**
 * Helper class to start a new ROS2 node in a separate thread.
 * Starting a new ROS2 node takes time so a non-blocking callback
 * is used to notify other instances when a ROS2 node has been initiated.
 *
 * @author Niels Tiben <nielstiben@outlook.com>
 */
@NoArgsConstructor
public class NodeManager implements NodeStarterListener {
    private NodeRunningStatusListener nodeRunningStatusListener;
    private NodeRunnable nodeRunnable;

    @Getter
    private OpentcsNode opentcsNode;

    @Getter
    private NodeRunningStatus nodeRunningStatus = NOT_ACTIVE;

    public void start(NodeRunningStatusListener nodeRunningStatusListener, NodeMessageListener nodeMessageListener, String namespace) {
        assert this.nodeRunningStatus == NodeRunningStatus.NOT_ACTIVE;
        this.nodeRunningStatusListener = nodeRunningStatusListener;

        changeNodeStatus(INITIATING);
        NodeRunnable nodeRunnable = new NodeRunnable(nodeMessageListener, this);
        new Thread(nodeRunnable).start();
    }

    public void stop() {
        assert this.nodeRunningStatus == ACTIVE; // Only stopping active nodes can be stopped.
        changeNodeStatus(TERMINATING);
        this.nodeRunnable.setRunning(false);
    }

    @Override
    public void onNodeStarted(@Nonnull NodeRunnable initialisedNodeRunnable) {
        this.nodeRunnable = initialisedNodeRunnable;
        this.opentcsNode = initialisedNodeRunnable.getOpentcsNode();
        changeNodeStatus(ACTIVE);
    }

    @Override
    public void onNodeStopped() {
        changeNodeStatus(NOT_ACTIVE);
        this.nodeRunnable = null;
        this.opentcsNode = null;
    }

    /**
     * Runnable for the node instance.
     */
    protected static class NodeRunnable implements Runnable {
        @Getter
        private OpentcsNode opentcsNode;
        private final NodeMessageListener nodeMessageListener;
        private NodeStarterListener nodeStarterListener;
        @Setter
        private boolean isRunning;

        public NodeRunnable(NodeMessageListener nodeMessageListener, NodeStarterListener nodeStarterListener) {
            this.nodeMessageListener = nodeMessageListener;
            this.nodeStarterListener = nodeStarterListener;
        }

        @SneakyThrows
        @Override
        public void run() {
            this.isRunning = true;
            RCLJava.rclJavaInit();
            this.opentcsNode = new OpentcsNode(nodeMessageListener);
            nodeStarterListener.onNodeStarted(this);

            while (RCLJava.ok() && this.isRunning) {
                RCLJava.spinOnce(this.opentcsNode.getNode());
            }

            this.opentcsNode.shutdown();
            this.opentcsNode = null;
            nodeStarterListener.onNodeStopped();
        }
    }

    private void changeNodeStatus(NodeRunningStatus newNodeRunningStatus) {
        this.nodeRunningStatus = newNodeRunningStatus;
        nodeRunningStatusListener.onNodeStatusChange(newNodeRunningStatus);
    }
}
