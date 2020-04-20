package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.SneakyThrows;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executors.SingleThreadedExecutor;

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

    @Getter
    private Node node;

    @Getter
    private NodeRunningStatus nodeRunningStatus = NOT_ACTIVE;

    public void start(NodeRunningStatusListener nodeRunningStatusListener, NodeMessageListener nodeMessageListener, int domainId) {
        assert this.nodeRunningStatus == NodeRunningStatus.NOT_ACTIVE;

        this.nodeRunningStatusListener = nodeRunningStatusListener;
        changeNodeStatus(INITIATING);

        NodeRunnable nodeRunnable = new NodeRunnable(nodeMessageListener);
        NodeWatcher nodeWatcher = new NodeWatcher(nodeRunnable, this);

        new Thread(nodeWatcher).start();
    }

    public void stop() {
        assert this.nodeRunningStatus == ACTIVE; // Only stopping active nodes can be stopped.
        changeNodeStatus(NOT_ACTIVE);
        this.node.stop();
        this.node = null;
    }

    @Override
    public void onNodeInitiated(Node initialisedNode) {
        this.node = initialisedNode;
        changeNodeStatus(ACTIVE);
    }

    /**
     * Runnable that watches the NodeRunnable and gives a callback when the node has been initialised.
     */
    @AllArgsConstructor
    private static class NodeWatcher implements Runnable {
        private NodeRunnable nodeRunnable;
        private NodeStarterListener nodeStarterListener;

        @SneakyThrows
        @Override
        public void run() {
            new Thread(nodeRunnable).start();

            while (nodeRunnable.getNode() == null) {
                // Waiting for node to initiate...
                Thread.sleep(500);
            }
            nodeStarterListener.onNodeInitiated(nodeRunnable.node);
        }
    }

    /**
     * Runnable for the node instance.
     */
    private static class NodeRunnable implements Runnable {
        @Getter
        private Node node;
        private NodeMessageListener nodeMessageListener;

        public NodeRunnable(NodeMessageListener nodeMessageListener) {
            this.nodeMessageListener = nodeMessageListener;
        }

        @Override
        public void run() {
            RCLJava.rclJavaInit();
            SingleThreadedExecutor exec = new SingleThreadedExecutor();
            this.node = new Node(nodeMessageListener);
            exec.addNode(node);
            exec.spin();
        }
    }

    private void changeNodeStatus(NodeRunningStatus newNodeRunningStatus) {
        this.nodeRunningStatus = newNodeRunningStatus;
        nodeRunningStatusListener.onNodeStatusChange(newNodeRunningStatus);
    }
}
