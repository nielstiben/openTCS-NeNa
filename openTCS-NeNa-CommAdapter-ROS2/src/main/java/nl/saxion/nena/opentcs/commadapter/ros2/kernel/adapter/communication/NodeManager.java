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
    private NodeRunnable nodeRunnable;

    @Getter
    private Node node;

    @Getter
    private NodeRunningStatus nodeRunningStatus = NOT_ACTIVE;

    public void start(NodeRunningStatusListener nodeRunningStatusListener, NodeMessageListener nodeMessageListener, String namespace) {
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

        this.nodeRunnable.stop();
        this.nodeRunnable = null;
        this.node = null;
    }

    @Override
    public void onNodeInitiated(NodeRunnable initialisedNodeRunnable) {
        this.nodeRunnable = initialisedNodeRunnable;
        this.node = initialisedNodeRunnable.getNode();
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
            nodeStarterListener.onNodeInitiated(nodeRunnable);
        }
    }

    /**
     * Runnable for the node instance.
     */
    protected static class NodeRunnable implements Runnable {
        @Getter
        private Node node;
        private NodeMessageListener nodeMessageListener;
        private SingleThreadedExecutor executor;

        public NodeRunnable(NodeMessageListener nodeMessageListener) {
            this.nodeMessageListener = nodeMessageListener;
        }

        @Override
        public void run() {
            RCLJava.rclJavaInit();
            executor = new SingleThreadedExecutor();
            this.node = new Node(nodeMessageListener);
            executor.addNode(node);
            executor.spin();
        }

        public void stop(){
            this.node.stop();
            this.executor.removeNode(node); // Remove the (stopped) node, otherwise it is still shown in the node list.

            this.node = null;
            this.executor = null;

        }
    }

    private void changeNodeStatus(NodeRunningStatus newNodeRunningStatus) {
        this.nodeRunningStatus = newNodeRunningStatus;
        nodeRunningStatusListener.onNodeStatusChange(newNodeRunningStatus);
    }
}
