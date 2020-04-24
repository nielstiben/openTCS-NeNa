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
    private OpenTcsNode opentcsNode;

    @Getter
    private NodeRunningStatus nodeRunningStatus = NOT_ACTIVE;

    public void start(NodeRunningStatusListener nodeRunningStatusListener, NodeMessageListener nodeMessageListener, String namespace) {
        assert this.nodeRunningStatus == NodeRunningStatus.NOT_ACTIVE;
        this.nodeRunningStatusListener = nodeRunningStatusListener;

        changeNodeStatus(INITIATING);

        NodeRunnable nodeRunnable = new NodeRunnable(nodeMessageListener, this);
        NodeWatcher nodeWatcher = new NodeWatcher(nodeRunnable, this);

        new Thread(nodeWatcher).start();
    }

    public void stop() {
        assert this.nodeRunningStatus == ACTIVE; // Only stopping active nodes can be stopped.
        changeNodeStatus(TERMINATING);

        this.nodeRunnable.stop();

    }

    @Override
    public void onNodeStarted(NodeRunnable nodeRunnable) throws InterruptedException {
        this.nodeRunnable = nodeRunnable;
        this.opentcsNode = nodeRunnable.getNode();
        changeNodeStatus(ACTIVE);
    }

    @Override
    public void onNodeStopped() {
        changeNodeStatus(NOT_ACTIVE);
        this.nodeRunnable = null;
        this.opentcsNode = null;
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
            nodeStarterListener.onNodeStarted(nodeRunnable);
        }
    }

    /**
     * Runnable for the node instance.
     */
    protected static class NodeRunnable implements Runnable {
        @Getter
        private OpenTcsNode node;
        private NodeMessageListener nodeMessageListener;
        private NodeStarterListener nodeStarterListener;
        private SingleThreadedExecutor executor;

        public NodeRunnable(NodeMessageListener nodeMessageListener,  NodeStarterListener nodeStarterListener) {
            this.nodeMessageListener = nodeMessageListener;
            this.nodeStarterListener = nodeStarterListener;
        }

        @Override
        public void run() {
            RCLJava.rclJavaInit();
            executor = new SingleThreadedExecutor();
            this.node = new OpenTcsNode(nodeMessageListener);
            executor.addNode(node);
            executor.spin();
        }

        public void stop(){
            this.node.shutdown();
            this.executor.removeNode(node); // Remove the (stopped) node, otherwise it is still shown in the node list.

            this.node = null;
            this.executor = null;
            nodeStarterListener.onNodeStopped();
        }
    }

    private void changeNodeStatus(NodeRunningStatus newNodeRunningStatus) {
        this.nodeRunningStatus = newNodeRunningStatus;
        nodeRunningStatusListener.onNodeStatusChange(newNodeRunningStatus);
    }
}
