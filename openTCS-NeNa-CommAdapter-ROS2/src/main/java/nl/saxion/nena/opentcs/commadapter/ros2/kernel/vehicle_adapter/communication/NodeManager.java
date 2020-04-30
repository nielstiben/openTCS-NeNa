package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.constants.NodeRunningStatus;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executors.SingleThreadedExecutor;

import javax.annotation.Nonnull;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.constants.NodeRunningStatus.*;

/**
 * Helper class to start a new ROS2 node in a separate thread.
 * Starting a new ROS2 node takes time so a non-blocking callback
 * is used to notify other instances when a ROS2 node has been initiated.
 *
 * @author Niels Tiben
 */
@NoArgsConstructor
public class NodeManager implements NodeRunnableListener {
    private NodeRunningStatusListener nodeRunningStatusListener;
    private NodeRunnable nodeRunnable;
    @Getter
    private Node node;
    @Getter
    private NodeRunningStatus nodeRunningStatus = NOT_ACTIVE;

    /**
     * Start a node in a separate thread, that is responsible for communication with a ROS2 vehicle.
     *
     * @param nodeRunningStatusListener Listener for notifying updates regarding the node running status.
     * @param nodeMessageListener       Listener for notifying about new incoming node messages (e.g. new amcl_pose).
     * @param namespace                 The ROS2 namespace that the node should use. Used for distinguishing multiple vehicles.
     */
    public void start(
            @Nonnull NodeRunningStatusListener nodeRunningStatusListener,
            @Nonnull NodeMessageListener nodeMessageListener,
            @Nonnull String namespace) {
        assert this.nodeRunningStatus == NodeRunningStatus.NOT_ACTIVE;
        this.nodeRunningStatusListener = nodeRunningStatusListener;

        changeNodeStatus(INITIATING);

        NodeRunnable nodeRunnable = new NodeRunnable(nodeMessageListener, this, namespace);
        new Thread(nodeRunnable).start();
    }

    /**
     * Method that get's called when a node has been effectively initiated after start() has been called.
     *
     * @param nodeRunnable The runnable instance of the just created node.
     */
    @Override
    public void onNodeStarted(@Nonnull NodeRunnable nodeRunnable) {
        this.nodeRunnable = nodeRunnable;
        this.node = nodeRunnable.getNode();
        changeNodeStatus(ACTIVE);
    }

    /**
     * Terminate the node instance.
     */
    public void stop() {
        assert this.nodeRunningStatus == ACTIVE; // Only stopping active nodes can be stopped.
        changeNodeStatus(TERMINATING);

        this.nodeRunnable.stop();

    }

    /**
     * Method that get's called when a node has been effectively terminated.
     */
    @Override
    public void onNodeStopped() {
        changeNodeStatus(NOT_ACTIVE);
        this.nodeRunnable = null;
        this.node = null;
    }

    /**
     * Helper method to change the node's running status.
     *
     * @param newNodeRunningStatus The updated node running status.
     */
    private void changeNodeStatus(NodeRunningStatus newNodeRunningStatus) {
        this.nodeRunningStatus = newNodeRunningStatus;
        nodeRunningStatusListener.onNodeRunningStatusUpdate(newNodeRunningStatus);
    }

    /**
     * Runnable for the node instance.
     */
    protected static class NodeRunnable implements Runnable {
        @Getter
        private Node node;
        private SingleThreadedExecutor executor;

        private final NodeMessageListener nodeMessageListener;
        private final NodeRunnableListener nodeRunnableListener;
        private final String nodeNamespace;

        /**
         * Constructor
         *
         * @param nodeMessageListener  Listener for notifying about new incoming node messages (e.g. new amcl_pose).
         * @param nodeRunnableListener Listener for notifying the NodeManager when a node has been effectively started or stopped.
         * @param nodeNamespace        The ROS2 namespace that the node should use. Used for distinguishing multiple vehicles.
         */
        public NodeRunnable(
                @Nonnull NodeMessageListener nodeMessageListener,
                @Nonnull NodeRunnableListener nodeRunnableListener,
                @Nonnull String nodeNamespace
        ) {
            this.nodeMessageListener = nodeMessageListener;
            this.nodeRunnableListener = nodeRunnableListener;
            this.nodeNamespace = nodeNamespace;
        }

        @SneakyThrows
        @Override
        public void run() {
            RCLJava.rclJavaInit();
            this.executor = new SingleThreadedExecutor();
            this.node = new Node(this.nodeMessageListener, this.nodeNamespace);
            this.executor.addNode(this.node);
            this.nodeRunnableListener.onNodeStarted(this);

            // spin() is blocking and keeps running until stop() is called.
            this.executor.spin();
        }

        public void stop() {
            RCLJava.shutdown();
            this.executor.removeNode(this.node); // Remove the (stopped) node, otherwise it is still shown in the node list.
            this.nodeRunnableListener.onNodeStopped();
        }
    }
}
