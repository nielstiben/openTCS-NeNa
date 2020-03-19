/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.simulation;

import com.google.common.base.Strings;
import com.google.common.primitives.Ints;
import nl.saxion.nena.opentcs.commadapter.ros2.ExampleCommAdapter;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderRequest;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateRequest;
import io.netty.channel.ChannelHandler;
import io.netty.handler.codec.LengthFieldBasedFrameDecoder;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.opentcs.contrib.tcp.netty.ClientEntry;
import org.opentcs.contrib.tcp.netty.ConnectionEventListener;
import org.opentcs.contrib.tcp.netty.TcpServerChannelManager;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * A standalone application to simulate communication between the {@link ExampleCommAdapter} and a
 * vehicle.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class VehicleSimulator
    implements ConnectionEventListener<byte[]> {

  /**
   * This class's logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(VehicleSimulator.class);
  /**
   * An idendifier for the client that connects to this vehicle.
   */
  public static final Object CLIENT_OBJECT = new Object();
  /**
   * The pool of clients to connect to this vehicle.
   * Here it's only one client.
   */
  private final Map<Object, ClientEntry<byte[]>> client = new HashMap<>();
  /**
   * Manages the connection to the {@link ExampleCommAdapter}.
   */
  private final TcpServerChannelManager<byte[], byte[]> vehicleServer;
  /**
   * The executor for the simluation.
   */
  private final ScheduledExecutorService simultationExecutor = Executors
      .newSingleThreadScheduledExecutor(runnable -> new Thread(runnable, "simulationExecutor"));
  /**
   * The executor for specific tasks.
   */
  private final ScheduledExecutorService taskExecutor = Executors
      .newSingleThreadScheduledExecutor(runnable -> new Thread(runnable, "taskExecutor"));

  /**
   * The internal state of the simulated vehicle.
   */
  private final VehicleState vehicleState = new VehicleState();

  /**
   * Creates a new instance.
   */
  public VehicleSimulator() {
    vehicleServer = new TcpServerChannelManager<>(2000,
                                                  client,
                                                  this::getChannelHandlers,
                                                  5000,
                                                  true);
  }

  private void initialize() {
    if (vehicleServer.isInitialized()) {
      return;
    }
    vehicleServer.initialize();
    vehicleServer.register(CLIENT_OBJECT, this, true);
  }

  private void terminate() {
    if (!vehicleServer.isInitialized()) {
      return;
    }
    vehicleServer.terminate();
  }

  @Override
  public void onIncomingTelegram(byte[] request) {
    LOG.info("Incoming request: {}", request);
    if (request[2] == StateRequest.TYPE) {
      int telegramCounter = Ints.fromBytes((byte) 0, (byte) 0, request[3], request[4]);
      vehicleState.setTelegramCounter(telegramCounter);
      byte[] response = createStateResponse();
      LOG.info("Sending response: {}", response);
      vehicleServer.send(CLIENT_OBJECT, response);
    }
    else if (request[2] == OrderRequest.TYPE) {
      int telegramCounter = Ints.fromBytes((byte) 0, (byte) 0, request[3], request[4]);
      vehicleState.setTelegramCounter(telegramCounter);
      int orderID = Ints.fromBytes((byte) 0, (byte) 0, request[5], request[6]);
      vehicleState.setLastReceivedOrderId(orderID);
      byte[] response = createOrderResponse();
      LOG.info("Sending response: {}", response);
      vehicleServer.send(CLIENT_OBJECT, response);
    }
  }

  @Override
  public void onConnect() {
    LOG.info("Communication adapter connected to vehicle.");
    configureVehicleBehaviour();
  }

  @Override
  public void onFailedConnectionAttempt() {
  }

  @Override
  public void onDisconnect() {
    LOG.info("Communication adapter disconnected from vehicle.");
    terminate();
    initialize();
  }

  @Override
  public void onIdle() {
    LOG.info("Communucation adapter is idle.");
  }

  private void startSimulationThread() {
    Runnable simulationTask = () -> {
      LOG.info("Starting simulation... (press the return key to stop the simulation)");
      Scanner scanner = new Scanner(System.in);
      boolean input = false;
      while (!input) {
        input = Strings.isNullOrEmpty(scanner.nextLine());
      }
      LOG.info("Stopping simulation...");
      terminate();
      System.exit(0);
    };
    simultationExecutor.schedule(simulationTask, 0, TimeUnit.SECONDS);
  }

  /**
   * Initializes the vehicle state.
   */
  private void configureVehicleBehaviour() {
    taskExecutor.schedule(setOperationMode('A'), 0, TimeUnit.SECONDS);
    taskExecutor.schedule(setOperationState('M'), 0, TimeUnit.SECONDS);
    taskExecutor.schedule(setPositionId(7089), 0, TimeUnit.SECONDS);

    taskExecutor.schedule(setOperationState('A'), 10, TimeUnit.SECONDS);
    taskExecutor.schedule(setOperationState('I'), 20, TimeUnit.SECONDS);
    taskExecutor.schedule(setOperationState('C'), 22, TimeUnit.SECONDS);
  }

  /**
   * Creates a state response from the simulated vehicle state.
   *
   * @return The byte representation of a state response
   */
  private byte[] createStateResponse() {
    return vehicleState.toStateResponse().getRawContent();
  }

  /**
   * Creates an order response from the simulated vehicle state.
   *
   * @return The byte representation of a state response
   */
  private byte[] createOrderResponse() {
    return vehicleState.toOrderResponse().getRawContent();
  }

  /**
   * Returns the channel handlers for the nio pipeline.
   *
   * @return The channel handlers for the nio pipeline
   */
  private List<ChannelHandler> getChannelHandlers() {
    return Arrays.asList(new LengthFieldBasedFrameDecoder(getMaxTelegramLength(), 1, 1, 2, 0),
                         new TelegramDecoder(),
                         new TelegramEncoder(),
                         new ConnectionAssociator(client));
  }

  /**
   * Returns the maximum telegram length.
   *
   * @return The maximum telegram length
   */
  private int getMaxTelegramLength() {
    return Ints.max(StateRequest.TELEGRAM_LENGTH,
                    OrderRequest.TELEGRAM_LENGTH);
  }

  /**
   * Starts the simulation of the vehicle.
   *
   * @param args
   */
  public static void main(String[] args) {
    VehicleSimulator simulator = new VehicleSimulator();
    simulator.initialize();
    simulator.startSimulationThread();
  }

  private Runnable setPositionId(int positionId) {
    return () -> vehicleState.setPositionId(positionId);
  }

  private Runnable setOperationMode(char operationMode) {
    return () -> vehicleState.setOperationMode(operationMode);
  }

  private Runnable setOperationState(char operationState) {
    return () -> vehicleState.setOperationMode(operationState);
  }
}
