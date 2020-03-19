/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import com.google.common.primitives.Ints;
import com.google.inject.assistedinject.Assisted;
import nl.saxion.nena.opentcs.commadapter.ros2.comm.VehicleTelegramDecoder;
import nl.saxion.nena.opentcs.commadapter.ros2.comm.VehicleTelegramEncoder;
import nl.saxion.nena.opentcs.commadapter.ros2.exchange.ExampleProcessModelTO;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderRequest;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderResponse;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateRequest;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateResponse;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateResponse.LoadState;
import nl.saxion.nena.opentcs.common.dispatching.LoadAction;
import nl.saxion.nena.opentcs.common.telegrams.BoundedCounter;
import static nl.saxion.nena.opentcs.common.telegrams.BoundedCounter.UINT16_MAX_VALUE;
import nl.saxion.nena.opentcs.common.telegrams.Request;
import nl.saxion.nena.opentcs.common.telegrams.RequestResponseMatcher;
import nl.saxion.nena.opentcs.common.telegrams.Response;
import nl.saxion.nena.opentcs.common.telegrams.StateRequesterTask;
import nl.saxion.nena.opentcs.common.telegrams.Telegram;
import nl.saxion.nena.opentcs.common.telegrams.TelegramSender;
import io.netty.channel.ChannelHandler;
import io.netty.handler.codec.LengthFieldBasedFrameDecoder;
import java.beans.PropertyChangeEvent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import static java.util.Objects.requireNonNull;
import java.util.concurrent.ConcurrentHashMap;
import javax.inject.Inject;
import org.opentcs.contrib.tcp.netty.ConnectionEventListener;
import org.opentcs.contrib.tcp.netty.TcpClientChannelManager;
import org.opentcs.data.model.Vehicle;
import org.opentcs.data.order.DriveOrder;
import org.opentcs.drivers.vehicle.BasicVehicleCommAdapter;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;
import org.opentcs.util.ExplainedBoolean;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * An example implementation for a communication adapter.
 *
 * @author Mats Wilhelm (Fraunhofer IML)
 */
public class ExampleCommAdapter
    extends BasicVehicleCommAdapter
    implements ConnectionEventListener<Response>,
               TelegramSender {

  /**
   * This class's logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(ExampleCommAdapter.class);
  /**
   * Maps movement commands from openTCS to the telegrams sent to the attached vehicle.
   */
  private final OrderMapper orderMapper;
  /**
   * The components factory.
   */
  private final ExampleAdapterComponentsFactory componentsFactory;
  /**
   * Manages counting the ids for all {@link Request} telegrams.
   */
  private final BoundedCounter globalRequestCounter = new BoundedCounter(0, UINT16_MAX_VALUE);
  /**
   * Maps commands to order IDs so we know which command to report as finished.
   */
  private final Map<MovementCommand, Integer> orderIds = new ConcurrentHashMap<>();
  /**
   * Manages the channel to the vehicle.
   */
  private TcpClientChannelManager<Request, Response> vehicleChannelManager;
  /**
   * Matches requests to responses and holds a queue for pending requests.
   */
  private RequestResponseMatcher requestResponseMatcher;
  /**
   * A task for enqueuing state requests periodically.
   */
  private StateRequesterTask stateRequesterTask;

  /**
   * Creates a new instance.
   *
   * @param vehicle The attached vehicle.
   * @param orderMapper The order mapper for movement commands.
   * @param componentsFactory The components factory.
   */
  @Inject
  public ExampleCommAdapter(@Assisted Vehicle vehicle,
                            OrderMapper orderMapper,
                            ExampleAdapterComponentsFactory componentsFactory) {
    super(new ExampleProcessModel(vehicle), 3, 2, LoadAction.CHARGE);
    this.orderMapper = requireNonNull(orderMapper, "orderMapper");
    this.componentsFactory = requireNonNull(componentsFactory, "componentsFactory");
  }

  @Override
  public void initialize() {
    super.initialize();
    this.requestResponseMatcher = componentsFactory.createRequestResponseMatcher(this);
    this.stateRequesterTask = componentsFactory.createStateRequesterTask(e -> {
      LOG.debug("Adding new state requests to the queue.");
      requestResponseMatcher.enqueueRequest(new StateRequest(Telegram.ID_DEFAULT));
    });
  }

  @Override
  public void terminate() {
    stateRequesterTask.disable();
    super.terminate();
  }

  @Override
  public synchronized void enable() {
    if (isEnabled()) {
      return;
    }

    //Create the channel manager responsible for connections with the vehicle
    vehicleChannelManager = new TcpClientChannelManager<>(this,
                                                          this::getChannelHandlers,
                                                          getProcessModel().getVehicleIdleTimeout(),
                                                          getProcessModel().isLoggingEnabled());
    //Initialize the channel manager
    vehicleChannelManager.initialize();
    super.enable();
  }

  @Override
  public synchronized void disable() {
    if (!isEnabled()) {
      return;
    }

    super.disable();
    vehicleChannelManager.terminate();
    vehicleChannelManager = null;
  }

  @Override
  public synchronized void clearCommandQueue() {
    super.clearCommandQueue();
    orderIds.clear();
  }

  @Override
  protected synchronized void connectVehicle() {
    if (vehicleChannelManager == null) {
      LOG.warn("{}: VehicleChannelManager not present.", getName());
      return;
    }

    vehicleChannelManager.connect(getProcessModel().getVehicleHost(),
                                  getProcessModel().getVehiclePort());
  }

  @Override
  protected synchronized void disconnectVehicle() {
    if (vehicleChannelManager == null) {
      LOG.warn("{}: VehicleChannelManager not present.", getName());
      return;
    }

    vehicleChannelManager.disconnect();
  }

  @Override
  protected synchronized boolean isVehicleConnected() {
    return vehicleChannelManager != null && vehicleChannelManager.isConnected();
  }

  @Override
  public void propertyChange(PropertyChangeEvent evt) {
    super.propertyChange(evt);
    if (!(evt.getSource() instanceof ExampleProcessModel)) {
      return;
    }

    // Handling of events from the vehicle gui panels start here
    if (Objects.equals(evt.getPropertyName(),
                       VehicleProcessModel.Attribute.COMM_ADAPTER_CONNECTED.name())) {
      if (getProcessModel().isCommAdapterConnected()) {
        // Once the connection is established, ensure that logging is enabled/disabled for it as
        // configured by the user.
        vehicleChannelManager.setLoggingEnabled(getProcessModel().isLoggingEnabled());
      }
    }
    if (Objects.equals(evt.getPropertyName(),
                       VehicleProcessModel.Attribute.COMM_ADAPTER_CONNECTED.name())
        || Objects.equals(evt.getPropertyName(),
                          ExampleProcessModel.Attribute.PERIODIC_STATE_REQUESTS_ENABLED.name())) {
      if (getProcessModel().isCommAdapterConnected()
          && getProcessModel().isPeriodicStateRequestEnabled()) {
        stateRequesterTask.enable();
      }
      else {
        stateRequesterTask.disable();
      }
    }
    if (Objects.equals(evt.getPropertyName(),
                       ExampleProcessModel.Attribute.PERIOD_STATE_REQUESTS_INTERVAL.name())) {
      stateRequesterTask.setRequestInterval(getProcessModel().getStateRequestInterval());
    }
  }

  @Override
  @Deprecated
  protected List<org.opentcs.drivers.vehicle.VehicleCommAdapterPanel> createAdapterPanels() {
    return new ArrayList<>();
  }

  @Override
  public final ExampleProcessModel getProcessModel() {
    return (ExampleProcessModel) super.getProcessModel();
  }

  @Override
  protected VehicleProcessModelTO createCustomTransferableProcessModel() {
    //Add extra information of the vehicle when sending to other software like control center or 
    //plant overview
    return new ExampleProcessModelTO()
        .setVehicleRef(getProcessModel().getVehicleReference())
        .setCurrentState(getProcessModel().getCurrentState())
        .setPreviousState(getProcessModel().getPreviousState())
        .setLastOrderSent(getProcessModel().getLastOrderSent())
        .setDisconnectingOnVehicleIdle(getProcessModel().isDisconnectingOnVehicleIdle())
        .setLoggingEnabled(getProcessModel().isLoggingEnabled())
        .setReconnectDelay(getProcessModel().getReconnectDelay())
        .setReconnectingOnConnectionLoss(getProcessModel().isReconnectingOnConnectionLoss())
        .setVehicleHost(getProcessModel().getVehicleHost())
        .setVehicleIdle(getProcessModel().isVehicleIdle())
        .setVehicleIdleTimeout(getProcessModel().getVehicleIdleTimeout())
        .setVehiclePort(getProcessModel().getVehiclePort())
        .setPeriodicStateRequestEnabled(getProcessModel().isPeriodicStateRequestEnabled())
        .setStateRequestInterval(getProcessModel().getStateRequestInterval());
  }

  @Override
  public synchronized void sendCommand(MovementCommand cmd)
      throws IllegalArgumentException {
    requireNonNull(cmd, "cmd");

    try {
      OrderRequest telegram = orderMapper.mapToOrder(cmd);
      orderIds.put(cmd, telegram.getOrderId());
      LOG.debug("{}: Enqueuing order telegram with ID {}: {}, {}",
                getName(),
                telegram.getOrderId(),
                telegram.getDestinationId(),
                telegram.getDestinationAction());

      // Add the telegram to the queue. Telegram will be send later when its the first telegram in 
      // the queue. This ensures that we always wait for a response until we send a new request.
      requestResponseMatcher.enqueueRequest(telegram);
      LOG.debug("{}: Finished enqueuing order telegram.", getName());
    }
    catch (IllegalArgumentException exc) {
      LOG.error("{}: Failed to enqueue command {}", getName(), cmd, exc);
    }
  }

  @Override
  public synchronized ExplainedBoolean canProcess(List<String> operations) {
    requireNonNull(operations, "operations");
    boolean canProcess = true;
    String reason = "";
    if (!isEnabled()) {
      canProcess = false;
      reason = "Adapter not enabled";
    }
    if (canProcess && !isVehicleConnected()) {
      canProcess = false;
      reason = "Vehicle does not seem to be connected";
    }
    if (canProcess
        && getProcessModel().getCurrentState().getLoadState() == LoadState.UNKNOWN) {
      canProcess = false;
      reason = "Vehicle's load state is undefined";
    }
    boolean loaded = getProcessModel().getCurrentState().getLoadState() == LoadState.FULL;
    final Iterator<String> opIter = operations.iterator();
    while (canProcess && opIter.hasNext()) {
      final String nextOp = opIter.next();
      // If we're loaded, we cannot load another piece, but could unload.
      if (loaded) {
        if (nextOp.startsWith(LoadAction.LOAD)) {
          canProcess = false;
          reason = "Cannot load when already loaded";
        }
        else if (nextOp.startsWith(LoadAction.UNLOAD)) {
          loaded = false;
        }
        else if (nextOp.startsWith(DriveOrder.Destination.OP_PARK)) {
          canProcess = false;
          reason = "Vehicle shouldn't park while in a loaded state.";
        }
        else if (nextOp.startsWith(LoadAction.CHARGE)) {
          canProcess = false;
          reason = "Vehicle shouldn't charge while in a loaded state.";
        }
      }
      // If we're not loaded, we could load, but not unload.
      else if (nextOp.startsWith(LoadAction.LOAD)) {
        loaded = true;
      }
      else if (nextOp.startsWith(LoadAction.UNLOAD)) {
        canProcess = false;
        reason = "Cannot unload when not loaded";
      }
    }
    return new ExplainedBoolean(canProcess, reason);
  }

  @Override
  public void processMessage(Object message) {
    //Process messages sent from the kernel or a kernel extension
  }

  @Override
  public void onConnect() {
    if (!isEnabled()) {
      return;
    }
    LOG.debug("{}: connected", getName());
    getProcessModel().setCommAdapterConnected(true);
    // Check for resending last request
    requestResponseMatcher.checkForSendingNextRequest();
  }

  @Override
  public void onFailedConnectionAttempt() {
    if (!isEnabled()) {
      return;
    }
    getProcessModel().setCommAdapterConnected(false);
    if (isEnabled() && getProcessModel().isReconnectingOnConnectionLoss()) {
      vehicleChannelManager.scheduleConnect(getProcessModel().getVehicleHost(),
                                            getProcessModel().getVehiclePort(),
                                            getProcessModel().getReconnectDelay());
    }
  }

  @Override
  public void onDisconnect() {
    LOG.debug("{}: disconnected", getName());
    getProcessModel().setCommAdapterConnected(false);
    getProcessModel().setVehicleIdle(true);
    getProcessModel().setVehicleState(Vehicle.State.UNKNOWN);
    if (isEnabled() && getProcessModel().isReconnectingOnConnectionLoss()) {
      vehicleChannelManager.scheduleConnect(getProcessModel().getVehicleHost(),
                                            getProcessModel().getVehiclePort(),
                                            getProcessModel().getReconnectDelay());
    }
  }

  @Override
  public void onIdle() {
    LOG.debug("{}: idle", getName());
    getProcessModel().setVehicleIdle(true);
    // If we are supposed to reconnect automatically, do so.
    if (isEnabled() && getProcessModel().isDisconnectingOnVehicleIdle()) {
      LOG.debug("{}: Disconnecting on idle timeout...", getName());
      disconnectVehicle();
    }
  }

  @Override
  public synchronized void onIncomingTelegram(Response response) {
    requireNonNull(response, "response");

    // Remember that we have received a sign of life from the vehicle
    getProcessModel().setVehicleIdle(false);

    //Check if the response matches the current request
    if (!requestResponseMatcher.tryMatchWithCurrentRequest(response)) {
      // XXX Either ignore the message or close the connection
      return;
    }

    if (response instanceof StateResponse) {
      onStateResponse((StateResponse) response);
    }
    else if (response instanceof OrderResponse) {
      LOG.debug("{}: Received a new order response: {}", getName(), response);
    }
    else {
      LOG.warn("{}: Unhandled telegram implementation: {}",
               getName(),
               response.getClass().getName());
    }

    //Send the next telegram if one is waiting
    requestResponseMatcher.checkForSendingNextRequest();
  }

  @Override
  public synchronized void sendTelegram(Request telegram) {
    requireNonNull(telegram, "telegram");
    if (!isVehicleConnected()) {
      LOG.debug("{}: Not connected - not sending request '{}'",
                getName(),
                telegram);
      return;
    }

    // Update the request's id
    telegram.updateRequestContent(globalRequestCounter.getAndIncrement());

    LOG.debug("{}: Sending request '{}'", getName(), telegram);
    vehicleChannelManager.send(telegram);

    // If the telegram is an order, remember it.
    if (telegram instanceof OrderRequest) {
      getProcessModel().setLastOrderSent((OrderRequest) telegram);
    }

    if (getProcessModel().isPeriodicStateRequestEnabled()) {
      stateRequesterTask.restart();
    }
  }

  public RequestResponseMatcher getRequestResponseMatcher() {
    return requestResponseMatcher;
  }

  private void onStateResponse(StateResponse stateResponse) {
    LOG.debug("{}: Received a new state response: {}", getName(), stateResponse);

    // Update the vehicle's current state and remember the old one.
    getProcessModel().setPreviousState(getProcessModel().getCurrentState());
    getProcessModel().setCurrentState(stateResponse);

    StateResponse previousState = getProcessModel().getPreviousState();
    StateResponse currentState = getProcessModel().getCurrentState();

    checkForVehiclePositionUpdate(previousState, currentState);
    checkForVehicleStateUpdate(previousState, currentState);
    checkOrderFinished(previousState, currentState);

    // XXX Process further state updates extracted from the telegram here.
  }

  private void checkForVehiclePositionUpdate(StateResponse previousState,
                                             StateResponse currentState) {
    if (previousState.getPositionId() == currentState.getPositionId()) {
      return;
    }
    // Map the reported position ID to a point name.
    String currentPosition = String.valueOf(currentState.getPositionId());
    LOG.debug("{}: Vehicle is now at point {}", getName(), currentPosition);
    // Update the position with the rest of the system, but only if it's not zero (unknown).
    if (currentState.getPositionId() != 0) {
      getProcessModel().setVehiclePosition(currentPosition);
    }
  }

  private void checkForVehicleStateUpdate(StateResponse previousState,
                                          StateResponse currentState) {
    if (previousState.getOperatingState() == currentState.getOperatingState()) {
      return;
    }
    getProcessModel().setVehicleState(translateVehicleState(currentState.getOperatingState()));
  }

  private void checkOrderFinished(StateResponse previousState, StateResponse currentState) {
    if (currentState.getLastFinishedOrderId() == 0) {
      return;
    }
    // If the last finished order ID hasn't changed, don't bother.
    if (previousState.getLastFinishedOrderId() == currentState.getLastFinishedOrderId()) {
      return;
    }
    // Check if the new finished order ID is in the queue of sent orders.
    // If yes, report all orders up to that one as finished.
    if (!orderIds.containsValue(currentState.getLastFinishedOrderId())) {
      LOG.debug("{}: Ignored finished order ID {} (reported by vehicle, not found in sent queue).",
                getName(),
                currentState.getLastFinishedOrderId());
      return;
    }

    Iterator<MovementCommand> cmdIter = getSentQueue().iterator();
    boolean finishedAll = false;
    while (!finishedAll && cmdIter.hasNext()) {
      MovementCommand cmd = cmdIter.next();
      cmdIter.remove();
      int orderId = orderIds.remove(cmd);
      if (orderId == currentState.getLastFinishedOrderId()) {
        finishedAll = true;
      }

      LOG.debug("{}: Reporting command with order ID {} as executed: {}", getName(), orderId, cmd);
      getProcessModel().commandExecuted(cmd);
    }
  }

  /**
   * Map the vehicle's operation states to the kernel's vehicle states.
   *
   * @param operationState The vehicle's current operation state.
   */
  private Vehicle.State translateVehicleState(StateResponse.OperatingState operationState) {
    switch (operationState) {
      case IDLE:
        return Vehicle.State.IDLE;
      case MOVING:
      case ACTING:
        return Vehicle.State.EXECUTING;
      case CHARGING:
        return Vehicle.State.CHARGING;
      case ERROR:
        return Vehicle.State.ERROR;
      default:
        return Vehicle.State.UNKNOWN;
    }
  }

  /**
   * Returns the channel handlers responsible for writing and reading from the byte stream.
   *
   * @return The channel handlers responsible for writing and reading from the byte stream
   */
  private List<ChannelHandler> getChannelHandlers() {
    return Arrays.asList(new LengthFieldBasedFrameDecoder(getMaxTelegramLength(), 1, 1, 2, 0),
                         new VehicleTelegramDecoder(this),
                         new VehicleTelegramEncoder());
  }

  private int getMaxTelegramLength() {
    return Ints.max(OrderResponse.TELEGRAM_LENGTH,
                    StateResponse.TELEGRAM_LENGTH);
  }
}
