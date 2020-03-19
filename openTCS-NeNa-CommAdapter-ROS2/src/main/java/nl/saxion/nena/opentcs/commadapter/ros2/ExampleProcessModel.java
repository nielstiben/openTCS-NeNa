/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import static com.google.common.base.Preconditions.checkArgument;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderRequest;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateRequest;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateResponse;
import static java.util.Objects.requireNonNull;
import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import static org.opentcs.util.Assertions.checkInRange;

/**
 * A custom model for the {@link ExampleCommAdapter} which holds additional information
 * about the connected vehicle.
 *
 * @author Mats Wilhelm (Fraunhofer IML)
 */
public class ExampleProcessModel
    extends VehicleProcessModel {

  /**
   * The current/most recent state reported by the vehicle.
   */
  private StateResponse currentState;
  /**
   * The previous state reported by the vehicle.
   */
  private StateResponse previousState;
  /**
   * The last order telegram sent to the vehicle.
   */
  private OrderRequest lastOrderSent;
  /**
   * The host to connect to.
   */
  private String vehicleHost = "127.0.0.1";
  /**
   * The port to connect to.
   */
  private int vehiclePort = 2000;
  /**
   * A flag indicating whether periodic sending of {@link StateRequest} telegrams is enabled.
   */
  private boolean periodicStateRequestEnabled = true;
  /**
   * The time to wait between periodic state request telegrams.
   */
  private int stateRequestInterval = 500;
  /**
   * How long (in ms) we tolerate not hearing from the vehicle before we consider communication
   * dead.
   */
  private int vehicleIdleTimeout = 5000;
  /**
   * Indicates whether the vehicle has not been heard of recently.
   */
  private boolean vehicleIdle = true;
  /**
   * Whether to close the connection if the vehicle is considered dead.
   */
  private boolean disconnectingOnVehicleIdle = true;
  /**
   * Whether to reconnect automatically when the vehicle connection times out.
   */
  private boolean reconnectingOnConnectionLoss = true;
  /**
   * The delay before reconnecting (in ms).
   */
  private int reconnectDelay = 10000;
  /**
   * Whether logging should be enabled or not.
   */
  private boolean loggingEnabled = false;

  /**
   * Creates a new instance.
   *
   * @param attachedVehicle The attached vehicle
   */
  public ExampleProcessModel(Vehicle attachedVehicle) {
    super(attachedVehicle);
    // Initialize the state fields
    final byte[] dummyData = new byte[StateResponse.TELEGRAM_LENGTH];
    previousState = new StateResponse(dummyData);
    currentState = new StateResponse(dummyData);
  }

  @Nonnull
  public StateResponse getCurrentState() {
    return currentState;
  }

  public void setCurrentState(@Nonnull StateResponse currentState) {
    StateResponse oldValue = this.currentState;
    this.currentState = requireNonNull(currentState, "currentState");

    getPropertyChangeSupport().firePropertyChange(Attribute.CURRENT_STATE.name(),
                                                  oldValue,
                                                  currentState);
  }

  @Nonnull
  public StateResponse getPreviousState() {
    return previousState;
  }

  public void setPreviousState(@Nonnull StateResponse previousState) {
    StateResponse oldValue = this.previousState;
    this.previousState = requireNonNull(previousState, "previousState");

    getPropertyChangeSupport().firePropertyChange(Attribute.PREVIOUS_STATE.name(),
                                                  oldValue,
                                                  previousState);
  }

  /**
   * Returns the last order telegram sent to the vehicle.
   *
   * @return The last order telegram sent to the vehicle, or <code>null</code>, if no order has been
   * sent to the vehicle, yet.
   */
  @Nullable
  public synchronized OrderRequest getLastOrderSent() {
    return lastOrderSent;
  }

  public synchronized void setLastOrderSent(@Nullable OrderRequest telegram) {
    OrderRequest oldValue = this.lastOrderSent;
    lastOrderSent = telegram;

    getPropertyChangeSupport().firePropertyChange(Attribute.LAST_ORDER.name(),
                                                  oldValue,
                                                  telegram);
  }

  /**
   * Returns the vehicle's host name/IP address.
   *
   * @return The vehicle's host name/IP address.
   */
  @Nonnull
  public synchronized String getVehicleHost() {
    return vehicleHost;
  }

  /**
   * Sets the vehicle's host name/IP address.
   *
   * @param vehicleHost The vehicle's host name/IP address.
   */
  public synchronized void setVehicleHost(@Nonnull String vehicleHost) {
    String oldValue = this.vehicleHost;
    this.vehicleHost = requireNonNull(vehicleHost, "vehicleHost");

    getPropertyChangeSupport().firePropertyChange(Attribute.VEHICLE_HOST.name(),
                                                  oldValue,
                                                  vehicleHost);
  }

  /**
   * Returns the TCP port number the vehicle is listening on.
   *
   * @return The TCP port number the vehicle is listening on.
   */
  public synchronized int getVehiclePort() {
    return vehiclePort;
  }

  /**
   * Sets the TCP port number the vehicle is listening on.
   *
   * @param vehiclePort The TCP port number.
   */
  public synchronized void setVehiclePort(int vehiclePort) {
    int oldValue = this.vehiclePort;
    this.vehiclePort = checkInRange(vehiclePort, 1, 65535, "vehiclePort");

    getPropertyChangeSupport().firePropertyChange(Attribute.VEHICLE_PORT.name(),
                                                  oldValue,
                                                  vehiclePort);
  }

  /**
   * Indicates whether the communication adapter periodically sends state requests to the vehicle.
   *
   * @return <code>true</code> if, and only if, the communication adapter periodically sends state
   * requests to the vehicle.
   */
  public synchronized boolean isPeriodicStateRequestEnabled() {
    return periodicStateRequestEnabled;
  }

  /**
   * Turns periodic state requests on/off.
   *
   * @param enabled If <code>true</code>, periodic state requests are turned on, otherwise they are
   * turned off.
   */
  public synchronized void setPeriodicStateRequestEnabled(boolean enabled) {
    if (periodicStateRequestEnabled == enabled) {
      return;
    }
    boolean oldVal = periodicStateRequestEnabled;
    periodicStateRequestEnabled = enabled;
    getPropertyChangeSupport().firePropertyChange(Attribute.PERIODIC_STATE_REQUESTS_ENABLED.name(),
                                                  oldVal,
                                                  enabled);
  }

  /**
   * Returns the interval (in ms) between two state requests.
   *
   * @return The interval (in ms) between two state requests.
   */
  public synchronized int getStateRequestInterval() {
    return stateRequestInterval;
  }

  /**
   * Sets the interval (in ms) between two state requests.
   *
   * @param interval The interval to be set.
   */
  public synchronized void setStateRequestInterval(int interval) {
    checkArgument(interval >= 0, "interval invalid: %s", interval);
    if (stateRequestInterval == interval) {
      return;
    }
    int oldVal = stateRequestInterval;
    stateRequestInterval = interval;
    getPropertyChangeSupport().firePropertyChange(Attribute.PERIOD_STATE_REQUESTS_INTERVAL.name(),
                                                  oldVal,
                                                  interval);
  }

  /**
   * Returns how long (in ms) we tolerate not hearing from the vehicle before we consider
   * communication dead.
   *
   * @return How long (in ms) we tolerate not hearing from the vehicle before we consider
   * communication dead
   */
  public int getVehicleIdleTimeout() {
    return vehicleIdleTimeout;
  }

  /**
   * Sets how long (in ms) we tolerate not hearing from the vehicle before we consider
   * communication dead
   *
   * @param idleTimeout How long (in ms) we tolerate not hearing from the vehicle before we consider
   * communication dead
   */
  public void setVehicleIdleTimeout(int idleTimeout) {
    int oldValue = this.vehicleIdleTimeout;
    this.vehicleIdleTimeout = checkInRange(idleTimeout, 1, Integer.MAX_VALUE, "idleTimeout");

    getPropertyChangeSupport().firePropertyChange(Attribute.VEHICLE_IDLE_TIMEOUT.name(),
                                                  oldValue,
                                                  idleTimeout);
  }

  /**
   * Returns whether the vehicle has not been heard of recently.
   *
   * @return Whether the vehicle has not been heard of recently.
   */
  public boolean isVehicleIdle() {
    return vehicleIdle;
  }

  /**
   * Sets whether the vehicle has not been heard of recently.
   *
   * @param idle Whether the vehicle has not been heard of recently.
   */
  public void setVehicleIdle(boolean idle) {
    boolean oldValue = this.vehicleIdle;
    this.vehicleIdle = idle;

    getPropertyChangeSupport().firePropertyChange(Attribute.VEHICLE_IDLE.name(),
                                                  oldValue,
                                                  idle);
  }

  /**
   * Returns whether to close the connection if the vehicle is considered dead.
   *
   * @return Whether to close the connection if the vehicle is considered dead
   */
  public boolean isDisconnectingOnVehicleIdle() {
    return disconnectingOnVehicleIdle;
  }

  /**
   * Sets whether to close the connection if the vehicle is considered dead.
   *
   * @param disconnectingOnVehicleIdle Whether to close the connection if the vehicle is
   * considered dead
   */
  public void setDisconnectingOnVehicleIdle(boolean disconnectingOnVehicleIdle) {
    boolean oldValue = this.disconnectingOnVehicleIdle;
    this.disconnectingOnVehicleIdle = disconnectingOnVehicleIdle;

    getPropertyChangeSupport().firePropertyChange(Attribute.DISCONNECTING_ON_IDLE.name(),
                                                  oldValue,
                                                  disconnectingOnVehicleIdle);
  }

  /**
   * Returns whether to reconnect automatically when the vehicle connection times out.
   *
   * @return whether to reconnect automatically when the vehicle connection times out
   */
  public boolean isReconnectingOnConnectionLoss() {
    return reconnectingOnConnectionLoss;
  }

  /**
   * Sets whether to reconnect automatically when the vehicle connection times out
   *
   * @param reconnectingOnConnectionLoss whether to reconnect automatically when the vehicle
   * connection times out
   */
  public void setReconnectingOnConnectionLoss(boolean reconnectingOnConnectionLoss) {
    boolean oldValue = this.reconnectingOnConnectionLoss;
    this.reconnectingOnConnectionLoss = reconnectingOnConnectionLoss;

    getPropertyChangeSupport().firePropertyChange(Attribute.RECONNECTING_ON_CONNECTION_LOSS.name(),
                                                  oldValue,
                                                  reconnectingOnConnectionLoss);
  }

  /**
   * Returns the delay before reconnecting (in ms).
   *
   * @return The delay before reconnecting (in ms)
   */
  public int getReconnectDelay() {
    return reconnectDelay;
  }

  /**
   * Sets the delay before reconnecting (in ms).
   *
   * @param reconnectDelay The delay before reconnecting (in ms)
   */
  public void setReconnectDelay(int reconnectDelay) {
    int oldValue = this.reconnectDelay;
    this.reconnectDelay = checkInRange(reconnectDelay, 1, Integer.MAX_VALUE, "reconnectDelay");

    getPropertyChangeSupport().firePropertyChange(Attribute.RECONNECT_DELAY.name(),
                                                  oldValue,
                                                  reconnectDelay);
  }

  /**
   * Returns whether logging is enabled.
   *
   * @return Whether logging is enabled
   */
  public boolean isLoggingEnabled() {
    return loggingEnabled;
  }

  /**
   * Sets whether logging should be enabled or not.
   *
   * @param loggingEnabled Whether logging should be enabled or not
   */
  public void setLoggingEnabled(boolean loggingEnabled) {
    boolean oldValue = this.loggingEnabled;
    this.loggingEnabled = loggingEnabled;

    getPropertyChangeSupport().firePropertyChange(Attribute.LOGGING_ENABLED.name(),
                                                  oldValue,
                                                  loggingEnabled);
  }

  /**
   * Model attributes specific to this implementation.
   */
  public static enum Attribute {
    CURRENT_STATE,
    PREVIOUS_STATE,
    LAST_ORDER,
    VEHICLE_HOST,
    VEHICLE_PORT,
    PERIODIC_STATE_REQUESTS_ENABLED,
    PERIOD_STATE_REQUESTS_INTERVAL,
    VEHICLE_IDLE_TIMEOUT,
    VEHICLE_IDLE,
    DISCONNECTING_ON_IDLE,
    RECONNECTING_ON_CONNECTION_LOSS,
    LOGGING_ENABLED,
    RECONNECT_DELAY;
  }
}
