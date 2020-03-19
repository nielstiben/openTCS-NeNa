/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.exchange;

import nl.saxion.nena.opentcs.commadapter.ros2.ExampleProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderRequest;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateRequest;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateResponse;
import org.opentcs.data.TCSObjectReference;
import org.opentcs.data.model.Vehicle;
import org.opentcs.drivers.vehicle.management.VehicleProcessModelTO;

/**
 * A serializable representation of a {@link ExampleProcessModel}.
 * This TO can be sent to other applications responsible for displaying the state of the vehicle,
 * like the control center or the plant overview.
 *
 * @author Mats Wilhelm (Fraunhofer IML)
 */
public class ExampleProcessModelTO
    extends VehicleProcessModelTO {

  /**
   * The vehicle reference.
   */
  private TCSObjectReference<Vehicle> vehicleRef;
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
  private String vehicleHost;
  /**
   * The port to connect to.
   */
  private int vehiclePort;
  /**
   * A flag indicating whether periodic sending of {@link StateRequest} telegrams is enabled.
   */
  private boolean periodicStateRequestEnabled;
  /**
   * The time to wait between periodic state request telegrams.
   */
  private int stateRequestInterval;
  /**
   * How long (in ms) we tolerate not hearing from the vehicle before we consider communication
   * dead.
   */
  private int vehicleIdleTimeout;
  /**
   * Indicates whether the vehicle has not been heard of recently.
   */
  private boolean vehicleIdle;
  /**
   * Whether to close the connection if the vehicle is considered dead.
   */
  private boolean disconnectingOnVehicleIdle;
  /**
   * Whether to reconnect automatically when the vehicle connection times out.
   */
  private boolean reconnectingOnConnectionLoss;
  /**
   * The delay before reconnecting (in ms).
   */
  private int reconnectDelay;
  /**
   * Whether logging should be enabled or not.
   */
  private boolean loggingEnabled;

  /**
   * Returns the vehicle reference.
   *
   * @return The vehicle reference
   */
  public TCSObjectReference<Vehicle> getVehicleRef() {
    return vehicleRef;
  }

  /**
   * Sets the vehicle reference.
   *
   * @param vehicleRef The vehicle reference
   * @return This
   */
  public ExampleProcessModelTO setVehicleRef(TCSObjectReference<Vehicle> vehicleRef) {
    this.vehicleRef = vehicleRef;
    return this;
  }

  /**
   * Returns the current/most recent state reported by the vehicle.
   *
   * @return The current/most recent state reported by the vehicle
   */
  public StateResponse getCurrentState() {
    return currentState;
  }

  /**
   * Sets the current/most recent state reported by the vehicle.
   *
   * @param currentState The current/most recent state reported by the vehicle.
   * @return This
   */
  public ExampleProcessModelTO setCurrentState(
      StateResponse currentState) {
    this.currentState = currentState;
    return this;
  }

  /**
   * Returns the previous state reported by the vehicle.
   *
   * @return The previous state reported by the vehicle
   */
  public StateResponse getPreviousState() {
    return previousState;
  }

  /**
   * Sets the previous state reported by the vehicle.
   *
   * @param previousState The previous state reported by the vehicle
   * @return This
   */
  public ExampleProcessModelTO setPreviousState(
      StateResponse previousState) {
    this.previousState = previousState;
    return this;
  }

  /**
   * Returns the last order telegram sent to the vehicle.
   *
   * @return The last order telegram sent to the vehicle
   */
  public OrderRequest getLastOrderSent() {
    return lastOrderSent;
  }

  /**
   * Sets the last order telegram sent to the vehicle.
   *
   * @param lastOrderSent The last order telegram sent to the vehicle
   * @return This
   */
  public ExampleProcessModelTO setLastOrderSent(
      OrderRequest lastOrderSent) {
    this.lastOrderSent = lastOrderSent;
    return this;
  }

  /**
   * Returns the host to connect to.
   *
   * @return The host to connect to
   */
  public String getVehicleHost() {
    return vehicleHost;
  }

  /**
   * Sets the host to connect to.
   *
   * @param vehicleHost The host to connect to
   * @return This
   */
  public ExampleProcessModelTO setVehicleHost(String vehicleHost) {
    this.vehicleHost = vehicleHost;
    return this;
  }

  /**
   * Returns the port to connect to.
   *
   * @return The port to connect to
   */
  public int getVehiclePort() {
    return vehiclePort;
  }

  /**
   * Sets the port to connect to.
   *
   * @param vehiclePort The port to connect to
   * @return This
   */
  public ExampleProcessModelTO setVehiclePort(int vehiclePort) {
    this.vehiclePort = vehiclePort;
    return this;
  }

  /**
   * Returns whether periodic sending of {@link StateRequest} telegrams is enabled.
   *
   * @return Whether periodic sending of {@link StateRequest} telegrams is enabled
   */
  public boolean isPeriodicStateRequestEnabled() {
    return periodicStateRequestEnabled;
  }

  /**
   * Sets whether periodic sending of {@link StateRequest} telegrams is enabled.
   *
   * @param periodicStateRequestEnabled Whether periodic sending of {@link StateRequest}
   * telegrams is enabled
   * @return This
   */
  public ExampleProcessModelTO setPeriodicStateRequestEnabled(boolean periodicStateRequestEnabled) {
    this.periodicStateRequestEnabled = periodicStateRequestEnabled;
    return this;
  }

  /**
   * Returns the time to wait between periodic state request telegrams.
   *
   * @return The time to wait between periodic state request telegrams
   */
  public int getStateRequestInterval() {
    return stateRequestInterval;
  }

  /**
   * Sets the time to wait between periodic state request telegrams.
   *
   * @param stateRequestInterval The time to wait between periodic state request telegrams
   * @return This
   */
  public ExampleProcessModelTO setStateRequestInterval(int stateRequestInterval) {
    this.stateRequestInterval = stateRequestInterval;
    return this;
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
   * Sets how long (in ms) we tolerate not hearing from the vehicle before we consider communication
   * dead
   *
   * @param vehicleIdleTimeout How long (in ms) we tolerate not hearing from the vehicle before
   * we consider communication dead
   * @return This
   */
  public ExampleProcessModelTO setVehicleIdleTimeout(int vehicleIdleTimeout) {
    this.vehicleIdleTimeout = vehicleIdleTimeout;
    return this;
  }

  /**
   * Returns whether the vehicle has not been heard of recently.
   *
   * @return Whether the vehicle has not been heard of recently
   */
  public boolean isVehicleIdle() {
    return vehicleIdle;
  }

  /**
   * Sets whether the vehicle has not been heard of recently.
   *
   * @param vehicleIdle Whether the vehicle has not been heard of recently
   * @return This
   */
  public ExampleProcessModelTO setVehicleIdle(boolean vehicleIdle) {
    this.vehicleIdle = vehicleIdle;
    return this;
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
   * @return This
   */
  public ExampleProcessModelTO setDisconnectingOnVehicleIdle(boolean disconnectingOnVehicleIdle) {
    this.disconnectingOnVehicleIdle = disconnectingOnVehicleIdle;
    return this;
  }

  /**
   * Returns whether to reconnect automatically when the vehicle connection times out.
   *
   * @return Whether to reconnect automatically when the vehicle connection times out
   */
  public boolean isReconnectingOnConnectionLoss() {
    return reconnectingOnConnectionLoss;
  }

  /**
   * Sets whether to reconnect automatically when the vehicle connection times out.
   *
   * @param reconnectingOnConnectionLoss Whether to reconnect automatically when the vehicle
   * connection times out
   * @return This
   */
  public ExampleProcessModelTO setReconnectingOnConnectionLoss(boolean reconnectingOnConnectionLoss) {
    this.reconnectingOnConnectionLoss = reconnectingOnConnectionLoss;
    return this;
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
   * @return This
   */
  public ExampleProcessModelTO setReconnectDelay(int reconnectDelay) {
    this.reconnectDelay = reconnectDelay;
    return this;
  }

  /**
   * Returns whether logging should be enabled or not.
   *
   * @return Whether logging should be enabled or not
   */
  public boolean isLoggingEnabled() {
    return loggingEnabled;
  }

  /**
   * Sets whether logging should be enabled or not.
   *
   * @param loggingEnabled Whether logging should be enabled or not
   * @return This
   */
  public ExampleProcessModelTO setLoggingEnabled(boolean loggingEnabled) {
    this.loggingEnabled = loggingEnabled;
    return this;
  }
}
