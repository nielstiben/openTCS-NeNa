/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.telegrams;

import static com.google.common.base.Ascii.ETX;
import static com.google.common.base.Ascii.STX;
import static com.google.common.base.Preconditions.checkArgument;
import com.google.common.primitives.Ints;
import nl.saxion.nena.opentcs.common.telegrams.Response;
import static java.util.Objects.requireNonNull;

/**
 * Represents a vehicle status response sent from the vehicle.
 *
 * @author Mats Wilhelm (Fraunhofer IML)
 */
public class StateResponse
    extends Response {

  /**
   * The response type.
   */
  public static final byte TYPE = 1;
  /**
   * The expected length of a telegram of this type.
   */
  public static final int TELEGRAM_LENGTH = 17;
  /**
   * The size of the payload (the raw content, without STX, SIZE, CHECKSUM and ETX).
   */
  public static final int PAYLOAD_LENGTH = TELEGRAM_LENGTH - 4;
  /**
   * The position of the checksum byte.
   */
  public static final int CHECKSUM_POS = TELEGRAM_LENGTH - 2;
  /**
   * The id of the point at the vehicle's current position.
   */
  private int positionId;
  /**
   * The vehicle's operating state.
   */
  private OperatingState operatingState;
  /**
   * The vehicle's load state.
   */
  private LoadState loadState;
  /**
   * The id of the last received order.
   */
  private int lastReceivedOrderId;
  /**
   * The id of the current order.
   */
  private int currentOrderId;
  /**
   * The id of the last finished order.
   */
  private int lastFinishedOrderId;

  /**
   * Creates a new instance.
   *
   * @param telegramData This telegram's raw content.
   */
  public StateResponse(byte[] telegramData) {
    super(TELEGRAM_LENGTH);
    requireNonNull(telegramData, "telegramData");
    checkArgument(telegramData.length == TELEGRAM_LENGTH);

    System.arraycopy(telegramData, 0, rawContent, 0, TELEGRAM_LENGTH);
    decodeTelegramContent();
  }

  /**
   * Returns the id of the point at the vehicle's current position.
   *
   * @return The id of the point at the vehicle's current position
   */
  public int getPositionId() {
    return positionId;
  }

  /**
   * Returns the vehicle's operating state.
   *
   * @return The vehicle's operating state.
   */
  public OperatingState getOperatingState() {
    return operatingState;
  }

  /**
   * Returns the vehicle's load state.
   *
   * @return The vehicle's load state.
   */
  public LoadState getLoadState() {
    return loadState;
  }

  /**
   * Returns the id of the last received order.
   *
   * @return The id of the last received order.
   */
  public int getLastReceivedOrderId() {
    return lastReceivedOrderId;
  }

  /**
   * Returns the id of the current order.
   *
   * @return The id of the current order.
   */
  public int getCurrentOrderId() {
    return currentOrderId;
  }

  /**
   * Returns the id of the last finished order.
   *
   * @return The id of the last finished order.
   */
  public int getLastFinishedOrderId() {
    return lastFinishedOrderId;
  }

  /**
   * Returns the telegram's checksum byte.
   *
   * @return The telegram's checksum byte.
   */
  public byte getCheckSum() {
    return rawContent[CHECKSUM_POS];
  }
  
  @Override
  public String toString() {
    return "StateResponse{" + "id=" + id + '}';
  }

  /**
   * Checks if the given byte array is a state reponse telegram.
   *
   * @param telegramData The telegram data to check.
   * @return {@code true} if, and only if, the given data is a state response telegram.
   */
  public static boolean isStateResponse(byte[] telegramData) {
    requireNonNull(telegramData, "data");

    boolean result = true;
    if (telegramData.length != TELEGRAM_LENGTH) {
      result = false;
    }
    else if (telegramData[0] != STX) {
      result = false;
    }
    else if (telegramData[TELEGRAM_LENGTH - 1] != ETX) {
      result = false;
    }
    else if (telegramData[1] != PAYLOAD_LENGTH) {
      result = false;
    }
    else if (telegramData[2] != TYPE) {
      result = false;
    }
    else if (getCheckSum(telegramData) != telegramData[CHECKSUM_POS]) {
      result = false;
    }
    return result;
  }

  private void decodeTelegramContent() {
    id = Ints.fromBytes((byte) 0, (byte) 0, rawContent[3], rawContent[4]);
    positionId = Ints.fromBytes((byte) 0, (byte) 0, rawContent[5], rawContent[6]);
    operatingState = decodeOperatingState((char) rawContent[7]);
    loadState = decodeLoadState((char) rawContent[8]);
    lastReceivedOrderId = Ints.fromBytes((byte) 0, (byte) 0, rawContent[9], rawContent[10]);
    currentOrderId = Ints.fromBytes((byte) 0, (byte) 0, rawContent[11], rawContent[12]);
    lastFinishedOrderId = Ints.fromBytes((byte) 0, (byte) 0, rawContent[13], rawContent[14]);
  }

  private OperatingState decodeOperatingState(char operatingStateRaw) {
    switch (operatingStateRaw) {
      case 'A':
        return OperatingState.ACTING;
      case 'I':
        return OperatingState.IDLE;
      case 'M':
        return OperatingState.MOVING;
      case 'E':
        return OperatingState.ERROR;
      case 'C':
        return OperatingState.CHARGING;
      default:
        return OperatingState.UNKNOWN;
    }
  }

  private LoadState decodeLoadState(char loadStateRaw) {
    switch (loadStateRaw) {
      case 'E':
        return LoadState.EMPTY;
      case 'F':
        return LoadState.FULL;
      default:
        return LoadState.UNKNOWN;
    }
  }

  /**
   * The load handling state of a vehicle.
   */
  public static enum LoadState {
    /**
     * The vehicle's load handling state is currently empty.
     */
    EMPTY,
    /**
     * The vehicle's load handling state is currently full.
     */
    FULL,
    /**
     * The vehicle's load handling state is currently unknown.
     */
    UNKNOWN
  }

  /**
   * The operating state of a vehicle.
   */
  public static enum OperatingState {
    /**
     * The vehicle is currently executing an operation.
     */
    ACTING,
    /**
     * The vehicle is currently idle.
     */
    IDLE,
    /**
     * The vehicle is currently moving.
     */
    MOVING,
    /**
     * The vehicle is currently in an error state.
     */
    ERROR,
    /**
     * The vehicle is currently recharging.
     */
    CHARGING,
    /**
     * The vehicle's state is currently unknown.
     */
    UNKNOWN
  }
}
