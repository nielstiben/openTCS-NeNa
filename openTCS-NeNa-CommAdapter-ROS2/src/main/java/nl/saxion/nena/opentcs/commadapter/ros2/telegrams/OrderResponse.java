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
 * Represents an order response sent from the vehicle.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class OrderResponse
    extends Response {

  /**
   * The response type.
   */
  public static final byte TYPE = 2;
  /**
   * The expected length of a telegram of this type.
   */
  public static final int TELEGRAM_LENGTH = 9;
  /**
   * The size of the payload (the raw content, without STX, SIZE, CHECKSUM and ETX).
   */
  public static final int PAYLOAD_LENGTH = TELEGRAM_LENGTH - 4;
  /**
   * The position of the checksum byte.
   */
  public static final int CHECKSUM_POS = TELEGRAM_LENGTH - 2;
  /**
   * The order id received by the vehicle.
   */
  private int orderId;

  /**
   * Creates a new instance.
   *
   * @param telegramData This telegram's raw content.
   */
  public OrderResponse(byte[] telegramData) {
    super(TELEGRAM_LENGTH);
    requireNonNull(telegramData, "telegramData");
    checkArgument(telegramData.length == TELEGRAM_LENGTH);

    System.arraycopy(telegramData, 0, rawContent, 0, TELEGRAM_LENGTH);
    decodeTelegramContent();
  }

  /**
   * Returns the order id received by the vehicle.
   *
   * @return The order id received by the vehicle.
   */
  public int getOrderId() {
    return orderId;
  }

  @Override
  public String toString() {
    return "OrderResponse{" + "id=" + id + '}';
  }

  /**
   * Checks if the given byte array is an order reponse telegram.
   *
   * @param telegramData The telegram data to check.
   * @return {@code true} if, and only if, the given data is an order response telegram.
   */
  public static boolean isOrderResponse(byte[] telegramData) {
    requireNonNull(telegramData, "telegramData");

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
    this.id = Ints.fromBytes((byte) 0, (byte) 0, rawContent[3], rawContent[4]);
    orderId = Ints.fromBytes((byte) 0, (byte) 0, rawContent[5], rawContent[6]);
  }
}
