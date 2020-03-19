/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.common.telegrams;

/**
 * Declares methods for comm adapters capable of sending telegrams/requests.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public interface TelegramSender {

  /**
   * Sends the given {@link Request}.
   *
   * @param request The {@link Request} to be sent.
   */
  void sendTelegram(Request request);
}
