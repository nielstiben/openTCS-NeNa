/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.common.telegrams;

import com.google.inject.assistedinject.Assisted;
import java.util.LinkedList;
import static java.util.Objects.requireNonNull;
import java.util.Optional;
import java.util.Queue;
import javax.annotation.Nonnull;
import javax.inject.Inject;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Keeps {@link Request}s in a queue and matches them with incoming {@link Response}s.
 *
 * @author Stefan Walter (Fraunhofer IML)
 */
public class RequestResponseMatcher {

  /**
   * This class's logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(RequestResponseMatcher.class);
  /**
   * The actual queue of requests.
   */
  private final Queue<Request> requests = new LinkedList<>();
  /**
   * Sends the queued {@link Request}s.
   */
  private final TelegramSender telegramSender;

  /**
   * Creates a new instance.
   *
   * @param telegramSender Sends the queued {@link Request}s.
   */
  @Inject
  public RequestResponseMatcher(@Assisted TelegramSender telegramSender) {
    this.telegramSender = requireNonNull(telegramSender, "telegramSender");
  }

  public void enqueueRequest(@Nonnull Request request) {
    requireNonNull(request, "request");
    boolean emptyQueueBeforeEnqueue = requests.isEmpty();

    LOG.debug("Enqueuing request: {}", request);
    requests.add(request);

    if (emptyQueueBeforeEnqueue) {
      checkForSendingNextRequest();
    }
  }

  /**
   * Checks if a telegram is enqueued and sends it.
   */
  public void checkForSendingNextRequest() {
    LOG.debug("Check for sending next request.");
    if (peekCurrentRequest().isPresent()) {
      telegramSender.sendTelegram(peekCurrentRequest().get());
    }
    else {
      LOG.debug("No requests to be sent.");
    }
  }

  /**
   * Returns the next request in the queue or an {@link Optional#EMPTY} if none is present.
   *
   * @return The next request in the queue or an {@link Optional#EMPTY} if none is present
   */
  public Optional<Request> peekCurrentRequest() {
    return Optional.ofNullable(requests.peek());
  }

  /**
   * Returns <code>true</code> if the response matches to the first request in the queue.
   * If it matches, the request will be removed.
   *
   * @param response The response to match
   * @return <code>true</code> if the response matches to the first request in the queue.
   */
  public boolean tryMatchWithCurrentRequest(@Nonnull Response response) {
    requireNonNull(response, "response");

    Request currentRequest = requests.peek();
    if (currentRequest != null && response.isResponseTo(currentRequest)) {
      requests.remove();
      return true;
    }

    if (currentRequest != null) {
      LOG.info("No request matching response with counter {}. Latest request counter is {}.",
               response.getId(), currentRequest.getId());
    }
    else {
      LOG.info("Received response with counter {}, but no request is waiting for a response.",
               response.getId());
    }

    return false;
  }

  /**
   * Clears all requests stored in the queue.
   */
  public void clear() {
    requests.clear();
  }
}
