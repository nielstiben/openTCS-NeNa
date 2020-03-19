/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.common.telegrams;

import static com.google.common.base.Ascii.ETX;
import static com.google.common.base.Ascii.STX;
import com.google.common.primitives.Ints;
import org.junit.*;
import static org.mockito.Matchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

/**
 * Test cases for the {@link RequestResponseMatcher}.
 *
 * @author Mats Wilhelm (Fraunhofer IML)
 */
public class RequestResponseMatcherTest {

  private RequestResponseMatcher matcher;

  private TelegramSender sender;

  @Before
  public void setUp() {
    sender = mock(TelegramSender.class);
    matcher = new RequestResponseMatcher(sender);
  }

  @Test
  public void shouldNotProcessResponseWithNoRequest() {
    matcher.tryMatchWithCurrentRequest(createResponse(1));
    verify(sender, times(0)).sendTelegram(any());
  }

  @Test
  public void shouldSendRequestIfQueueIsEmpty() {
    matcher.enqueueRequest(createRequest(1));
    verify(sender, times(1)).sendTelegram(any());
  }

  @Test
  public void shouldNotSendRequestIfQueueIsNotEmpty() {
    matcher.enqueueRequest(createRequest(1));
    verify(sender, times(1)).sendTelegram(any());
    matcher.enqueueRequest(createRequest(2));
    verify(sender, times(1)).sendTelegram(any());
  }

  @Test
  public void shouldProcessResponseForWaitingRequest() {
    matcher.enqueueRequest(createRequest(1));
    verify(sender, times(1)).sendTelegram(any());
    matcher.tryMatchWithCurrentRequest(createResponse(1));
    Assert.assertFalse("A request is waiting in the matcher but should not.",
                       matcher.peekCurrentRequest().isPresent());
  }

  @Test
  public void shouldSendNextRequestAfterProcessingResponse() {
    matcher.enqueueRequest(createRequest(1));
    verify(sender, times(1)).sendTelegram(any());
    matcher.enqueueRequest(createRequest(2));
    verify(sender, times(1)).sendTelegram(any());
    matcher.tryMatchWithCurrentRequest(createResponse(1));
    matcher.checkForSendingNextRequest();
    verify(sender, times(2)).sendTelegram(any());
  }

  /**
   * Creates an response with it's byte contents.
   *
   * @param telegramCounter The telegram counter in the response
   * @return The response
   */
  private Request createRequest(int telegramCounter) {
    Request request = new Request(7) {
      @Override
      public void updateRequestContent(int telegramId) {
        this.id = telegramId;
        rawContent[0] = STX;
        rawContent[1] = 2;
        rawContent[2] = 0;
        // set telegram counter
        byte[] tmp = Ints.toByteArray(telegramCounter);
        rawContent[3] = tmp[2];
        rawContent[4] = tmp[3];
        // set checksum
        rawContent[5] = Telegram.getCheckSum(rawContent);
        rawContent[6] = ETX;
      }
    };
    request.updateRequestContent(telegramCounter);
    return request;
  }

  /**
   * Creates a response with the given content.
   *
   * @param telegramCounter The telegram counter
   * @return The response
   */
  private Response createResponse(int telegramCounter) {
    int telegramSize = 7;
    byte[] telegramData = new byte[telegramSize];

    telegramData[0] = STX;
    telegramData[1] = 2;
    telegramData[2] = 0;
    // set telegram counter
    byte[] tmp = Ints.toByteArray(telegramCounter);
    telegramData[3] = tmp[2];
    telegramData[4] = tmp[3];
    // set checksum
    telegramData[5] = Telegram.getCheckSum(telegramData);
    telegramData[6] = ETX;

    Response response = new Response(telegramSize) {
    };
    response.id = telegramCounter;
    System.arraycopy(telegramData, 0, response.rawContent, 0, telegramSize);
    return response;
  }
}
