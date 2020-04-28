///**
// * Copyright (c) Fraunhofer IML
// */
///*
// * To change this license header, choose License Headers in Project Properties.
// * To change this template file, choose Tools | Templates
// * and open the template in the editor.
// */
//package nl.saxion.nena.opentcs.commadapter.ros2.comm;
//
//import static com.google.common.base.Ascii.ETX;
//import static com.google.common.base.Ascii.STX;
//import com.google.common.primitives.Ints;
//import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderResponse;
//import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateResponse;
//import nl.saxion.nena.opentcs.common.telegrams.Response;
//import nl.saxion.nena.opentcs.common.telegrams.Telegram;
//import io.netty.buffer.ByteBuf;
//import io.netty.buffer.Unpooled;
//import io.netty.channel.ChannelHandlerContext;
//import java.util.LinkedList;
//import org.junit.*;
//import static org.mockito.Matchers.any;
//import static org.mockito.Mockito.mock;
//import static org.mockito.Mockito.never;
//import static org.mockito.Mockito.times;
//import static org.mockito.Mockito.verify;
//import org.opentcs.contrib.tcp.netty.ConnectionEventListener;
//
///**
// * Test cases for the {@link VehicleTelegramDecoder}.
// *
// * @author Mats Wilhelm (Fraunhofer IML)
// */
//public class VehicleTelegramDecoderTest {
//
//  private VehicleTelegramDecoder decoder;
//
//  private ConnectionEventListener<Response> responseListener;
//
//  @Before
//  @SuppressWarnings("unchecked")
//  public void setUp() {
//    responseListener = mock(ConnectionEventListener.class);
//    decoder = new VehicleTelegramDecoder(responseListener);
//  }
//
//  @Test
//  public void shouldNotReadTelegramWhenStreamIsEmpty() {
//    decoder.decode(mock(ChannelHandlerContext.class), Unpooled.buffer(), new LinkedList<>());
//    verify(responseListener, never()).onIncomingTelegram(any());
//  }
//
//  @Test
//  public void shouldNotReadTelegramWithNotEnoughBytes() {
//    ByteBuf buffer = Unpooled.buffer();
//    buffer.writeByte(0);
//    decoder.decode(mock(ChannelHandlerContext.class), buffer, new LinkedList<>());
//    verify(responseListener, never()).onIncomingTelegram(any());
//  }
//
//  @Test
//  public void shouldReadOrderResponse() {
//    ByteBuf buffer = Unpooled.buffer();
//    buffer.writeBytes(createOrderResponse(0, 0).getRawContent());
//    decoder.decode(mock(ChannelHandlerContext.class), buffer, new LinkedList<>());
//    verify(responseListener, times(1)).onIncomingTelegram(any());
//  }
//
//  @Test
//  public void shouldReadStatusResponse() {
//    ByteBuf buffer = Unpooled.buffer();
//    buffer.writeBytes(createStateResponse(0, 0, 'M', 'E', 0, 0, 0).getRawContent());
//    decoder.decode(mock(ChannelHandlerContext.class), buffer, new LinkedList<>());
//    verify(responseListener, times(1)).onIncomingTelegram(any());
//  }
//
//  @Test
//  public void shouldIgnoreWrongTelegram() {
//    ByteBuf buffer = Unpooled.buffer();
//    buffer.writeZero(StateResponse.TELEGRAM_LENGTH + 1);
//    decoder.decode(mock(ChannelHandlerContext.class), buffer, new LinkedList<>());
//    verify(responseListener, times(0)).onIncomingTelegram(any());
//  }
//
//  /**
//   * Creates an order response with it's byte contents.
//   *
//   * @param telegramCounter The telegram counter in the order response
//   * @param lastOrderId The last received order id in the order response
//   * @return The order response
//   */
//  private OrderResponse createOrderResponse(int telegramCounter, int lastOrderId) {
//    byte[] telegramData = new byte[OrderResponse.TELEGRAM_LENGTH];
//
//    telegramData[0] = STX;
//    telegramData[1] = OrderResponse.PAYLOAD_LENGTH;
//    telegramData[2] = OrderResponse.TYPE;
//    // set telegram counter
//    byte[] tmp = Ints.toByteArray(telegramCounter);
//    telegramData[3] = tmp[2];
//    telegramData[4] = tmp[3];
//    // set last received order id
//    tmp = Ints.toByteArray(lastOrderId);
//    telegramData[5] = tmp[2];
//    telegramData[6] = tmp[3];
//    // set checksum
//    telegramData[OrderResponse.CHECKSUM_POS] = Telegram.getCheckSum(telegramData);
//    telegramData[OrderResponse.TELEGRAM_LENGTH - 1] = ETX;
//
//    return new OrderResponse(telegramData);
//  }
//
//  /**
//   * Creates a state response with the given content.
//   *
//   * @param telegramCounter The telegram counter
//   * @param positionId The current position
//   * @param operationMode The current operation mode
//   * @param loadState The current load handling state
//   * @param lastReceivedOrder The last received order id
//   * @param currentOrder The current order id
//   * @param lastFinishedOrder The last finished order id
//   * @return The state response
//   */
//  private StateResponse createStateResponse(int telegramCounter,
//                                            int positionId,
//                                            char operationMode,
//                                            char loadState,
//                                            int lastReceivedOrder,
//                                            int currentOrder,
//                                            int lastFinishedOrder) {
//    byte[] telegramData = new byte[StateResponse.TELEGRAM_LENGTH];
//
//    telegramData[0] = STX;
//    telegramData[1] = StateResponse.PAYLOAD_LENGTH;
//    telegramData[2] = StateResponse.TYPE;
//    // set telegram counter
//    byte[] tmp = Ints.toByteArray(telegramCounter);
//    telegramData[3] = tmp[2];
//    telegramData[4] = tmp[3];
//    // set pos id
//    tmp = Ints.toByteArray(positionId);
//    telegramData[5] = tmp[2];
//    telegramData[6] = tmp[3];
//    // set op mode
//    telegramData[7] = (byte) operationMode;
//    // set load state
//    telegramData[8] = (byte) loadState;
//    // set last received order id
//    tmp = Ints.toByteArray(lastReceivedOrder);
//    telegramData[9] = tmp[2];
//    telegramData[10] = tmp[3];
//    // set current order id
//    tmp = Ints.toByteArray(currentOrder);
//    telegramData[11] = tmp[2];
//    telegramData[12] = tmp[3];
//    // set last finished order id
//    tmp = Ints.toByteArray(lastFinishedOrder);
//    telegramData[13] = tmp[2];
//    telegramData[14] = tmp[3];
//    // set checksum
//    telegramData[StateResponse.CHECKSUM_POS] = Telegram.getCheckSum(telegramData);
//    telegramData[StateResponse.TELEGRAM_LENGTH - 1] = ETX;
//
//    return new StateResponse(telegramData);
//  }
//}
