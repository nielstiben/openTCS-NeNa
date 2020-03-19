/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.comm;

import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderRequest;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.StateRequest;
import nl.saxion.nena.opentcs.common.telegrams.Request;
import io.netty.buffer.ByteBuf;
import io.netty.channel.ChannelHandlerContext;
import io.netty.handler.codec.MessageToByteEncoder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Encodes outgoing {@link StateRequest} instances.
 *
 * @author Stefan Walter (Fraunhofer IML)
 */
public class VehicleTelegramEncoder
    extends MessageToByteEncoder<Request> {

  /**
   * This class's Logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(VehicleTelegramEncoder.class);

  @Override
  protected void encode(ChannelHandlerContext ctx, Request msg, ByteBuf out)
      throws Exception {
    LOG.debug("Encoding request of class {}", msg.getClass().getName());

    if (msg instanceof OrderRequest) {
      OrderRequest order = (OrderRequest) msg;
      LOG.debug("Encoding order telegram: {}", order.toString());
    }
    
    out.writeBytes(msg.getRawContent());
  }
}
