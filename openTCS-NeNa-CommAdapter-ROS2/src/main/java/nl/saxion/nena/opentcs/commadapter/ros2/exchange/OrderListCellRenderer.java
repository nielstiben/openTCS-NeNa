/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2.exchange;

import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderAction;
import nl.saxion.nena.opentcs.commadapter.ros2.telegrams.OrderRequest;
import java.awt.Component;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.ListCellRenderer;

/**
 * Renders order telegrams when displayed in a list.
 *
 * @author Martin Grzenia (Fraunhofer IML)
 */
public class OrderListCellRenderer
    extends JLabel
    implements ListCellRenderer<OrderRequest> {

  /**
   * A prototype for the list to compute its preferred size.
   */
  public static final OrderRequest PROTOTYPE_TELEGRAM
      = new OrderRequest(0, 0, 0, OrderAction.NONE);

  @Override
  public Component getListCellRendererComponent(JList<? extends OrderRequest> list,
                                                OrderRequest value,
                                                int index,
                                                boolean isSelected,
                                                boolean cellHasFocus) {
    StringBuilder sb = new StringBuilder();
    sb.append('#');
    sb.append(value.getId());
    sb.append(": ");
    sb.append(value.getDestinationId());
    sb.append(' ');
    sb.append(value.getDestinationAction());
    sb.append("...");
    setText(sb.toString());
    return this;
  }

}
