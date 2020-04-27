package nl.saxion.nena.opentcs.commadapter.ros2.control_center.gui_components;

import lombok.AllArgsConstructor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.text.BadLocationException;
import javax.swing.text.Document;

/**
 * Abstract base class for <code>InputPanels</code> that use text fields for input.
 * The main purpose of this class is to provide an easy to use way to validate
 * text inputs using it's nested class {@link TextInputPanel.TextInputValidator
 * TextInputValidator}.
 *
 * @author Niels Tiben
 */
public abstract class TextInputPanel extends InputPanel {

  /**
   * This class's logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(TextInputPanel.class);


  public TextInputPanel(String title) {
    super(title);
  }

  protected void setInputValid(boolean valid, Document doc) {
    setInputValid(valid);
  }

  @AllArgsConstructor
  public class TextInputValidator implements DocumentListener {
    private final String validationRegex;

    @Override
    public void insertUpdate(DocumentEvent e) {
      validate(e.getDocument());
    }

    @Override
    public void removeUpdate(DocumentEvent e) {
      validate(e.getDocument());
    }

    @Override
    public void changedUpdate(DocumentEvent e) {
      // Do nothing 
    }

    /**
     * Validate the specified <code>Document</code> and set the validation
     * state in the {@link InputPanel} accordingly.
     * @param doc The <code>Document</code> to validate.
     */
    private void validate(Document doc) {
      String text;
      try {
        text = doc.getText(0, doc.getLength());
      }
      catch (BadLocationException e) {
        LOG.warn("Exception retrieving document text", e);
        return;
      }
      setInputValid(text.matches(validationRegex), doc);
    }
  }
}
