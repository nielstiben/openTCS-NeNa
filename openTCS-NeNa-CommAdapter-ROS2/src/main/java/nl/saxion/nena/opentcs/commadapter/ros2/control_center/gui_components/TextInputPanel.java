/**
 * Copyright (c) The openTCS Authors.
 *
 * This program is free software and subject to the MIT license. (For details,
 * see the licensing information (LICENSE.txt) you should have received with
 * this copy of the software.)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.control_center.gui_components;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.text.BadLocationException;
import javax.swing.text.Document;
import java.util.Objects;

/**
 * Abstract base class for <code>InputPanels</code> that use text fields for input.
 * The main purpose of this class is to provide an easy to use way to validate
 * text inputs using it's nested class {@link TextInputPanel.TextInputValidator
 * TextInputValidator}.
 *
 * @author Tobias Marquardt (Fraunhofer IML)
 */
public abstract class TextInputPanel
    extends InputPanel {

  /**
   * This class's logger.
   */
  private static final Logger LOG = LoggerFactory.getLogger(TextInputPanel.class);
  /**
   * Create a new instance of <code>TextInputPanel</code>.
   * @param title The title of this panel.
   */
  public TextInputPanel(String title) {
    super(title);
  }

  protected void setInputValid(boolean valid, Document doc) {
    setInputValid(valid);
  }

  public class TextInputValidator
      implements DocumentListener {


    public static final String REGEX_FLOAT = "[-+]?[0-9]+(\\.[0-9]+)?";
    /**
     * Regular expression that accepts a positive floating point number of 
     * arbitrary length and 0. 
     */
    public static final String REGEX_FLOAT_POS = "\\+?[0-9]+(\\.[0-9]+)?";
    /**
     * Regular expression that accepts a negative floating point number of 
     * arbitrary length and 0. 
     */
    public static final String REGEX_FLOAT_NEG = "-[0-9]+(\\.[0-9]+)?|0+(\\.0+)?";
    /** 
     * Regular expression that accepts any integer of 
     * arbitrary length.
     */
    public static final String REGEX_INT = "[-+]?[0-9]+";
    /** 
     * Regular expression that accepts any positive integer of arbitrary length
     * and 0.
     */
    public static final String REGEX_INT_POS = "\\+?[0-9]+";
    /** 
     * Regular expression that accepts any negative integer of arbitrary length
     * and 0.
     */
    public static final String REGEX_INT_NEG = "-[0-9]+|0+";
    /**
     * Regular expression that accepts an integer in the interval [0,100].
     */
    public static final String REGEX_INT_RANGE_0_100 = "[0-9]|[1-9][0-9]|100";
    /**
     * Regular expression that accepts anything except an empty (or 
     * whitespace-only) string.
     */
    public static final String REGEX_NOT_EMPTY = ".*\\S.*";
    /**
     * Regular expression to validate the documents text against.
     */
    private final String format;

    /**
     * Create an instance of <code>TextInputValidator</code>.
     * @param format The regular expression to use for validation.
     */
    protected TextInputValidator(String format) {
      this.format = Objects.requireNonNull(format);
    }

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
      setInputValid(text.matches(format), doc);
    }
  }
}
