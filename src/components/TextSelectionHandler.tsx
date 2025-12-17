import React, { useEffect } from 'react';
import { useChatContext } from '../contexts/ChatContext';
import { validateSelectedText } from './ChatAPIUtils';

const TextSelectionHandler: React.FC = () => {
  const { setSelectedText, setVisible, setError } = useChatContext();

  useEffect(() => {
    const handleTextSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();

      if (selectedText) {
        // Validate the selected text length
        const validation = validateSelectedText(selectedText);

        if (!validation.isValid) {
          setError(validation.error);
          return;
        }

        // Get the current page information
        const selectedTextContext = {
          text: selectedText,
          pageUrl: window.location.href,
          pageTitle: document.title,
          timestamp: new Date(),
          position: {
            start: 0, // In a real implementation, we would track the actual position
            end: selectedText.length
          }
        };

        // Store the selected text in the context
        setSelectedText(selectedTextContext);
      } else {
        // If no text is selected, clear the selected text context
        setSelectedText(null);
      }
    };

    // Add event listeners for text selection
    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('touchend', handleTextSelection);

    // Cleanup event listeners on component unmount
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('touchend', handleTextSelection);
    };
  }, [setSelectedText, setError]);

  // Render a floating button when text is selected
  return null; // This component doesn't render anything itself, but we'll add the button to the main ChatComponent
};

export default TextSelectionHandler;