import React, { useState, useEffect } from 'react';
import styles from './ChatWidget.module.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isLoaded, setIsLoaded] = useState(false);

  // Determine the chatbot URL based on environment
  const getChatbotUrl = () => {
    if (typeof window !== 'undefined') {
      // Client-side check
      if (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') {
        return 'http://localhost:3001'; // Using 3001 since 3000 is Docusaurus
      }
    }
    // For production/GitHub Pages, use the deployed URL
    return process.env.REACT_APP_CHATBOT_URL || 'https://tayyabahussain98.github.io/hackathon_1-physical_ai_humanoid_robotics_textbook/';
  };

  const chatbotUrl = getChatbotUrl();

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleLoad = () => {
    setIsLoaded(true);
  };

  return (
    <div className={styles.chatWidget}>
      {isOpen ? (
        <div className={styles.chatContainer}>
          <div className={styles.chatHeader}>
            <span>AI Assistant</span>
            <button
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>
          <div className={styles.chatFrameContainer}>
            {!isLoaded && (
              <div className={styles.loadingSpinner}>
                <div className={styles.spinner}></div>
                Loading...
              </div>
            )}
            <iframe
              src={chatbotUrl}
              title="AI Chatbot"
              className={`${styles.chatFrame} ${isLoaded ? styles.loaded : ''}`}
              onLoad={handleLoad}
              style={{ display: isLoaded ? 'block' : 'none' }}
              allow="microphone; camera"
            />
          </div>
        </div>
      ) : null}

      <button
        className={styles.chatButton}
        onClick={toggleChat}
        aria-label="Open chat"
      >
        💬
      </button>
    </div>
  );
};

export default ChatWidget;