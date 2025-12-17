import React, { useState, useEffect, useRef } from 'react';
import { useChatContext } from '../contexts/ChatContext';
import { sendMessageToRAG, handleAPIError } from './ChatAPIUtils';
import '../components/chat-styles.css';

const ChatComponent: React.FC = () => {
  const {
    session,
    isVisible,
    isMinimized,
    selectedText,
    isLoading,
    error,
    setVisible,
    setMinimized,
    setLoading,
    setError,
    clearError,
    addMessage,
    resetSession
  } = useChatContext();

  const [isComponentVisible, setIsComponentVisible] = useState(isVisible);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [session?.messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Handle sending a message
  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!session || isLoading) return;

    if (!inputValue.trim()) return;

    try {
      setLoading(true);
      clearError();

      // Add user message to UI immediately
      const userMessage = {
        id: `msg-${Date.now()}-${Math.random()}`,
        content: inputValue,
        role: 'user' as const,
        timestamp: new Date(),
      };

      addMessage(userMessage);

      // Prepare context - use selected text if available, otherwise use session context
      let context = session.context;
      if (selectedText && selectedText.text) {
        context = `${selectedText.text}\n\nUser question: ${inputValue}`;
      } else {
        // For follow-up questions, we can include the conversation history as context
        // Limit to last few messages to avoid exceeding API limits
        const recentMessages = session.messages.slice(-3); // Last 3 messages
        if (recentMessages.length > 0) {
          const recentContext = recentMessages.map(msg =>
            `${msg.role === 'user' ? 'User' : 'Assistant'}: ${msg.content}`
          ).join('\n');
          context = `${recentContext}\n\nCurrent question: ${inputValue}`;
        } else {
          context = inputValue;
        }
      }

      // Send message to RAG backend
      const response = await sendMessageToRAG(inputValue, context, session.id);

      // Add assistant response to the chat
      const assistantMessage = {
        id: response.id,
        content: response.content,
        role: 'assistant' as const,
        timestamp: new Date(response.createdAt),
        sources: response.sources,
      };

      addMessage(assistantMessage);

      // Clear input
      setInputValue('');
    } catch (err) {
      const errorMessage = handleAPIError(err);
      setError(errorMessage);
      console.error('Error sending message:', err);
    } finally {
      setLoading(false);
    }
  };

  // Local state for input since we're not directly using useChat in this implementation
  const [inputValue, setInputValue] = useState('');

  // Handle input change
  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setInputValue(e.target.value);
  };

  // Toggle visibility
  const toggleVisibility = () => {
    const newVisible = !isComponentVisible;
    setIsComponentVisible(newVisible);
    setVisible(newVisible);
  };

  // Toggle minimized state
  const toggleMinimized = () => {
    setMinimized(!isMinimized);
  };

  // If not visible, don't render anything
  if (!isComponentVisible) {
    return (
      <button
        className="chat-header-btn"
        onClick={toggleVisibility}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          zIndex: 10000,
          background: '#4f46e5',
          color: 'white',
          borderRadius: '50%',
          width: '50px',
          height: '50px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          cursor: 'pointer',
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)'
        }}
      >
        💬
      </button>
    );
  }

  // If minimized, show minimized view
  if (isMinimized) {
    return (
      <div className="chat-container minimized">
        <div
          className="chat-header"
          onClick={toggleMinimized}
          onKeyDown={(e) => {
            if (e.key === 'Enter' || e.key === ' ') {
              e.preventDefault();
              toggleMinimized();
            }
          }}
          tabIndex={0}
          role="button"
          aria-label="Expand chat panel"
        >
          <h3>AI Assistant</h3>
          <div className="chat-header-controls">
            <button
              className="chat-header-btn"
              onClick={toggleMinimized}
              onKeyDown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  toggleMinimized();
                }
              }}
              tabIndex={0}
              aria-label="Expand chat panel"
            >
              +
            </button>
            <button
              className="chat-header-btn"
              onClick={toggleVisibility}
              onKeyDown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  toggleVisibility();
                }
              }}
              tabIndex={0}
              aria-label="Hide chat panel"
            >
              ✕
            </button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <>
      {/* Floating button for selected text */}
      {selectedText && !isComponentVisible && (
        <button
          className="floating-context-btn"
          onClick={() => {
            setVisible(true);
            // Pre-fill the input with the selected text
            setInputValue(selectedText.text);
          }}
        >
          💬 Ask about selection
        </button>
      )}

      {/* Main chat container */}
      <div className="chat-container">
        <div
          className="chat-header"
          onClick={toggleMinimized}
          onKeyDown={(e) => {
            if (e.key === 'Enter' || e.key === ' ') {
              e.preventDefault();
              toggleMinimized();
            }
          }}
          tabIndex={0}
          role="button"
          aria-label="Minimize chat panel"
        >
          <h3>AI Assistant</h3>
          <div className="chat-header-controls">
            <button
              className="chat-header-btn"
              onClick={(e) => {
                e.stopPropagation();
                if (window.confirm('Are you sure you want to clear all chat history?')) {
                  resetSession();
                }
              }}
              onKeyDown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  e.stopPropagation();
                  if (window.confirm('Are you sure you want to clear all chat history?')) {
                    resetSession();
                  }
                }
              }}
              tabIndex={0}
              aria-label="Clear chat history"
              title="Clear chat"
            >
              🗑️
            </button>
            <button
              className="chat-header-btn"
              onClick={toggleMinimized}
              onKeyDown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  toggleMinimized();
                }
              }}
              tabIndex={0}
              aria-label="Minimize chat panel"
            >
              −
            </button>
            <button
              className="chat-header-btn"
              onClick={toggleVisibility}
              onKeyDown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  toggleVisibility();
                }
              }}
              tabIndex={0}
              aria-label="Hide chat panel"
            >
              ✕
            </button>
          </div>
        </div>

        <div className="chat-messages">
          {session?.messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.role === 'user' ? 'user' : 'assistant'}`}
            >
              {message.content}
              {message.sources && message.sources.length > 0 && (
                <div className="message-sources">
                  Sources:
                  {message.sources.map((source, idx) => (
                    <div key={idx}>
                      <a href={source.url} target="_blank" rel="noopener noreferrer">
                        {source.title}
                      </a>
                    </div>
                  ))}
                </div>
              )}
            </div>
          ))}
          {isLoading && (
            <div className="message assistant">
              <div className="chat-loading">
                <div className="loading-dots">
                  <div className="loading-dot"></div>
                  <div className="loading-dot"></div>
                  <div className="loading-dot"></div>
                </div>
              </div>
            </div>
          )}
          {error && (
            <div className="chat-error">
              {error}
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <form className="chat-input-area" onSubmit={handleSendMessage}>
          <textarea
            className="chat-input"
            value={inputValue}
            onChange={handleInputChange}
            placeholder="Ask about this documentation..."
            rows={1}
            onKeyDown={(e) => {
              if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                handleSendMessage(e);
              }
            }}
          />
          <button
            type="submit"
            className="chat-submit-btn"
            disabled={isLoading || !inputValue.trim()}
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
            </svg>
          </button>
        </form>
      </div>
    </>
  );
};

export default ChatComponent;