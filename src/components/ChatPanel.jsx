import React, { useState, useEffect, useRef } from 'react';

const ChatPanel = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, content: "Hello! I'm your AI assistant. How can I help you today?", role: 'assistant', timestamp: new Date() }
  ]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [userId] = useState(`user_${Date.now()}`);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const getBackendUrl = () => {
    if (typeof window !== 'undefined') {
      if (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') {
        return 'http://localhost:8000';
      }
    }
    return process.env.REACT_APP_BACKEND_URL || 'https://your-fastapi-backend.up.railway.app';
  };

  const sendMessage = async () => {
    if (!inputMessage.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      content: inputMessage,
      role: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    const newInput = inputMessage;
    setInputMessage('');
    setIsLoading(true);

    try {
      const response = await fetch(`${getBackendUrl()}/chat/default_thread`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: newInput,
          user_id: userId
        })
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage = {
          id: data.id,
          content: data.content,
          role: 'assistant',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage = {
          id: Date.now(),
          content: "Sorry, I encountered an error processing your request.",
          role: 'assistant',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage = {
        id: Date.now(),
        content: "Sorry, I'm having trouble connecting to the server.",
        role: 'assistant',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  const styles = {
    chatWidget: {
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      zIndex: 1000,
      fontFamily: 'system-ui, -apple-system, sans-serif'
    },
    chatButton: {
      width: '60px',
      height: '60px',
      borderRadius: '50%',
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      color: 'white',
      border: 'none',
      fontSize: '24px',
      cursor: 'pointer',
      boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      transition: 'all 0.3s ease'
    },
    chatButtonHover: {
      transform: 'scale(1.05)',
      boxShadow: '0 6px 16px rgba(0, 0, 0, 0.2)'
    },
    chatContainer: {
      position: 'absolute',
      bottom: '80px',
      right: '0',
      width: '380px',
      height: '500px',
      background: 'white',
      borderRadius: '12px',
      boxShadow: '0 8px 30px rgba(0, 0, 0, 0.2)',
      overflow: 'hidden',
      display: 'flex',
      flexDirection: 'column'
    },
    chatHeader: {
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      color: 'white',
      padding: '16px',
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
      fontWeight: '600'
    },
    closeButton: {
      background: 'none',
      border: 'none',
      color: 'white',
      fontSize: '24px',
      cursor: 'pointer',
      padding: '0',
      margin: '0',
      width: '30px',
      height: '30px',
      display: 'flex',
      alignItems: 'center',
      justify: 'center',
      borderRadius: '50%'
    },
    closeButtonHover: {
      background: 'rgba(255, 255, 255, 0.2)'
    },
    messagesContainer: {
      flex: 1,
      overflowY: 'auto',
      padding: '16px',
      display: 'flex',
      flexDirection: 'column',
      gap: '12px',
      background: '#cyberpink'
    },
    message: {
      maxWidth: '80%',
      padding: '12px 16px',
      borderRadius: '18px',
      fontSize: '14px',
      lineHeight: '1.4',
      wordWrap: 'break-word'
    },
    userMessage: {
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      color: 'white',
      marginLeft: 'auto',
      textAlign: 'right'
    },
    assistantMessage: {
      background: 'white',
      color: '#333',
      marginRight: 'auto',
      border: '1px solid #e9ecef'
    },
    inputContainer: {
      padding: '16px',
      borderTop: '1px solid #e9ecef',
      background: 'white'
    },
    input: {
      width: 'calc(100% - 50px)',
      padding: '12px 16px',
      border: '1px solid #ddd',
      borderRadius: '24px',
      fontSize: '14px',
      resize: 'none',
      minHeight: '48px',
      maxHeight: '100px',
      outline: 'none',
      fontFamily: 'inherit'
    },
    inputFocus: {
      borderColor: '#667eea',
      boxShadow: '0 0 0 2px rgba(102, 126, 234, 0.2)'
    },
    sendButton: {
      width: '40px',
      height: '40px',
      borderRadius: '50%',
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      color: 'white',
      border: 'none',
      marginLeft: '8px',
      cursor: 'pointer',
      display: 'flex',
      alignItems: 'center',
      justify: 'center',
      fontSize: '16px'
    },
    loading: {
      display: 'flex',
      alignItems: 'center',
      justify: 'center',
      padding: '8px'
    },
    spinner: {
      width: '20px',
      height: '20px',
      border: '2px solid #f3f3f3',
      borderTop: '2px solid #667eea',
      borderRadius: '50%',
      animation: 'spin 1s linear infinite'
    }
  };

  return (
    <div style={styles.chatWidget}>
      {isOpen && (
        <div style={styles.chatContainer}>
          <div style={styles.chatHeader}>
            <span>AI Assistant</span>
            <button
              style={styles.closeButton}
              onClick={toggleChat}
              onMouseEnter={(e) => Object.assign(e.target.style, styles.closeButtonHover)}
              onMouseLeave={(e) => Object.assign(e.target.style, styles.closeButton)}
            >
              ×
            </button>
          </div>
          <div style={styles.messagesContainer}>
            {messages.map((msg) => (
              <div
                key={msg.id}
                style={{
                  ...styles.message,
                  ...(msg.role === 'user' ? styles.userMessage : styles.assistantMessage)
                }}
              >
                {msg.content}
              </div>
            ))}
            {isLoading && (
              <div style={{ ...styles.message, ...styles.assistantMessage, display: 'flex', alignItems: 'center' }}>
                <div style={styles.loading}>
                  <div style={styles.spinner}></div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <div style={styles.inputContainer}>
            <textarea
              ref={inputRef}
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message..."
              style={styles.input}
              onMouseEnter={(e) => Object.assign(e.target.style, styles.inputFocus)}
              onMouseLeave={(e) => Object.assign(e.target.style, styles.input)}
              disabled={isLoading}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !inputMessage.trim()}
              style={styles.sendButton}
            >
              ➤
            </button>
          </div>
        </div>
      )}
      <button
        style={styles.chatButton}
        onClick={toggleChat}
        onMouseEnter={(e) => Object.assign(e.target.style, styles.chatButtonHover)}
        onMouseLeave={(e) => Object.assign(e.target.style, styles.chatButton)}
      >
        💬
      </button>
      <style jsx>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
    </div>
  );
};

export default ChatPanel;