// Gemini RAG Chatbot Integration Script
// This script integrates the chatbot into Docusaurus documentation sites

(function() {
  'use strict';

  // Configuration
  const CONFIG = {
    backendUrl: 'http://localhost:8000', // Will be replaced in production
    chatElementId: 'gemini-rag-chatbot',
    buttonSize: '60px',
    buttonPosition: {
      bottom: '20px',
      right: '20px'
    }
  };

  // Main chatbot class
  class GeminiRAGChatbot {
    constructor() {
      this.isOpen = false;
      this.selectedText = '';
      this.init();
    }

    init() {
      // Wait for page to load
      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', () => this.setupChatbot());
      } else {
        this.setupChatbot();
      }
    }

    setupChatbot() {
      // Create chat button
      this.createChatButton();

      // Setup text selection listener
      this.setupTextSelection();
    }

    createChatButton() {
      // Create the floating chat button
      const chatButton = document.createElement('div');
      chatButton.id = 'gemini-chat-button';
      chatButton.innerHTML = '💬';
      chatButton.style.position = 'fixed';
      chatButton.style.bottom = CONFIG.buttonPosition.bottom;
      chatButton.style.right = CONFIG.buttonPosition.right;
      chatButton.style.width = CONFIG.buttonSize;
      chatButton.style.height = CONFIG.buttonSize;
      chatButton.style.borderRadius = '50%';
      chatButton.style.backgroundColor = '#3b82f6';
      chatButton.style.color = 'white';
      chatButton.style.display = 'flex';
      chatButton.style.alignItems = 'center';
      chatButton.style.justifyContent = 'center';
      chatButton.style.fontSize = '24px';
      chatButton.style.cursor = 'pointer';
      chatButton.style.zIndex = '10000';
      chatButton.style.boxShadow = '0 4px 12px rgba(0,0,0,0.15)';
      chatButton.style.fontWeight = 'bold';
      chatButton.title = 'Open Gemini RAG Chatbot';

      chatButton.addEventListener('click', () => this.toggleChat());

      document.body.appendChild(chatButton);
    }

    setupTextSelection() {
      document.addEventListener('mouseup', () => {
        const selection = window.getSelection();
        if (selection && selection.toString().trim() !== '') {
          this.selectedText = selection.toString().trim();

          // Show confirmation popup near selection
          this.showSelectionPopup(selection);
        }
      });
    }

    showSelectionPopup(selection) {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      // Remove existing popup if any
      const existingPopup = document.getElementById('gemini-selection-popup');
      if (existingPopup) {
        existingPopup.remove();
      }

      const popup = document.createElement('div');
      popup.id = 'gemini-selection-popup';
      popup.style.position = 'fixed';
      popup.style.left = (rect.right + 10) + 'px';
      popup.style.top = (rect.top - 40) + 'px';
      popup.style.maxWidth = '300px';
      popup.style.backgroundColor = 'white';
      popup.style.border = '1px solid #e5e7eb';
      popup.style.borderRadius = '8px';
      popup.style.padding = '12px';
      popup.style.boxShadow = '0 10px 15px -3px rgba(0,0,0,0.1)';
      popup.style.zIndex = '10001';
      popup.style.fontSize = '14px';
      popup.style.fontFamily = 'inherit';

      const textPreview = this.selectedText.length > 60
        ? this.selectedText.substring(0, 60) + '...'
        : this.selectedText;

      popup.innerHTML = `
        <div style="margin-bottom: 8px;">
          <strong>You selected:</strong> "${textPreview}"
        </div>
        <div style="display: flex; gap: 8px;">
          <button id="ask-gemini-btn" style="
            background-color: #3b82f6;
            color: white;
            border: none;
            border-radius: 4px;
            padding: 6px 12px;
            font-size: 12px;
            cursor: pointer;
          ">Ask Gemini</button>
          <button id="cancel-selection-btn" style="
            background-color: #e5e7eb;
            color: #374151;
            border: none;
            border-radius: 4px;
            padding: 6px 12px;
            font-size: 12px;
            cursor: pointer;
          ">Cancel</button>
        </div>
      `;

      document.body.appendChild(popup);

      document.getElementById('ask-gemini-btn').addEventListener('click', () => {
        this.openChatWithSelection(this.selectedText);
        popup.remove();
      });

      document.getElementById('cancel-selection-btn').addEventListener('click', () => {
        popup.remove();
      });
    }

    openChatWithSelection(selectedText) {
      this.selectedText = selectedText;
      this.toggleChat();
    }

    toggleChat() {
      if (this.isOpen) {
        this.closeChat();
      } else {
        this.openChat();
      }
    }

    openChat() {
      this.isOpen = true;

      // Create chat container
      const chatContainer = document.createElement('div');
      chatContainer.id = CONFIG.chatElementId;
      chatContainer.style.position = 'fixed';
      chatContainer.style.bottom = '80px';
      chatContainer.style.right = '20px';
      chatContainer.style.width = '400px';
      chatContainer.style.height = '500px';
      chatContainer.style.backgroundColor = 'white';
      chatContainer.style.border = '1px solid #e5e7eb';
      chatContainer.style.borderRadius = '12px';
      chatContainer.style.boxShadow = '0 20px 25px -5px rgba(0,0,0,0.1)';
      chatContainer.style.zIndex = '10000';
      chatContainer.style.display = 'flex';
      chatContainer.style.flexDirection = 'column';
      chatContainer.style.overflow = 'hidden';

      chatContainer.innerHTML = `
        <div style="
          background-color: #3b82f6;
          color: white;
          padding: 16px;
          display: flex;
          justify-content: space-between;
          align-items: center;
        ">
          <h3 style="margin: 0; font-size: 16px;">Gemini RAG Chatbot</h3>
          <button id="close-chat-btn" style="
            background: none;
            border: none;
            color: white;
            font-size: 20px;
            cursor: pointer;
          ">&times;</button>
        </div>
        <div id="chat-messages" style="
          flex: 1;
          padding: 16px;
          overflow-y: auto;
          background-color: #f9fafb;
        ">
          <div style="padding: 20px; text-align: center; color: #6b7280;">
            <p>Hello! I'm your Gemini RAG assistant.</p>
            <p>I can help answer questions about this documentation.</p>
          </div>
        </div>
        <div style="padding: 16px; border-top: 1px solid #e5e7eb; background-color: white;">
          <div style="display: flex; gap: 8px;">
            <input
              type="text"
              id="chat-input"
              placeholder="Ask about the documentation..."
              style="
                flex: 1;
                padding: 10px 12px;
                border: 1px solid #d1d5db;
                border-radius: 6px;
                font-size: 14px;
              "
            />
            <button id="send-message-btn" style="
              background-color: #3b82f6;
              color: white;
              border: none;
              border-radius: 6px;
              padding: 0 16px;
              cursor: pointer;
            ">Send</button>
          </div>
        </div>
      `;

      document.body.appendChild(chatContainer);

      // Add event listeners
      document.getElementById('close-chat-btn').addEventListener('click', () => this.closeChat());
      document.getElementById('send-message-btn').addEventListener('click', () => this.sendMessage());
      document.getElementById('chat-input').addEventListener('keypress', (e) => {
        if (e.key === 'Enter') {
          this.sendMessage();
        }
      });

      // If we have selected text, use it as context
      if (this.selectedText) {
        const contextMsg = document.createElement('div');
        contextMsg.style.marginBottom = '10px';
        contextMsg.style.padding = '8px 12px';
        contextMsg.style.backgroundColor = '#dbeafe';
        contextMsg.style.borderRadius = '6px';
        contextMsg.style.fontSize = '12px';
        contextMsg.innerHTML = `<strong>Context:</strong> "${this.selectedText.substring(0, 100)}${this.selectedText.length > 100 ? '...' : ''}"`;
        document.getElementById('chat-messages').appendChild(contextMsg);
      }
    }

    closeChat() {
      this.isOpen = false;
      const chatEl = document.getElementById(CONFIG.chatElementId);
      if (chatEl) {
        chatEl.remove();
      }
    }

    async sendMessage() {
      const inputEl = document.getElementById('chat-input');
      const message = inputEl.value.trim();
      if (!message) return;

      const chatMessages = document.getElementById('chat-messages');

      // Add user message
      const userMsg = document.createElement('div');
      userMsg.style.marginBottom = '10px';
      userMsg.style.padding = '8px 12px';
      userMsg.style.backgroundColor = '#dbeafe';
      userMsg.style.borderRadius = '6px';
      userMsg.style.textAlign = 'right';
      userMsg.innerHTML = `<strong>You:</strong> ${this.escapeHtml(message)}`;
      chatMessages.appendChild(userMsg);

      // Clear input
      inputEl.value = '';

      // Show typing indicator
      const typingIndicator = document.createElement('div');
      typingIndicator.id = 'typing-indicator';
      typingIndicator.style.marginBottom = '10px';
      typingIndicator.style.padding = '8px 12px';
      typingIndicator.style.backgroundColor = '#f3f4f6';
      typingIndicator.style.borderRadius = '6px';
      typingIndicator.innerHTML = '<em>Gemini is thinking...</em>';
      chatMessages.appendChild(typingIndicator);

      // Scroll to bottom
      chatMessages.scrollTop = chatMessages.scrollHeight;

      try {
        // In a real implementation, this would call the backend API
        // For now, we'll simulate a response
        setTimeout(() => {
          typingIndicator.remove();

          const botMsg = document.createElement('div');
          botMsg.style.marginBottom = '10px';
          botMsg.style.padding = '8px 12px';
          botMsg.style.backgroundColor = '#f0fdf4';
          botMsg.style.borderRadius = '6px';
          botMsg.innerHTML = `<strong>Gemini:</strong> This is a simulated response. In the full implementation, this would connect to the backend API at ${CONFIG.backendUrl} to get a real response based on the documentation and your query: "${this.escapeHtml(message)}".`;
          chatMessages.appendChild(botMsg);

          // Scroll to bottom
          chatMessages.scrollTop = chatMessages.scrollHeight;
        }, 1500);
      } catch (error) {
        typingIndicator.remove();

        const errorMsg = document.createElement('div');
        errorMsg.style.marginBottom = '10px';
        errorMsg.style.padding = '8px 12px';
        errorMsg.style.backgroundColor = '#fee2e2';
        errorMsg.style.borderRadius = '6px';
        errorMsg.innerHTML = `<strong>Error:</strong> Failed to get response from Gemini. Please check the connection.`;
        chatMessages.appendChild(errorMsg);
      }
    }

    escapeHtml(text) {
      const div = document.createElement('div');
      div.textContent = text;
      return div.innerHTML;
    }
  }

  // Initialize the chatbot when the script loads
  new GeminiRAGChatbot();

})();