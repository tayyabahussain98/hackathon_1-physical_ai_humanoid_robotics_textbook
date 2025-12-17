import React, { createContext, useContext, useReducer, ReactNode } from 'react';

// Define TypeScript interfaces based on data model
export interface ChatMessage {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
  sources?: Array<{ title: string; content: string; url: string }>;
}

export interface ChatSession {
  id: string;
  messages: ChatMessage[];
  createdAt: Date;
  lastActive: Date;
  context: string;
}

export interface SelectedTextContext {
  text: string;
  pageUrl: string;
  pageTitle: string;
  timestamp: Date;
  position: { start: number; end: number };
}

// Define the state structure
interface ChatState {
  session: ChatSession | null;
  isVisible: boolean;
  isMinimized: boolean;
  selectedText: SelectedTextContext | null;
  isLoading: boolean;
  error: string | null;
}

// Define action types
type ChatAction =
  | { type: 'SET_SESSION'; payload: ChatSession }
  | { type: 'ADD_MESSAGE'; payload: ChatMessage }
  | { type: 'SET_VISIBLE'; payload: boolean }
  | { type: 'SET_MINIMIZED'; payload: boolean }
  | { type: 'SET_SELECTED_TEXT'; payload: SelectedTextContext | null }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: string | null }
  | { type: 'CLEAR_ERROR' }
  | { type: 'RESET_SESSION' };

// Load state from localStorage on initial load
const loadStateFromStorage = (): ChatState => {
  // Check if we're in a browser environment
  if (typeof window === 'undefined' || typeof localStorage === 'undefined') {
    return {
      session: null,
      isVisible: true,
      isMinimized: false,
      selectedText: null,
      isLoading: false,
      error: null,
    };
  }

  try {
    const stored = localStorage.getItem('chatState');
    if (stored) {
      const parsed = JSON.parse(stored);
      // Restore Date objects
      if (parsed.session?.createdAt) {
        parsed.session.createdAt = new Date(parsed.session.createdAt);
      }
      if (parsed.session?.lastActive) {
        parsed.session.lastActive = new Date(parsed.session.lastActive);
      }
      if (parsed.session?.messages) {
        parsed.session.messages = parsed.session.messages.map(msg => ({
          ...msg,
          timestamp: new Date(msg.timestamp)
        }));
      }
      if (parsed.selectedText?.timestamp) {
        parsed.selectedText.timestamp = new Date(parsed.selectedText.timestamp);
      }
      return parsed;
    }
  } catch (error) {
    console.error('Error loading chat state from storage:', error);
  }
  return {
    session: null,
    isVisible: true,
    isMinimized: false,
    selectedText: null,
    isLoading: false,
    error: null,
  };
};

// Initial state
const initialState: ChatState = loadStateFromStorage();

// Save state to localStorage
const saveStateToStorage = (state: ChatState) => {
  // Check if we're in a browser environment
  if (typeof window === 'undefined' || typeof localStorage === 'undefined') {
    return;
  }

  try {
    // Convert Date objects to ISO strings for serialization
    const serializableState = {
      ...state,
      session: state.session ? {
        ...state.session,
        createdAt: state.session.createdAt.toISOString(),
        lastActive: state.session.lastActive.toISOString(),
        messages: state.session.messages.map(msg => ({
          ...msg,
          timestamp: msg.timestamp.toISOString()
        }))
      } : null,
      selectedText: state.selectedText ? {
        ...state.selectedText,
        timestamp: state.selectedText.timestamp.toISOString()
      } : null
    };
    localStorage.setItem('chatState', JSON.stringify(serializableState));
  } catch (error) {
    console.error('Error saving chat state to storage:', error);
  }
};

// Reducer function
const chatReducer = (state: ChatState, action: ChatAction): ChatState => {
  let newState: ChatState;

  switch (action.type) {
    case 'SET_SESSION':
      newState = {
        ...state,
        session: action.payload,
        error: null,
      };
      break;
    case 'ADD_MESSAGE':
      if (!state.session) return state;
      // Limit message history to prevent memory issues (max 100 items)
      const updatedMessages = [...state.session.messages, action.payload];
      if (updatedMessages.length > 100) {
        // Keep only the last 100 messages
        updatedMessages.splice(0, updatedMessages.length - 100);
      }
      newState = {
        ...state,
        session: {
          ...state.session,
          messages: updatedMessages,
          lastActive: new Date(),
        },
        isLoading: false,
      };
      break;
    case 'SET_VISIBLE':
      newState = {
        ...state,
        isVisible: action.payload,
      };
      break;
    case 'SET_MINIMIZED':
      newState = {
        ...state,
        isMinimized: action.payload,
      };
      break;
    case 'SET_SELECTED_TEXT':
      newState = {
        ...state,
        selectedText: action.payload,
      };
      break;
    case 'SET_LOADING':
      newState = {
        ...state,
        isLoading: action.payload,
      };
      break;
    case 'SET_ERROR':
      newState = {
        ...state,
        error: action.payload,
        isLoading: false,
      };
      break;
    case 'CLEAR_ERROR':
      newState = {
        ...state,
        error: null,
      };
      break;
    case 'RESET_SESSION':
      newState = {
        ...initialState,
        isVisible: state.isVisible,
      };
      break;
    default:
      return state;
  }

  // Save the new state to localStorage after each action
  saveStateToStorage(newState);
  return newState;
};

// Create context
interface ChatContextType extends ChatState {
  addMessage: (message: ChatMessage) => void;
  setVisible: (visible: boolean) => void;
  setMinimized: (minimized: boolean) => void;
  setSelectedText: (selectedText: SelectedTextContext | null) => void;
  setLoading: (loading: boolean) => void;
  setError: (error: string | null) => void;
  clearError: () => void;
  resetSession: () => void;
  createNewSession: () => void;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

// Provider component
interface ChatProviderProps {
  children: ReactNode;
}

export const ChatProvider: React.FC<ChatProviderProps> = ({ children }) => {
  const [state, dispatch] = useReducer(chatReducer, initialState);

  const addMessage = (message: ChatMessage) => {
    dispatch({ type: 'ADD_MESSAGE', payload: message });
  };

  const setVisible = (visible: boolean) => {
    dispatch({ type: 'SET_VISIBLE', payload: visible });
  };

  const setMinimized = (minimized: boolean) => {
    dispatch({ type: 'SET_MINIMIZED', payload: minimized });
  };

  const setSelectedText = (selectedText: SelectedTextContext | null) => {
    dispatch({ type: 'SET_SELECTED_TEXT', payload: selectedText });
  };

  const setLoading = (loading: boolean) => {
    dispatch({ type: 'SET_LOADING', payload: loading });
  };

  const setError = (error: string | null) => {
    dispatch({ type: 'SET_ERROR', payload: error });
  };

  const clearError = () => {
    dispatch({ type: 'CLEAR_ERROR' });
  };

  const resetSession = () => {
    dispatch({ type: 'RESET_SESSION' });
  };

  const createNewSession = () => {
    const newSession: ChatSession = {
      id: `session-${Date.now()}`,
      messages: [],
      createdAt: new Date(),
      lastActive: new Date(),
      context: '',
    };
    dispatch({ type: 'SET_SESSION', payload: newSession });
  };

  // Initialize a session when the provider mounts if none exists
  React.useEffect(() => {
    if (!state.session) {
      createNewSession();
    }
  }, []);

  const contextValue: ChatContextType = {
    ...state,
    addMessage,
    setVisible,
    setMinimized,
    setSelectedText,
    setLoading,
    setError,
    clearError,
    resetSession,
    createNewSession,
  };

  return <ChatContext.Provider value={contextValue}>{children}</ChatContext.Provider>;
};

// Session timeout in milliseconds (30 minutes)
const SESSION_TIMEOUT = 30 * 60 * 1000; // 30 minutes

// Custom hook to use the chat context
export const useChatContext = (): ChatContextType => {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within a ChatProvider');
  }

  // Check for session timeout periodically
  React.useEffect(() => {
    const checkSessionTimeout = () => {
      if (context.session && context.session.lastActive) {
        const now = new Date();
        const timeDiff = now.getTime() - context.session.lastActive.getTime();

        if (timeDiff > SESSION_TIMEOUT) {
          // Session has timed out, reset it
          context.resetSession();
        }
      }
    };

    // Check every 5 minutes
    const interval = setInterval(checkSessionTimeout, 5 * 60 * 1000);

    return () => clearInterval(interval);
  }, [context.session?.lastActive, context.resetSession]);

  return context;
};