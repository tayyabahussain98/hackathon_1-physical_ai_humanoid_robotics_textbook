// Utility functions for API communication with FastAPI RAG endpoint

export interface ChatRequest {
  message: string;
  context?: string;
  sessionId?: string;
}

export interface ChatResponse {
  id: string;
  content: string;
  sources?: Array<{ title: string; url: string; content: string }>;
  createdAt: string;
}

export interface SessionRequest {
  initialContext?: string;
}

export interface SessionResponse {
  sessionId: string;
  createdAt: string;
}

export interface ContextRequest {
  sessionId: string;
  context: string;
}

// API endpoints configuration
const API_BASE_URL = 'http://localhost:8000';
const RAG_ENDPOINT = `${API_BASE_URL}/query`;

// Function to send a message to the RAG backend
export const sendMessageToRAG = async (
  message: string,
  context?: string,
  sessionId?: string
): Promise<ChatResponse> => {
  try {
    const requestBody: any = {
      query: message,
      top_k: 5,
      threshold: 0.7,
    };

    // Add context if provided
    if (context) {
      requestBody.context = context;
    }

    // Add session ID if provided
    if (sessionId) {
      requestBody.session_id = sessionId;
    }

    const response = await fetch(RAG_ENDPOINT, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
    });

    if (!response.ok) {
      throw new Error(`RAG API error: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();

    // Transform the response to match our ChatResponse interface
    return {
      id: `msg-${Date.now()}`,
      content: data.response || data.answer || data.result || 'No response from RAG system',
      sources: data.sources || data.references || [],
      createdAt: new Date().toISOString(),
    };
  } catch (error) {
    console.error('Error sending message to RAG:', error);
    throw error;
  }
};

// Function to create a new chat session
export const createChatSession = async (
  initialContext?: string
): Promise<SessionResponse> => {
  try {
    const requestBody: SessionRequest = {};
    if (initialContext) {
      requestBody.initialContext = initialContext;
    }

    // In a real implementation, this would call an endpoint to create a session
    // For now, we'll simulate creating a session
    return {
      sessionId: `session-${Date.now()}`,
      createdAt: new Date().toISOString(),
    };
  } catch (error) {
    console.error('Error creating chat session:', error);
    throw error;
  }
};

// Function to update context for a session
export const updateSessionContext = async (
  sessionId: string,
  context: string
): Promise<boolean> => {
  try {
    // In a real implementation, this would call an endpoint to update session context
    // For now, we'll just return true to simulate success
    return true;
  } catch (error) {
    console.error('Error updating session context:', error);
    throw error;
  }
};

// Function to validate selected text length according to data model rules
export const validateSelectedText = (text: string): { isValid: boolean; error?: string } => {
  if (text.length < 10) {
    return {
      isValid: false,
      error: 'Selected text is too short. Must be at least 10 characters.',
    };
  }

  if (text.length > 5000) {
    return {
      isValid: false,
      error: 'Selected text is too long. Must be no more than 5000 characters.',
    };
  }

  return { isValid: true };
};

// Function to handle API errors gracefully
export const handleAPIError = (error: any): string => {
  if (error.name === 'TypeError' && error.message.includes('fetch')) {
    return 'Network error: Unable to connect to the RAG backend. Please ensure the FastAPI server is running on http://localhost:8000.';
  }

  if (error.message) {
    return `Error: ${error.message}`;
  }

  return 'An unknown error occurred while communicating with the RAG backend.';
};