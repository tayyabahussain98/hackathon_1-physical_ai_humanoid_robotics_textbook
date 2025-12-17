# Quickstart: Chat Component Integration

## Prerequisites
- Node.js 16+ installed
- Docusaurus project already set up
- FastAPI RAG backend running on http://localhost:8000

## Setup Steps

### 1. Install Dependencies
The Vercel AI SDK is already installed in the project (ai package). If needed, install it with:
```bash
npm install ai
```

### 2. Create Chat Component
Create the main chat UI component:

```bash
# Create the components directory if it doesn't exist
mkdir -p src/components
```

Create `src/components/ChatComponent.tsx` with the chat UI implementation using the `useChat` hook.

### 3. Create Chat Context
Create `src/contexts/ChatContext.tsx` to manage chat state across the application.

### 4. Integrate with Docusaurus Layout
Create or modify `src/theme/Layout/index.tsx` to inject the chat component globally.

### 5. Configure API Connection
Set up the connection between the chat component and the FastAPI RAG endpoint at http://localhost:8000/query.

## Running the Integration

1. Start your FastAPI RAG backend:
```bash
cd backend
uvicorn main:app --reload --port 8000
```

2. Start the Docusaurus development server:
```bash
npm run start
```

3. The chat component should appear on all documentation pages.

## Testing the Integration

1. Navigate to any documentation page
2. Verify the chat component appears consistently
3. Type a question and submit it
4. Verify the response comes from the RAG backend
5. Select text on the page and verify the "Ask about selection" functionality works
6. Navigate to different pages and verify the chat session persists
7. Test minimize/expand functionality

## Configuration Options

- API endpoint URL: Can be configured via environment variables
- Chat position: Can be adjusted in CSS
- Session timeout: Configurable duration before session clears
- Context character limits: Configurable for selected text