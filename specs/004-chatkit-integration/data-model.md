# Data Model: Chat Component Integration

## Entities

### Chat Message
- **Description**: Represents a single exchange in the conversation
- **Fields**:
  - id: string (unique identifier)
  - content: string (the text content of the message)
  - role: 'user' | 'assistant' (who sent the message)
  - timestamp: Date (when the message was created)
  - sources?: Array<{title: string, content: string, url: string}> (sources referenced in the response)

### Chat Session
- **Description**: Contains the conversation history for a user's interaction
- **Fields**:
  - id: string (unique session identifier)
  - messages: Array<ChatMessage> (list of messages in the conversation)
  - createdAt: Date (when the session was created)
  - lastActive: Date (when the session was last used)
  - context: string (current context, including selected text)

### Selected Text Context
- **Description**: Captures the text content and metadata when a user selects text on a documentation page
- **Fields**:
  - text: string (the selected text content)
  - pageUrl: string (URL of the page where text was selected)
  - pageTitle: string (title of the page where text was selected)
  - timestamp: Date (when the text was selected)
  - position: {start: number, end: number} (character positions of the selection)

## State Transitions

### Chat Session States
- **Inactive**: Session exists but no recent activity
- **Active**: Session currently being used for conversation
- **Minimized**: Session exists but UI is minimized
- **Cleared**: Session has been removed (expires after period of inactivity)

## Validation Rules

### Chat Message
- content must not be empty
- role must be either 'user' or 'assistant'
- timestamp must be in the past or present

### Chat Session
- must contain at least one message to be active
- createdAt must be before or equal to lastActive
- messages array must not exceed 100 items (to prevent memory issues)

### Selected Text Context
- text must be between 10 and 5000 characters (to prevent excessive context)
- pageUrl must be a valid URL
- timestamp must be in the past or present