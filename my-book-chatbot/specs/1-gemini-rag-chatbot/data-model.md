# Data Model: Gemini RAG Chatbot

## Entities

### 1. User
**Description**: Represents a user interacting with the chatbot
- **Fields**:
  - id (string): Unique identifier for the user
  - session_id (string): Session identifier for anonymous users
  - created_at (timestamp): When the user session was created
  - last_active (timestamp): When the user was last active

**Validation**:
- id must be unique
- session_id must be unique per session
- timestamps must be in ISO 8601 format

### 2. Conversation
**Description**: Represents a conversation thread between user and chatbot
- **Fields**:
  - id (string): Unique identifier for the conversation
  - user_id (string): Reference to the user
  - created_at (timestamp): When the conversation started
  - updated_at (timestamp): When the conversation was last updated
  - title (string): Auto-generated title based on first query

**Validation**:
- id must be unique
- user_id must reference an existing user
- timestamps must be in ISO 8601 format
- title must be 50 characters or less

### 3. Message
**Description**: Represents a single message in a conversation
- **Fields**:
  - id (string): Unique identifier for the message
  - conversation_id (string): Reference to the conversation
  - role (string): 'user' or 'assistant'
  - content (string): The message content
  - sources (array): List of source documents referenced
  - created_at (timestamp): When the message was created
  - metadata (object): Additional message metadata

**Validation**:
- id must be unique
- conversation_id must reference an existing conversation
- role must be either 'user' or 'assistant'
- content must be 10000 characters or less
- sources must be an array of valid source objects
- timestamps must be in ISO 8601 format

### 4. Document
**Description**: Represents a document chunk in the vector database
- **Fields**:
  - id (string): Unique identifier for the document chunk
  - original_id (string): Identifier from source documentation
  - content (string): The document content chunk
  - metadata (object): Source information and additional metadata
  - embedding (array): Vector embedding for similarity search
  - created_at (timestamp): When the document was indexed

**Validation**:
- id must be unique
- content must be 10000 characters or less per chunk
- embedding must be a valid vector array
- timestamps must be in ISO 8601 format

### 5. Source
**Description**: Represents a reference to a specific document in a response
- **Fields**:
  - id (string): Unique identifier for the source reference
  - document_id (string): Reference to the document
  - snippet (string): Relevant text snippet from the document
  - url (string): URL to the original documentation location
  - relevance_score (number): Score indicating relevance to the query

**Validation**:
- id must be unique
- document_id must reference an existing document
- snippet must be 500 characters or less
- url must be a valid URL format
- relevance_score must be between 0 and 1

## Relationships

### Conversation → Message
- One conversation can have many messages
- Foreign key: conversation_id in Message references id in Conversation
- Cascade delete: When conversation is deleted, all messages are deleted

### Message → Source
- One message can reference many sources
- Junction: sources field in Message contains array of Source objects
- No direct foreign key but logical relationship

### Document → Source
- One document can be referenced by many sources
- Foreign key: document_id in Source references id in Document

## State Transitions

### Message States
- PENDING: Message is being processed by the LLM
- COMPLETED: Message has been fully generated
- ERROR: Error occurred during message generation

### Document States
- PENDING: Document is being processed for ingestion
- INDEXED: Document has been successfully indexed in vector database
- FAILED: Error occurred during indexing

## Constraints

1. **Data Integrity**:
   - All foreign key relationships must be valid
   - Required fields cannot be null
   - Unique constraints must be enforced

2. **Performance**:
   - Message content should be limited to prevent oversized records
   - Document chunks should be optimized for RAG performance
   - Indexes should be created on frequently queried fields

3. **Security**:
   - No PII should be stored in messages or metadata
   - API keys and sensitive data should be stored separately
   - All user inputs should be sanitized