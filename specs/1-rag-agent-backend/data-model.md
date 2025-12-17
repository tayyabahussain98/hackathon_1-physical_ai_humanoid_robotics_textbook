# Data Model: RAG Agent Backend with OpenAI Agents SDK using Gemini

## Entities

### Query
**Description**: User input text that triggers the RAG process
- **Fields**:
  - `text` (string): The query text from the user
  - `top_k` (integer, optional): Number of documents to retrieve (default: 5)
  - `threshold` (float, optional): Minimum similarity score threshold (default: 0.7)

### RetrievedDocuments
**Description**: Contextual information retrieved from Qdrant based on semantic similarity to the query
- **Fields**:
  - `documents` (list of strings): Retrieved document content
  - `metadata` (list of objects): Additional information about each document
  - `similarity_scores` (list of floats): Similarity score for each document
  - `source_urls` (list of strings): URLs of the source documents

### AgentResponse
**Description**: Generated answer from the agent using OpenAI SDK with Google Gemini backend
- **Fields**:
  - `content` (string): The agent's response text
  - `confidence` (float): Confidence score of the response
  - `sources` (list of strings): Source documents referenced in the response
  - `retrieved_context_count` (integer): Number of documents used in generating response

### APIRequest
**Description**: HTTP request containing the user query and optional parameters for the RAG process
- **Fields**:
  - `query` (string): The user's query text
  - `top_k` (integer, optional): Number of documents to retrieve
  - `threshold` (float, optional): Minimum similarity score threshold
  - `include_sources` (boolean, optional): Whether to include source information in response

### APIResponse
**Description**: HTTP response containing the agent's answer and metadata about the retrieval process
- **Fields**:
  - `answer` (string): The agent's answer to the query
  - `retrieved_documents` (list of objects): Documents retrieved during the process
  - `sources` (list of strings): Source information for the response
  - `processing_time` (float): Time taken to process the request in seconds
  - `error` (string, optional): Error message if processing failed

### OpenAICompatibleAPIClient
**Description**: Client configured to route requests to Google's Gemini API using OpenAI SDK
- **Fields**:
  - `api_key` (string): API key for accessing the service
  - `base_url` (string): Base URL for the OpenAI-compatible API endpoint
  - `model` (string): Model name to use (e.g., gemini-2.5-flash)
  - `timeout` (integer): Request timeout in seconds

## Relationships
- `Query` → `RetrievedDocuments` (one-to-many): A query can retrieve multiple documents
- `RetrievedDocuments` → `AgentResponse` (one-to-one): Retrieved documents are used to generate a single agent response
- `APIRequest` → `Query` (one-to-one): API request contains a single query
- `AgentResponse` → `APIResponse` (one-to-one): Agent response is embedded in API response

## Validation Rules
- Query text must not be empty
- top_k value must be between 1 and 20
- threshold value must be between 0.0 and 1.0
- API responses must include either an answer or an error message
- Retrieved documents must have valid similarity scores between 0.0 and 1.0