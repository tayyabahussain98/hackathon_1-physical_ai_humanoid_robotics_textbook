# Architecture Plan: Gemini RAG Chatbot

## Feature: Production-grade RAG chatbot embedded in documentation sites

### Technical Context
- **Backend Framework**: Python with FastAPI for the single backend/main.py file
- **Frontend Framework**: Next.js with App Router for the single frontend/src/app/page.js file
- **LLM Provider**: Gemini 1.5 Flash via LiteLLM
- **Vector Database**: Qdrant Cloud for document storage and retrieval
- **Metadata Storage**: Neon Postgres for conversation history and metadata
- **Chat UI**: @openai/chatkit-react for the chat interface
- **Project Management**: uv for Python package management
- **Text Selection**: Browser-native selection APIs for highlighting functionality
- **Performance Target**: <4 seconds response time for 95% of queries

### Architecture Decision Record (ADR)

#### 1. Scope and Dependencies
- **In Scope**:
  - Single-file backend implementation in backend/main.py
  - Single-file frontend implementation in frontend/src/app/page.js
  - RAG functionality with document ingestion and retrieval
  - Chat interface with streaming, memory, and source citations
  - Text selection integration for contextual conversations
  - Dark theme support

- **Out of Scope**:
  - Multi-language support beyond documentation language
  - Advanced analytics dashboard
  - Offline functionality
  - Integration with non-Docusaurus documentation systems

- **External Dependencies**:
  - Qdrant Cloud (vector database)
  - Neon Postgres (metadata storage)
  - Google Gemini API (LLM access)
  - LiteLLM (API proxy/abstraction)
  - @openai/chatkit-react (chat UI components)
  - uv (Python package manager)

#### 2. Key Decisions and Rationale
- **Options Considered**:
  - Full-stack frameworks (Next.js + FastAPI) vs. microservices
  - Single-file architecture vs. modular structure
  - Different LLM providers (OpenAI, Anthropic, Gemini)
  - Various vector databases (Pinecone, Weaviate, Qdrant, Chroma)

- **Trade-offs**:
  - Single-file architecture provides simplicity but may impact maintainability as features grow
  - RAG approach provides accurate responses from specific docs but requires document ingestion pipeline
  - Client-side vs. server-side text selection handling

- **Rationale**:
  - Single-file architecture chosen to meet constitution requirements
  - FastAPI chosen for Python backend due to async support and OpenAPI generation
  - Next.js App Router chosen for modern React framework with SSR capabilities
  - Gemini 1.5 Flash chosen for performance and cost-effectiveness for RAG use case
  - Qdrant Cloud chosen for managed vector database with good Python integration

- **Principles**:
  - Follow single-file architecture constraints from constitution
  - Maintain under 4-second response time SLA
  - Preserve user privacy and data security
  - Ensure accessibility with dark theme support

#### 3. Interfaces and API Contracts
- **Public APIs**:
  - POST /api/chat - Process chat messages with RAG context
  - POST /api/ingest - Ingest documentation content into vector store
  - GET /api/health - Health check endpoint

- **Versioning Strategy**:
  - API versioning through URL path (v1, v2) when needed
  - Semantic versioning for client libraries

- **Idempotency, Timeouts, Retries**:
  - Chat endpoints are not idempotent (new messages each time)
  - 30-second timeout for chat requests
  - Client-side retry logic with exponential backoff

- **Error Taxonomy**:
  - 400: Bad Request - Invalid input format
  - 401: Unauthorized - Missing or invalid API key
  - 429: Too Many Requests - Rate limiting
  - 500: Internal Server Error - Processing failure
  - 503: Service Unavailable - External service unavailable

#### 4. Non-Functional Requirements (NFRs) and Budgets
- **Performance**:
  - 95% of queries return response in under 4 seconds
  - Support 100 concurrent users during peak usage
  - 99.9% uptime during business hours

- **Reliability**:
  - Automatic retry for failed vector database operations
  - Circuit breaker pattern for external LLM API calls
  - Graceful degradation when vector database is unavailable

- **Security**:
  - API key validation for all endpoints
  - Input sanitization to prevent injection attacks
  - Encryption in transit for all data transmission
  - No PII storage in conversation metadata

- **Cost**:
  - Optimize token usage to minimize LLM costs
  - Efficient vector search to minimize database costs
  - Caching strategies to reduce redundant API calls

#### 5. Data Management and Migration
- **Source of Truth**:
  - Documentation content in Qdrant Cloud
  - Conversation metadata in Neon Postgres

- **Schema Evolution**:
  - Use Alembic for database schema migrations
  - Versioned vector embeddings to handle model updates
  - Backward-compatible API changes

- **Migration and Rollback**:
  - Blue-green deployment strategy
  - Database migration scripts with rollback capabilities
  - Vector database reindexing procedures when needed

- **Data Retention**:
  - Conversation history retention policy (e.g., 90 days)
  - Automatic cleanup of old conversation data
  - GDPR-compliant data deletion procedures

#### 6. Operational Readiness
- **Observability**:
  - Structured logging with request IDs
  - Performance metrics for response times
  - Error rate monitoring and alerting
  - LLM token usage tracking

- **Alerting**:
  - High error rate (>5%)
  - Response time degradation (>6 seconds average)
  - Service unavailability
  - High resource utilization

- **Runbooks**:
  - Database connection issues
  - LLM API failures
  - Vector database performance degradation
  - Security incident response

- **Deployment and Rollback**:
  - CI/CD pipeline with automated testing
  - Canary deployments for new features
  - Automated rollback on health check failures

- **Feature Flags**:
  - Toggle for dark/light theme
  - Rate limiting controls
  - Experimental RAG features

#### 7. Risk Analysis and Mitigation
- **Top 3 Risks**:
  1. LLM API availability/cost - Mitigation: Implement circuit breaker, caching, and cost monitoring
  2. Vector database performance - Mitigation: Optimize queries, implement caching, monitor performance
  3. Single-file architecture complexity - Mitigation: Careful code organization and documentation

- **Blast Radius**:
  - Backend failures affect chat functionality but not documentation
  - Database failures affect conversation history but not new chats
  - LLM unavailability affects responses but not interface

- **Kill Switches/Guardrails**:
  - Circuit breaker for LLM API calls
  - Rate limiting to prevent abuse
  - Safe fallback responses when RAG fails

#### 8. Evaluation and Validation
- **Definition of Done**:
  - All functional requirements implemented
  - Performance targets met (sub-4s response time)
  - Security scanning passed
  - Cross-browser compatibility verified
  - Dark theme functionality tested

- **Output Validation**:
  - Automated tests for API endpoints
  - Integration tests for RAG functionality
  - Performance tests to validate response times
  - Security scanning for vulnerabilities

#### 9. Constitution Check
- ✅ All architecture decisions comply with project constitution v3.0
- ✅ Single file backend architecture (backend/main.py only)
- ✅ Single file frontend architecture (frontend/src/app/page.js only)
- ✅ Gemini via LiteLLM only
- ✅ Official OpenAI ChatKit React (@openai/chatkit-react) only
- ✅ uv project management only
- ✅ Selected-text mode functionality included
- ✅ All code via Claude only