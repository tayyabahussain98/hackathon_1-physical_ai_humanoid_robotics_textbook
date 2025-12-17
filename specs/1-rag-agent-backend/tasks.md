# Tasks: RAG Agent Backend with OpenAI Agents SDK using Gemini

**Feature**: 1-rag-agent-backend
**Branch**: 1-rag-agent-backend
**Date**: 2025-12-16
**Spec**: specs/1-rag-agent-backend/spec.md
**Plan**: specs/1-rag-agent-backend/plan.md

## Implementation Strategy

Build the RAG agent backend incrementally following the user story priorities:
- **MVP**: User Story 1 (Initialize FastAPI Backend with OpenAI Agent) - Provides basic functionality
- **Phase 2**: User Story 4 (API Endpoint) - Exposes the functionality to users
- **Phase 3**: User Story 2 (Qdrant Integration) - Adds RAG capability
- **Phase 4**: User Story 3 (Context Injection) - Completes the RAG loop

## Phase 1: Setup (Project Initialization)

**Goal**: Initialize the project structure and dependencies following the planned architecture

- [ ] T001 Create backend directory structure per implementation plan
- [ ] T002 [P] Create requirements.txt with FastAPI, openai-agents, qdrant-client, python-dotenv, pydantic, pytest
- [ ] T003 [P] Create .env.example with GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, GEMINI_MODEL_NAME
- [ ] T004 [P] Create README.md with project description and setup instructions
- [ ] T005 Create main.py entry point file
- [ ] T006 Create agents/__init__.py
- [ ] T007 Create agents/rag_agent.py
- [ ] T008 Create agents/system_prompt.py
- [ ] T009 Create api/__init__.py
- [ ] T010 Create api/router.py
- [ ] T011 Create api/models.py
- [ ] T012 Create services/__init__.py
- [ ] T013 Create services/qdrant_service.py
- [ ] T014 Create services/context_injector.py
- [ ] T015 Create config/__init__.py
- [ ] T016 Create config/settings.py
- [ ] T017 Create utils/__init__.py
- [ ] T018 Create utils/helpers.py
- [ ] T019 Create tests/__init__.py
- [ ] T020 Create tests/test_api.py
- [ ] T021 Create tests/test_agent.py
- [ ] T022 Create tests/test_qdrant.py

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Implement foundational components that all user stories depend on

- [ ] T023 [P] Implement config/settings.py with environment variable loading and validation
- [ ] T024 [P] Implement api/models.py with Pydantic models for QueryRequest and QueryResponse
- [ ] T025 [P] Create system prompt in agents/system_prompt.py for RAG agent
- [ ] T026 [P] Implement OpenAI-compatible client configuration in agents/rag_agent.py
- [ ] T027 [P] Create basic FastAPI app in main.py with CORS and basic configuration
- [ ] T028 [P] Implement basic Qdrant client initialization in services/qdrant_service.py
- [ ] T029 [P] Create basic context injection utilities in services/context_injector.py

## Phase 3: [US1] Initialize FastAPI Backend with OpenAI Agent using Gemini

**Goal**: Create a working FastAPI server that hosts an agent using OpenAI SDK configured to work with Google Gemini, capable of processing queries

**Independent Test**: Can be fully tested by starting the server and making a simple API call to the agent endpoint, which should return a response from the Gemini model via OpenAI SDK

**Tasks**:
- [ ] T030 [P] [US1] Implement OpenAI-compatible API client in agents/rag_agent.py with Google Gemini configuration
- [ ] T031 [P] [US1] Create RAG agent class in agents/rag_agent.py with system instructions
- [ ] T032 [P] [US1] Test basic agent functionality with simple prompt in test environment
- [ ] T033 [US1] Create basic FastAPI app in main.py with agent initialization
- [ ] T034 [US1] Test FastAPI app startup with agent initialization
- [ ] T035 [US1] Verify agent responds to basic queries with Gemini model

## Phase 4: [US4] Expose Clean API Endpoint for Queries

**Goal**: Create a clean API endpoint that accepts user queries and returns agent responses

**Independent Test**: Can be fully tested by making HTTP requests to the API endpoint and receiving properly formatted responses

**Tasks**:
- [ ] T036 [P] [US4] Implement basic /query POST endpoint in api/router.py
- [ ] T037 [P] [US4] Connect API endpoint to agent in api/router.py
- [ ] T038 [P] [US4] Add request/response validation to /query endpoint
- [ ] T039 [US4] Add error handling to API endpoint
- [ ] T040 [US4] Integrate API router with main FastAPI app
- [ ] T041 [US4] Test basic API functionality with curl requests
- [ ] T042 [US4] Verify proper response formatting according to API contract

## Phase 5: [US2] Integrate Vector Retrieval from Qdrant

**Goal**: Enable the agent to retrieve relevant content from Qdrant before generating responses

**Independent Test**: Can be fully tested by making a query and verifying that the system retrieves relevant documents from Qdrant based on the query content

**Tasks**:
- [ ] T043 [P] [US2] Implement Qdrant client connection with credentials in services/qdrant_service.py
- [ ] T044 [P] [US2] Create semantic search method in services/qdrant_service.py
- [ ] T045 [P] [US2] Implement document retrieval with similarity scoring in qdrant_service.py
- [ ] T046 [P] [US2] Add source URL and metadata retrieval in qdrant_service.py
- [ ] T047 [US2] Test Qdrant connection and basic retrieval functionality
- [ ] T048 [US2] Verify top-k document retrieval with similarity scores
- [ ] T049 [US2] Test retrieval with various query types and verify relevance

## Phase 6: [US3] Inject Retrieved Context into Agent Reasoning

**Goal**: Ensure the agent incorporates retrieved context when generating responses and keeps responses grounded in retrieved content

**Independent Test**: Can be fully tested by submitting queries and verifying that responses contain information from the retrieved documents rather than fabricated content

**Tasks**:
- [ ] T050 [P] [US3] Implement context injection logic in services/context_injector.py
- [ ] T051 [P] [US3] Create prompt formatting with retrieved context in context_injector.py
- [ ] T052 [P] [US3] Implement grounded response validation in context_injector.py
- [ ] T053 [US3] Integrate context injection with agent in agents/rag_agent.py
- [ ] T054 [US3] Modify API endpoint to use full RAG flow (retrieve + inject + respond)
- [ ] T055 [US3] Test that responses are grounded in retrieved content
- [ ] T056 [US3] Verify agent indicates when no relevant documents are found

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Complete the implementation with error handling, performance optimization, and production readiness

- [ ] T057 Add comprehensive error handling for API unavailability (Qdrant, Gemini)
- [ ] T058 Implement request/response logging and monitoring
- [ ] T059 Add rate limiting and request validation
- [ ] T060 Implement proper shutdown procedures for services
- [ ] T061 Add performance monitoring and response time tracking
- [ ] T062 Create comprehensive test suite for all components
- [ ] T063 Add documentation and usage examples to README.md
- [ ] T064 Implement token limit management for context window
- [ ] T065 Add OAuth 2.0 authentication to API endpoints
- [ ] T066 Perform end-to-end testing of complete RAG flow
- [ ] T067 Optimize response time to meet <10 second goal
- [ ] T068 Add health check endpoint for monitoring
- [ ] T069 Update OpenAPI documentation with all implemented features

## Dependencies

**User Story Completion Order**:
1. US1 (Initialize FastAPI Backend) - Foundation for all other stories
2. US4 (API Endpoint) - Exposes the agent functionality
3. US2 (Qdrant Integration) - Adds RAG capability
4. US3 (Context Injection) - Completes the RAG loop

**Dependencies**:
- US2 depends on US1 (needs agent to inject context into)
- US3 depends on US2 (needs retrieved documents to inject)
- US4 depends on US1 (needs agent to connect to endpoint)

## Parallel Execution Examples

**Per User Story**:
- **US1**: T030-T032 can run in parallel (agent implementation), T033-T035 can run in parallel (integration)
- **US4**: T036-T038 can run in parallel (endpoint implementation), T039-T042 can run in parallel (integration and testing)
- **US2**: T043-T045 can run in parallel (service implementation), T046-T049 can run in parallel (testing)
- **US3**: T050-T052 can run in parallel (service implementation), T053-T056 can run in parallel (integration and testing)

## MVP Scope

**Minimum Viable Product**: US1 + US4 (T001-T042) - Basic FastAPI server with OpenAI agent that responds to queries via API endpoint, providing immediate value before full RAG functionality.