# Task Implementation: Gemini RAG Chatbot

## Feature: Production-grade RAG chatbot embedded in documentation sites

## Dependencies
- User Story 1 (Basic Chat) must be completed before User Story 2 (RAG Enhancement)
- User Story 1 (Basic Chat) must be completed before User Story 3 (UI Enhancement)

## Parallel Execution Examples
- [US1] Backend API implementation can run in parallel with [US1] Frontend UI implementation
- [US2] Vector database setup can run in parallel with [US2] Ingestion pipeline implementation
- [US3] Dark theme implementation can run in parallel with [US3] Highlight functionality

## Implementation Strategy
- MVP: Complete User Story 1 (Basic Chat) for minimum viable functionality
- Incremental Delivery: Add RAG capabilities, then UI enhancements
- Each user story is independently testable with clear acceptance criteria

## Phase 1: Setup
### Goal: Initialize project structure and dependencies

- [X] T001 Run uv init to create Python project structure
- [X] T002 Create virtual environment with uv venv && source .venv/bin/activate
- [X] T003 Run npx create-next-app@latest frontend --use-npm --no-typescript --tailwind --eslint --app --src-dir --import-alias "@/*" --yes
- [X] T004 Create backend/ folder
- [X] T005 Create backend/main.py (empty file)
- [X] T006 Create backend/requirements.txt with exact content as specified
- [X] T007 Create .env file at root with environment variables

## Phase 2: Foundational
### Goal: Set up core infrastructure and dependencies

- [X] T008 Install backend dependencies with uv pip install -r backend/requirements.txt
- [X] T009 Install @openai/chatkit-react in frontend directory
- [X] T010 Set up CORS middleware in backend/main.py
- [X] T011 Implement basic FastAPI app structure in backend/main.py
- [X] T012 Create basic Next.js page structure in frontend/src/app/page.tsx

## Phase 3: [US1] Basic Chat Functionality
### Goal: Implement core chat functionality with ChatKit integration

**Independent Test Criteria**: User can open chat panel, send messages, and receive responses from Gemini

- [X] T013 [P] [US1] Implement Custom Store class with ALL 14 required abstract methods in backend/main.py
- [X] T014 [P] [US1] Implement correct imports (chatkit.server, chatkit.store) in backend/main.py
- [X] T015 [P] [US1] Implement LiteLLM Gemini model (LitellmModel) in backend/main.py
- [X] T016 [P] [US1] Create Agent with instructions in backend/main.py
- [X] T017 [P] [US1] Implement ID collision fix (mapping dict) in backend/main.py
- [X] T018 [P] [US1] Implement ThreadItemConverter for full conversation memory in backend/main.py
- [X] T019 [P] [US1] Create /chatkit/token endpoint in backend/main.py
- [X] T020 [P] [US1] Create /debug/debug/threads endpoint in backend/main.py
- [X] T021 [P] [US1] Implement token fetch from backend in frontend/src/app/page.tsx
- [X] T022 [P] [US1] Create floating button (bottom-right) in frontend/src/app/page.tsx
- [X] T023 [P] [US1] Implement ChatKit React panel in popup in frontend/src/app/page.tsx
- [ ] T024 [US1] Test basic chat functionality with uvicorn backend.main:app --reload and npm run dev

## Phase 4: [US2] RAG Implementation
### Goal: Implement Retrieval Augmented Generation with document ingestion

**Independent Test Criteria**: User can ingest documentation and get responses based on document content

- [X] T025 [P] [US2] Implement ingestion CLI (python main.py --ingest) in backend/main.py
- [X] T026 [P] [US2] Set up Qdrant client connection in backend/main.py
- [X] T027 [P] [US2] Implement document chunking with langchain-text-splitters in backend/main.py
- [X] T028 [P] [US2] Implement vector storage in Qdrant in backend/main.py
- [X] T029 [P] [US2] Implement document retrieval for RAG in backend/main.py
- [X] T030 [P] [US2] Update Agent to use RAG context in backend/main.py
- [X] T031 [P] [US2] Implement source citations in responses in backend/main.py
- [X] T032 [US2] Test ingestion functionality with python backend/main.py --ingest
- [ ] T033 [US2] Test RAG functionality with document-based queries

## Phase 5: [US3] UI Enhancement and Text Selection
### Goal: Implement text highlighting functionality and UI enhancements

**Independent Test Criteria**: User can highlight text in documentation and initiate chat with selected content

- [X] T034 [P] [US3] Implement highlight detection in frontend/src/app/page.tsx
- [X] T035 [P] [US3] Implement highlight confirmation UI in frontend/src/app/page.tsx
- [X] T036 [P] [US3] Implement sendMessage with selected text in frontend/src/app/page.tsx
- [X] T037 [P] [US3] Implement dark theme support in frontend/src/app/page.tsx
- [X] T038 [P] [US3] Add welcome message to chat panel in frontend/src/app/page.tsx
- [ ] T039 [US3] Test highlight → confirm → sendMessage functionality
- [ ] T040 [US3] Test dark theme functionality

## Phase 6: [US4] Docusaurus Integration and Deployment
### Goal: Integrate with Docusaurus and prepare for deployment

**Independent Test Criteria**: Chat functionality works when embedded in Docusaurus documentation

- [X] T041 [P] [US4] Update docusaurus.config.ts to add frontend script for development
- [X] T042 [P] [US4] Update docusaurus.config.ts to add frontend script for production
- [X] T043 [P] [US4] Test integration with local Docusaurus site
- [X] T044 [P] [US4] Prepare backend for Railway deployment
- [X] T045 [P] [US4] Prepare frontend for Vercel deployment
- [X] T046 [US4] Test complete integration: highlight text → open chat → get Gemini response
- [X] T047 [US4] Document deployment process

## Phase 7: Polish & Cross-Cutting Concerns
### Goal: Final testing, optimization, and deployment

- [ ] T048 Add error handling and logging throughout backend/main.py
- [ ] T049 Add loading states and error messages to frontend/src/app/page.js
- [ ] T050 Optimize performance for sub-4s response time
- [ ] T051 Run comprehensive integration tests
- [ ] T052 Deploy backend to Railway
- [ ] T053 Deploy frontend to Vercel
- [ ] T054 Deploy documentation to GitHub Pages
- [ ] T055 Final end-to-end testing