<!-- SYNC IMPACT REPORT -->
<!-- Version change: N/A → v3.0 -->
<!-- Added sections: All principles and governance sections -->
<!-- Templates requiring updates: N/A (first version) -->
<!-- -->

# Project Constitution v3.0

## Project: Gemini RAG Chatbot in Next.js + Python FastAPI (Single File Each)

## Principles

### Principle 1: Single File Backend Architecture
Backend code MUST be contained in ONLY backend/main.py (uv project). No additional backend files are permitted without explicit architectural review.

**Rationale**: Maintains simplicity and reduces complexity in the project structure.

### Principle 2: Single File Frontend Architecture
Frontend code MUST be contained in ONLY frontend/src/app/page.js (Next.js App Router). No additional frontend files are permitted without explicit architectural review.

**Rationale**: Maintains simplicity and reduces complexity in the project structure.

### Principle 3: Gemini LLM Integration
The project MUST use Gemini via LiteLLM as the exclusive LLM provider. No other LLM providers are permitted.

**Rationale**: Ensures consistency in AI capabilities and maintains compatibility with the chosen ecosystem.

### Principle 4: Official Chat UI Framework
The project MUST use the official OpenAI ChatKit React (@openai/chatkit-react) for the chat interface. No alternative chat UI frameworks are permitted.

**Rationale**: Ensures professional-grade chat interface with established patterns and components.

### Principle 5: Modern Python Project Management
The project MUST use uv (modern Python) for project initialization and dependency management. No other Python package managers are permitted.

**Rationale**: Ensures modern, fast, and efficient Python project management.

### Principle 6: Selected-Text Mode Support
The application MUST support selected-text mode functionality. This feature is required and cannot be omitted.

**Rationale**: Enables core functionality for text selection and interaction with the AI model.

### Principle 7: Claude-Only Code Generation
All code MUST be generated via Claude only. Human-written code is not permitted without explicit exception.

**Rationale**: Ensures consistency in code quality and style, and maintains the project's AI-assisted development approach.

### Principle 8: Constitution Exclusivity
This constitution is the ONLY constitution in the universe for this project. No other constitutions, guidelines, or rules apply.

**Rationale**: Ensures absolute clarity and consistency in project governance.

## Governance

### Ratification
This constitution was ratified on [RATIFICATION_DATE].

### Amendment Procedure
Amendments to this constitution require unanimous consent from all project stakeholders and explicit approval from the project architect.

### Versioning Policy
This is version 3.0 of the constitution. All changes to this constitution are governed by the amendment procedure above.

### Compliance Review
Regular compliance reviews MUST be conducted to ensure adherence to all principles in this constitution.

## Effective Date
This constitution is effective immediately upon creation.