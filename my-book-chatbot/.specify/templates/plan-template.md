# Architecture Plan Template

## Feature: [FEATURE_NAME]

### Architecture Decision Record (ADR)

#### 1. Scope and Dependencies
- **In Scope**: [IN_SCOPE_BOUNDARIES]
- **Out of Scope**: [OUT_OF_SCOPE_ITEMS]
- **External Dependencies**: [EXTERNAL_DEPENDENCIES]

#### 2. Key Decisions and Rationale
- **Options Considered**: [OPTIONS_CONSIDERED]
- **Trade-offs**: [TRADE_OFFS]
- **Rationale**: [RATIONALE]
- **Principles**: [PRINCIPLES]

#### 3. Interfaces and API Contracts
- **Public APIs**: [API_INPUTS_OUTPUTS]
- **Versioning Strategy**: [VERSIONING_STRATEGY]
- **Idempotency, Timeouts, Retries**: [RETRY_STRATEGY]
- **Error Taxonomy**: [ERROR_CODES]

#### 4. Non-Functional Requirements (NFRs) and Budgets
- **Performance**: [PERFORMANCE_REQUIREMENTS]
- **Reliability**: [RELIABILITY_REQUIREMENTS]
- **Security**: [SECURITY_REQUIREMENTS]
- **Cost**: [COST_REQUIREMENTS]

#### 5. Data Management and Migration
- **Source of Truth**: [DATA_SOURCE]
- **Schema Evolution**: [SCHEMA_EVOLUTION]
- **Migration and Rollback**: [MIGRATION_STRATEGY]
- **Data Retention**: [DATA_RETENTION]

#### 6. Operational Readiness
- **Observability**: [OBSERVABILITY_STRATEGY]
- **Alerting**: [ALERTING_STRATEGY]
- **Runbooks**: [RUNBOOKS]
- **Deployment and Rollback**: [DEPLOYMENT_STRATEGY]
- **Feature Flags**: [FEATURE_FLAGS]

#### 7. Risk Analysis and Mitigation
- **Top 3 Risks**: [RISK_ANALYSIS]
- **Blast Radius**: [BLAST_RADIUS]
- **Kill Switches/Guardrails**: [GUARDRAILS]

#### 8. Evaluation and Validation
- **Definition of Done**: [DONE_CRITERIA]
- **Output Validation**: [VALIDATION_STRATEGY]

#### 9. Constitution Check
- All architecture decisions comply with project constitution v3.0
- Single file backend architecture (backend/main.py only)
- Single file frontend architecture (frontend/src/app/page.js only)
- Gemini via LiteLLM only
- Official OpenAI ChatKit React (@openai/chatkit-react) only
- uv project management only
- Selected-text mode functionality included
- All code via Claude only