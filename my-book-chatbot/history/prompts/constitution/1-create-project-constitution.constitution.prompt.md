---
id: 1
title: "Create Project Constitution"
stage: constitution
date_iso: "2025-12-10"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "main"
user: "user"
command: "/sp.constitution"
labels:
  - "constitution"
  - "governance"
  - "setup"
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
  - ".specify/templates/spec-template.md"
  - ".specify/templates/plan-template.md"
  - ".specify/templates/tasks-template.md"
tests: []
prompt_text: |
  # FINAL CONSTITUTION v3.0 – LOCKED FOREVER

  Project: Gemini RAG Chatbot in Next.js + Python FastAPI (Single File Each)

  RULES (NO CHANGES EVER)

  • Backend  = ONLY backend/main.py (uv project)

  • Frontend = ONLY frontend/src/app/page.js (Next.js App Router)

  • LLM      = Gemini via LiteLLM only

  • Chat UI = Official OpenAI ChatKit React (@openai/chatkit-react)

  • Project init = uv (modern Python)

  • Selected-text mode = must work

  • All code via Claude only

  • This is the ONLY constitution in the universe
response_text: |
  Created the project constitution file with all specified principles and governance structure.
  Also created the required template files for spec, plan, and tasks.
outcome: "Constitution file created successfully with all specified principles and governance structure. Template files created to support the SpecKit Plus workflow."
evaluation: "Success - All required files created and constitution principles properly documented."
---

# Prompt History Record: Create Project Constitution

## Summary
Created the project constitution with all specified principles and governance structure.

## Details
This PHR documents the creation of the project constitution for the Gemini RAG Chatbot project, establishing the foundational rules and principles that will govern all future development.