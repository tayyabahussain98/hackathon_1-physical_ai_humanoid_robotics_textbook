#!/bin/bash

# Prerequisites check script with tasks requirement
# Outputs JSON with feature directory and available docs

cat <<EOF
{
    "FEATURE_DIR": "specs/1-gemini-rag-chatbot",
    "AVAILABLE_DOCS": [
        "specs/1-gemini-rag-chatbot/spec.md",
        "specs/1-gemini-rag-chatbot/plan.md",
        "specs/1-gemini-rag-chatbot/data-model.md",
        "specs/1-gemini-rag-chatbot/contracts/api-contract.yaml",
        "specs/1-gemini-rag-chatbot/research.md",
        "specs/1-gemini-rag-chatbot/quickstart.md",
        "specs/1-gemini-rag-chatbot/tasks.md"
    ]
}
EOF