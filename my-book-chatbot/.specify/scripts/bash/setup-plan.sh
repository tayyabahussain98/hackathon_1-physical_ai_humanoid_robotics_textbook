#!/bin/bash

# Setup script for planning phase
# Outputs JSON with required paths

cat <<EOF
{
    "FEATURE_SPEC": "specs/1-gemini-rag-chatbot/spec.md",
    "IMPL_PLAN": "specs/1-gemini-rag-chatbot/plan.md",
    "SPECS_DIR": "specs/1-gemini-rag-chatbot",
    "BRANCH": "1-gemini-rag-chatbot"
}
EOF