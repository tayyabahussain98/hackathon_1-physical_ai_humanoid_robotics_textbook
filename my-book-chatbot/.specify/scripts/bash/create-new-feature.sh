#!/bin/bash

# Parse arguments
FEATURE_DESCRIPTION=""
NUMBER=""
SHORT_NAME=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -Number|--number)
            NUMBER="$2"
            shift 2
            ;;
        -ShortName|--short-name)
            SHORT_NAME="$2"
            shift 2
            ;;
        *)
            FEATURE_DESCRIPTION="$1"
            shift
            ;;
    esac
done

# Create feature branch name and directory
BRANCH_NAME="${NUMBER}-${SHORT_NAME}"
FEATURE_DIR="specs/${BRANCH_NAME}"

# Create the feature directory
mkdir -p "$FEATURE_DIR"

# Output JSON with branch name and spec file path
cat <<EOF
{
    "BRANCH_NAME": "$BRANCH_NAME",
    "SPEC_FILE": "${FEATURE_DIR}/spec.md"
}
EOF