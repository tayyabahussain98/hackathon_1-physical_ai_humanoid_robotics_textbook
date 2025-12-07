# Project Constitution — Spec-Driven Book System (v2.0.0)

Applies to: Docusaurus • Spec-Kit Plus • Claude Code Agents

Version: 2.0.0

Updated: 2025-12-06

Maintained by: Master Book Agent

---

## I. Core Principles

### 1. Spec-Driven Writing
All book content MUST strictly follow the structure defined in `/sp.specify`.
No chapter may be created outside the approved modules, sections, and hierarchy.

### 2. Consistency
All chapters must maintain:
- Same heading levels
- Same formatting rules
- Same file naming pattern
- Same MDX standards

### 3. Accuracy
All content must be:
- Factually correct
- Aligned with robotics, humanoids, AI, ROS2, Isaac, VLA
- Beginner-friendly but technically accurate

### 4. Scope Discipline
Only 4 modules + 1 capstone allowed.
No new modules, no random topics, no "extra explanation" outside the spec.

### 5. Agent-Assisted Workflow (NEW)
The entire workflow is handled by the following system:

1. **Master Book Agent (Orchestrator)**
2. **Content Writer (invoked internally)**
3. **Proofreading Skill (validation)**
4. **UI-Design Skill (layout & MDX validation)**

No external agents required.
No agent may skip another step.

---

## II. Allowed Content Types

### ✔ Allowed
- Images (diagrams, flowcharts, architecture)
- Code blocks (≤20 lines)
- Short Python, YAML, JSON, Bash examples
- ROS2 node diagrams
- Isaac/Simulation visuals
- URDF examples
- Block diagrams

### ❌ Not Allowed
- Long programs
- Full applications
- Tutorials beyond module scope
- Marketing or personal opinions
- More than 2 images per chapter

---

## III. Formatting & MDX Standards

### 1. MDX Layout Rules
- Chapters must be `.md` or `.mdx`
- Use valid React components only
- Code blocks must be fenced with language tag

### 2. Image Rules
Use MDX syntax:

```mdx
<img src="/img/module1/diagram.png" alt="ROS graph" width="750" />
```

- Max 2 images per chapter
- Images stored in `/static/img/moduleX/`

### 3. File Structure Rules
```
/docs/module1/
/docs/module2/
/docs/module3/
/docs/module4/
/docs/capstone/
```

No nested folders.

### 4. Code Block Standards
- Max 20 lines
- Must be conceptual
- No production-level code

---

## IV. Quality Validation (Handled by Skills)

### A. Proofreading Skill Validates:
- Grammar + clarity
- Style consistency
- Frontmatter
- Spec alignment
- Heading hierarchy

### B. UI-Design Skill Validates:
- MDX layout
- Sidebar compatibility
- Image placement
- Responsiveness
- Accessibility

Only content passing BOTH skills is accepted.

---

## V. Success Criteria for a Valid Chapter

A chapter is VALID only if:

✔ Follows `/sp.specify`
✔ Uses correct MDX + headings
✔ Has ≤2 images with correct paths
✔ Has ≤20-line code blocks
✔ Passes Proofreading Skill
✔ Passes UI-Design Skill
✔ Appears correctly in sidebar
✔ Does not break Docusaurus
✔ File naming is lowercase-hyphenated

---

## VI. Governance Rules

### 1. Constitution Overrides All
No model or agent may generate content outside this constitution.

### 2. Enforcement
- All outputs must comply with `/sp.constitude` + `/sp.specify`
- If formatting rule breaks → regenerate
- If validation fails → regenerate

### 3. Versioning
Every update must be approved by Master Book Agent.
Current version: **2.0.0**

---

## VII. Amendment Log

**v2.0.0 (2025-12-06)**
- Removed old agents
- Installed Master Book Agent system
- Linked Proofreading + UI-Design skills
- Simplified workflow
- Updated MDX, images, folder rules
- Updated quality validation flow