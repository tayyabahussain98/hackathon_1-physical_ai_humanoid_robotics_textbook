---
id: 1
title: Physical AI Humanoid Robotics Constitution Update
stage: constitution
date_iso: "2025-12-06"
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels:
  - constitution
  - update
  - physical-ai
  - humanoid-robotics
files_yaml:
  - ".specify/memory/constitution.md"
  - "history/prompts/constitution/1-physical-ai-humanoid-robotics.constitution.prompt.md"
tests_yaml: []
links:
  spec: null
  ticket: null
  adr: null
  pr: null
---

## Outcome
Updated the project constitution to version 1.2.0 to include detailed guidelines for images, code blocks, and formatting standards for the Physical AI & Humanoid Robotics textbook project.

## Evaluation
The constitution was successfully updated to include:
- New content type allowances (images, diagrams, code blocks)
- Detailed formatting standards for MDX compatibility
- File structure requirements
- Code and image rules
- Constraints for content, Docusaurus, and Spec-Kit Plus
- Success criteria for valid chapters
- Governance and enforcement rules

## Prompt Text
```
Project: Physical AI & Humanoid Robotics — Spec-Driven Textbook

Applies to: Docusaurus + Spec-Kit Plus + Claude Code Agents



---



## I. Core Principles



### 1. Spec-Driven Writing

All chapters MUST follow the exact structure defined in `/sp.specify`.

No content may be generated outside approved modules, chapters, or sections.



### 2. Consistency

All chapters must follow:

- same heading hierarchy

- same formatting style

- same section ordering

- same filename patterns

- same code + image formatting rules



### 3. Accuracy

All explanations must be:

- factually correct

- aligned with robotics, simulation, AI, ROS 2, Isaac, VLA

- free from hallucinations

- written for students learning robotics



### 4. Scope Discipline

This book is limited to:

- 4 Modules

- 1 Capstone

- No extra chapters

- No new modules

- No random content outside scope



### 5. Agent-Assisted Workflow

All work MUST pass through all agents in this order:



1. **Orchestrator Agent** — determines task

2. **Book Structure Agent** — layout + file placement

3. **Content Writer Agent** — writes content

4. **Quality Agent** — checks formatting, clarity, structure



No agent may skip another.



---



## II. Allowed Content Types (Important Update)



This book **allows visuals and practical examples** because it is an educational textbook.



### ✔ Allowed

- **Images (diagrams, charts, architecture, flow maps)**

- **Code blocks (short, ≤ 20 lines)**

- **Robot diagrams**

- **Digital twin diagrams**

- **ROS 2 nodes → topics diagrams**

- **Isaac pipeline visuals**

- **VLA block diagrams**

- **URDF and Python examples**



### ❌ Not Allowed

- Full apps or long programs

- Marketing content

- Opinions

- Ethical analysis

- Vendor comparisons

- Research papers

- Random personal advice



---



## III. Standards (Formatting + Structure)



### 1. Markdown Format (MDX Ready for Docusaurus)

- All chapters must use `.md` or `.mdx`

- All images must use MDX syntax

  ```mdx

  <img src="/img/module1/diagram.png" alt="ROS diagram" width="750" />

2. File Structure Must Follow:

bash

Copy code

/docs/module1/

/docs/module2/

/docs/module3/

/docs/module4/

/docs/capstone/

No extra folders allowed.



3. Code Rules

Max 20 lines per code block



Allowed languages:



python



bash



yaml



json



Code must stay conceptual, not production-level



No fully runnable applications



4. Image Rules

Max 2 images per chapter



Must be stored in /static/img/moduleX/



Must be high clarity



Only diagrams, architecture charts, step flows allowed



No personal photos



No copyrighted images



IV. Constraints

Content Constraints

Word count: 3,000–6,000 words full book



No unnecessary depth beyond module requirements



No full tutorials



No nested folder structures beyond module folders



Docusaurus Constraints

Sidebar MUST recognize all files



No file may break MDX compilation



All filenames must be lowercase + hyphenated



Spec-Kit Plus Constraints

Structure MUST follow /sp.specify



Agents must comply with JSON schema



No drift allowed



V. Success Criteria (What counts as a "correct chapter")

A chapter is VALID only if:



Follows the section structure in /sp.specify



Uses headings correctly (H1, H2, H3)



Contains only allowed images



Contains only allowed code



Passes all Quality Agent checks



Renders correctly in Docusaurus



Sidebar displays chapter correctly



File naming follows pattern



Image paths are correct



VI. Governance

1. Constitution Overrides All

No model or agent may improvise or generate extra content outside this constitution.



2. Enforcement Rules

All generated outputs MUST follow /sp.constitution + /sp.specify



No exceptions allowed



Planning → Writing → Formatting → Quality is mandatory flow



If any agent violates formatting, output must be regenerated



3. Versioning

Current version: v1.2.0



Any change must be approved by Orchestrator Agent



Change → Amendment → Version bump required



VII. Amendment Log

v1.2.0: Updated constitution to allow images, diagrams, code, MDX syntax



Added formatting rules for images



Added code rules



Added visual diagram guidelines
```

## Response Text
Updated the project constitution file (.specify/memory/constitution.md) to version 1.2.0, incorporating all the specified principles, content type allowances, formatting standards, constraints, and governance rules. The new constitution now includes detailed sections on allowed content types (images, diagrams, code blocks), formatting requirements for MDX compatibility, file structure, code and image rules, various constraints, success criteria for chapters, and governance procedures.