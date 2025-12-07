# Data Model: Physical AI & Humanoid Robotics Textbook

**Feature**: 001-physical-ai-humanoid-robotics
**Date**: 2025-12-06
**Status**: Design

## Content Entities

### 1. Module
**Description**: A major section of the textbook covering a specific aspect of humanoid robotics

**Fields**:
- `id`: Unique identifier (e.g., "module1", "module2", "capstone")
- `title`: Display title of the module
- `description`: Brief description of the module's content
- `subchapters`: Array of subchapter entities
- `word_count`: Target word count (1000-1500)
- `content_types`: Array of allowed content types (text, code, image, diagram, table)

**Relationships**:
- Contains 3-5 Subchapter entities
- Referenced by Navigation system

**Validation Rules**:
- Title must be 5-100 characters
- Word count must be between 1000-1500
- Must contain at least 1 subchapter

**State Transitions**:
- Draft → In Review → Published

### 2. Subchapter
**Description**: A subsection within a module that focuses on a specific topic

**Fields**:
- `id`: Unique identifier (e.g., "1.1", "2.3", "5.4")
- `module_id`: Reference to parent module
- `title`: Display title of the subchapter
- `content`: Main content text
- `code_examples`: Array of code block entities
- `diagrams`: Array of diagram/image entities
- `word_count`: Actual word count
- `content_types`: Array of content types used

**Relationships**:
- Belongs to 1 Module entity
- Contains 0+ CodeBlock entities
- Contains 0+ Image entities

**Validation Rules**:
- Title must be 5-100 characters
- Word count must not exceed module limits
- Must follow content type restrictions per constitution

**State Transitions**:
- Draft → In Review → Published

### 3. CodeBlock
**Description**: A code snippet included in textbook content

**Fields**:
- `id`: Unique identifier within parent subchapter
- `subchapter_id`: Reference to parent subchapter
- `language`: Programming language (python, bash, yaml, json)
- `code`: The actual code content
- `line_count`: Number of lines in the block
- `description`: Brief explanation of the code's purpose
- `caption`: Optional caption for the code block

**Relationships**:
- Belongs to 1 Subchapter entity

**Validation Rules**:
- Line count must be ≤20 lines
- Language must be in allowed list (python, bash, yaml, json)
- Code must be syntactically correct for the language

**State Transitions**:
- Draft → Validated → Published

### 4. Image
**Description**: An image or diagram included in textbook content

**Fields**:
- `id`: Unique identifier within parent subchapter
- `subchapter_id`: Reference to parent subchapter
- `filename`: Name of the image file
- `alt_text`: Alternative text for accessibility
- `caption`: Optional caption for the image
- `type`: Type of image (diagram, screenshot, chart, photo)
- `path`: Relative path to the image file

**Relationships**:
- Belongs to 1 Subchapter entity

**Validation Rules**:
- File must be SVG or PNG format
- Alt text is required for accessibility
- Max 2 images per subchapter
- Path must follow `/static/img/moduleX/` pattern

**State Transitions**:
- Draft → Processed → Published

### 5. NavigationItem
**Description**: An item in the textbook's navigation system

**Fields**:
- `id`: Unique identifier
- `title`: Display title in navigation
- `path`: Relative path to the content
- `parent_id`: Reference to parent navigation item (null for top-level)
- `order`: Display order in navigation
- `type`: Type of item (module, subchapter, capstone)

**Relationships**:
- Can have 0+ child NavigationItem entities
- References 1 Module or Subchapter entity

**Validation Rules**:
- Title must match corresponding Module/Subchapter title
- Path must be valid and point to existing content
- Order must be unique among siblings

**State Transitions**:
- Draft → Published

## Content Flow

### Creation Flow
1. Module entity is created with basic information
2. Subchapter entities are added to the module
3. Content is added to subchapters (text, code blocks, images)
4. Navigation items are created to represent the structure
5. Content is validated against constitution requirements
6. Content is published to the documentation site

### Validation Flow
1. Content is checked against word count requirements
2. Code blocks are validated for line count and language
3. Images are validated for format and quantity
4. Content types are verified against allowed list
5. File paths are validated for correctness
6. Navigation structure is validated for completeness

## Constraints and Limitations

### Content Constraints
- Total book word count: 3,000-6,000 words
- Module word count: 1,000-1,500 words
- Code block limit: ≤20 lines per block
- Image limit: ≤2 per subchapter
- Allowed code languages: python, bash, yaml, json

### Structural Constraints
- Maximum 5 modules (4 core + 1 capstone)
- Maximum 3-5 subchapters per module
- File naming: lowercase with hyphens
- Directory structure: `/docs/moduleX/`
- Image storage: `/static/img/moduleX/`

### Format Constraints
- Content format: MDX for Docusaurus compatibility
- Image formats: SVG or PNG only
- Code format: Fenced markdown blocks
- Navigation: Docusaurus sidebar format