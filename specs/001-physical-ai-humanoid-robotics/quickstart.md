# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Feature**: 001-physical-ai-humanoid-robotics
**Date**: 2025-12-06
**Status**: Ready

## Overview

This guide provides the essential steps to set up, develop, and deploy the Physical AI & Humanoid Robotics textbook project. The project is a documentation site built with Docusaurus, containing 5 modules covering the complete humanoid robotics pipeline.

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- Text editor with Markdown support
- Basic understanding of ROS 2, Gazebo, NVIDIA Isaac, and VLA concepts

## Setup Instructions

### 1. Environment Setup

```bash
# Clone the repository
git clone [repository-url]
cd [repository-name]

# Install dependencies
npm install
# OR if using yarn
yarn install
```

### 2. Project Structure

After setup, your project will have this structure:

```
project-root/
├── docs/                 # Textbook content
│   ├── module1/          # The Robotic Nervous System (ROS 2)
│   ├── module2/          # The Digital Twin (Gazebo & Unity)
│   ├── module3/          # The AI-Robot Brain (NVIDIA Isaac)
│   ├── module4/          # Vision-Language-Action (VLA)
│   └── capstone/         # The Autonomous Humanoid
├── static/              # Static assets (images)
│   ├── img/
│   │   ├── module1/
│   │   ├── module2/
│   │   ├── module3/
│   │   ├── module4/
│   │   └── capstone/
├── src/                 # Docusaurus custom components
├── docusaurus.config.js # Docusaurus configuration
└── package.json         # Project dependencies
```

### 3. Local Development

```bash
# Start local development server
npm run start
# OR
yarn start

# The site will be available at http://localhost:3000
```

### 4. Content Creation

#### Adding a New Subchapter

1. Create a new MDX file in the appropriate module directory:
   ```
   docs/module1/new-subchapter.mdx
   ```

2. Follow the naming convention: `x-x-short-description.mdx`

3. Include proper frontmatter:
   ```md
   ---
   title: Your Subchapter Title
   sidebar_position: 2
   ---

   # Your Subchapter Title

   Your content here...
   ```

#### Adding Code Examples

- Use fenced code blocks with language specification
- Keep code blocks to ≤20 lines
- Use only allowed languages: python, bash, yaml, json

```python
# Example Python code
def example_function():
    """This is an example function."""
    return "Hello, ROS 2!"
```

#### Adding Images

1. Place images in the appropriate module directory under `/static/img/`
2. Reference using MDX syntax:
   ```mdx
   <img src="/img/module1/ros2-architecture.png" alt="ROS 2 Architecture" width="750" />
   ```
3. Limit to 2 images per subchapter

## Content Guidelines

### Writing Style

- Target audience: Students learning robotics
- Explain concepts clearly with practical examples
- Maintain consistency with existing content
- Follow the word count guidelines (1000-1500 words per module)

### Technical Requirements

- All content must be in MDX format
- Code examples ≤20 lines each
- Images must be SVG or PNG format
- Maximum 2 images per chapter
- All paths must be relative and correct

### Module Structure

Each module should follow this general structure:
1. Introduction to the topic
2. Core concepts and theory
3. Practical examples and code
4. Integration with other systems
5. Summary and next steps

## Building and Deployment

### Build for Production

```bash
# Build static files
npm run build
# OR
yarn build

# Files will be generated in the build/ directory
```

### Local Preview of Production Build

```bash
# Build and serve locally
npm run serve
# OR
yarn serve
```

## Validation Checklist

Before committing content, verify:

- [ ] Content follows the specification requirements
- [ ] Word count is within module limits (1000-1500 per module)
- [ ] Code blocks are ≤20 lines and use allowed languages
- [ ] Images follow naming and format requirements
- [ ] Maximum 2 images per subchapter
- [ ] All links are valid and working
- [ ] MDX syntax is correct
- [ ] Content follows the constitution guidelines

## Common Tasks

### Update Navigation

The sidebar navigation is configured in `docusaurus.config.js`. Modules and subchapters will automatically appear based on the file structure, but you can customize the order and titles in the configuration file.

### Add New Module

1. Create a new directory in `/docs/` (e.g., `docs/module5/`)
2. Add an `index.md` file as the module overview
3. Add subchapter files following the naming convention
4. Update the sidebar configuration if needed

### Testing Content

1. Use `npm run start` to preview changes in real-time
2. Verify all links work correctly
3. Check that images render properly
4. Validate code examples syntax
5. Confirm responsive design on different screen sizes

## Troubleshooting

### Content Not Appearing

- Verify the file is in the correct directory
- Check that the filename follows the convention
- Ensure frontmatter is properly formatted
- Confirm the file has a `.md` or `.mdx` extension

### Images Not Loading

- Verify the image is in the `/static/img/` directory
- Check that the path in the MDX file is correct
- Confirm the image format is SVG or PNG
- Ensure the image file name uses lowercase and hyphens

### Build Errors

- Check for syntax errors in MDX files
- Verify all links and image paths are correct
- Ensure code blocks follow the ≤20 line rule
- Validate frontmatter syntax

## Next Steps

1. Review the detailed tasks in `tasks.md` for specific implementation steps
2. Begin content creation following the module structure
3. Use the `Content Writer Agent` to generate content following constitution guidelines
4. Run quality checks with the `Quality Agent` before publishing