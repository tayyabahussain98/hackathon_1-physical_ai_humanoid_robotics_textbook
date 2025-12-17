# Physical AI & Humanoid Robotics Textbook

Learn to design, simulate, and deploy humanoid robots using ROS 2, Gazebo, NVIDIA Isaac, and VLA techniques.

## Table of Contents
- [About](#about)
- [Modules](#modules)
- [Getting Started](#getting-started)
- [Development](#development)
- [Contributing](#contributing)

## About

This comprehensive textbook covers the complete pipeline for designing and deploying humanoid robots. It's organized into 5 modules:

1. **Module 1**: The Robotic Nervous System (ROS 2)
2. **Module 2**: The Digital Twin (Gazebo & Unity)
3. **Module 3**: The AI-Robot Brain (NVIDIA Isaac)
4. **Module 4**: Vision-Language-Action (VLA)
5. **Capstone**: The Autonomous Humanoid

## Modules

### Module 1: The Robotic Nervous System (ROS 2)
- Introduction to ROS 2
- ROS 2 Nodes, Topics, and Services
- Python Agents Bridging to ROS Controllers
- Understanding URDF for Humanoids

### Module 2: The Digital Twin (Gazebo & Unity)
- Physics Simulation in Gazebo
- High-Fidelity Rendering in Unity
- Sensor Simulation: LiDAR, Depth Cameras, IMUs

### Module 3: The AI-Robot Brain (NVIDIA Isaac)
- NVIDIA Isaac Sim: Photorealistic Simulation
- Isaac ROS: VSLAM and Navigation
- Nav2: Path Planning for Bipedal Humanoids

### Module 4: Vision-Language-Action (VLA)
- Voice-to-Action with OpenAI Whisper
- Cognitive Planning: Language to ROS 2 Actions
- Multi-modal Interaction: Speech, Vision, Gesture

### Capstone: The Autonomous Humanoid
- Project Overview
- Voice Command Processing
- Navigation & Obstacle Handling
- Object Recognition & Manipulation
- Final Integration & Demonstration

## Getting Started

1. **Prerequisites**:
   - Node.js (version 18 or higher)
   - npm or yarn package manager

2. **Installation**:
   ```bash
   # Clone the repository
   git clone [repository-url]
   cd [repository-name]

   # Install dependencies
   npm install
   ```

3. **Start Development Server**:
   ```bash
   npm start
   ```
   The site will be available at http://localhost:3000

## Development

### Content Creation Guidelines

- All content must be in MDX format
- Code examples must be ≤20 lines
- Images must be SVG or PNG format
- Maximum 2 images per chapter
- All paths must be relative and correct

### Adding a New Subchapter

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

### Adding Code Examples

- Use fenced code blocks with language specification
- Keep code blocks to ≤20 lines
- Use only allowed languages: python, bash, yaml, json

### Adding Images

1. Place images in the appropriate module directory under `/static/img/`
2. Reference using MDX syntax:
   ```mdx
   <img src="/img/module1/ros2-architecture.png" alt="ROS 2 Architecture" width="750" />
   ```
3. Limit to 2 images per subchapter

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the terms specified in the LICENSE file.
