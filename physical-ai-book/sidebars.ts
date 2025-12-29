import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar structure for the Physical AI & Humanoid Robotics documentation
  docsSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1/ros2-fundamentals',
        'module-1/python-agents-robot-control',
        'module-1/urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twins & Physical Simulation',
      items: [
        'module-2/physics-simulation-gazebo',
        'module-2/sensor-simulation',
        'module-2/human-robot-interaction-unity',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI–Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3/photorealistic-simulation-synthetic-data',
        'module-3/perception-localization',
        'module-3/navigation-motion-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision–Language–Action (VLA) Systems',
      items: [
        'module-4/voice-to-intent',
        'module-4/cognitive-planning-llms',
        'module-4/capstone-autonomous-humanoid',
      ],
    },
  ],
};

export default sidebars;
