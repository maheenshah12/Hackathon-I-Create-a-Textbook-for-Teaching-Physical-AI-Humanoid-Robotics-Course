// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  curriculumSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'ROS2 Foundations',
      items: [
        'modules/ros2-foundations/intro',
        'modules/ros2-foundations/chapter-1-intro-ros2',
        'modules/ros2-foundations/chapter-2-ros2-nodes',
        'modules/ros2-foundations/chapter-3-ros2-actions',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Simulation',
      items: [
        'modules/digital-twin-simulation/intro',
        'modules/digital-twin-simulation/chapter-1-gazebo-basics/index',
        'modules/digital-twin-simulation/chapter-2-unity-digital-twin/index',
        'modules/digital-twin-simulation/chapter-3-advanced-sim/index',
      ],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac Perception',
      items: [
        'modules/nvidia-isaac-perception/intro',
        'modules/nvidia-isaac-perception/chapter-1-isaac-basics',
        'modules/nvidia-isaac-perception/chapter-2-vslam',
        'modules/nvidia-isaac-perception/chapter-3-manipulation',
      ],
    },
    {
      type: 'category',
      label: 'VLA Pipelines',
      items: [
        'modules/vla-pipelines/intro',
        'modules/vla-pipelines/chapter-1-whisper-integration/index',
        'modules/vla-pipelines/chapter-2-gpt-ros-bridge/index',
        'modules/vla-pipelines/chapter-3-vla-workflows/index',
      ],
    },
  ],
};

module.exports = sidebars;