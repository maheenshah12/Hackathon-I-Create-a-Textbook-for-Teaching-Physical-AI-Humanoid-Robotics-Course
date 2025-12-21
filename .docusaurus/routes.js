import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '684'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', 'de5'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '995'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'e82'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '414'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '945'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '115'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'c67'),
    routes: [
      {
        path: '/docs/intro',
        component: ComponentCreator('/docs/intro', '3d3'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/digital-twin-simulation/chapter-1-gazebo-basics/',
        component: ComponentCreator('/docs/modules/digital-twin-simulation/chapter-1-gazebo-basics/', 'd7e'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/digital-twin-simulation/chapter-2-unity-digital-twin/',
        component: ComponentCreator('/docs/modules/digital-twin-simulation/chapter-2-unity-digital-twin/', '083'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/digital-twin-simulation/chapter-3-advanced-sim/',
        component: ComponentCreator('/docs/modules/digital-twin-simulation/chapter-3-advanced-sim/', '204'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/digital-twin-simulation/intro',
        component: ComponentCreator('/docs/modules/digital-twin-simulation/intro', 'a30'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/digital-twin-simulation/simulation-troubleshooting',
        component: ComponentCreator('/docs/modules/digital-twin-simulation/simulation-troubleshooting', '74f'),
        exact: true
      },
      {
        path: '/docs/modules/digital-twin-simulation/simulation-validation-notes',
        component: ComponentCreator('/docs/modules/digital-twin-simulation/simulation-validation-notes', '5ff'),
        exact: true
      },
      {
        path: '/docs/modules/digital-twin-simulation/simulation-workflow-tutorial',
        component: ComponentCreator('/docs/modules/digital-twin-simulation/simulation-workflow-tutorial', '3d6'),
        exact: true
      },
      {
        path: '/docs/modules/nvidia-isaac-perception/chapter-1-isaac-basics',
        component: ComponentCreator('/docs/modules/nvidia-isaac-perception/chapter-1-isaac-basics', '44f'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/nvidia-isaac-perception/chapter-2-vslam',
        component: ComponentCreator('/docs/modules/nvidia-isaac-perception/chapter-2-vslam', '6a9'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/nvidia-isaac-perception/chapter-3-manipulation',
        component: ComponentCreator('/docs/modules/nvidia-isaac-perception/chapter-3-manipulation', 'bc1'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/nvidia-isaac-perception/intro',
        component: ComponentCreator('/docs/modules/nvidia-isaac-perception/intro', 'c65'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/nvidia-isaac-perception/isaac-sim-tutorial-steps',
        component: ComponentCreator('/docs/modules/nvidia-isaac-perception/isaac-sim-tutorial-steps', '6e3'),
        exact: true
      },
      {
        path: '/docs/modules/nvidia-isaac-perception/isaac-troubleshooting',
        component: ComponentCreator('/docs/modules/nvidia-isaac-perception/isaac-troubleshooting', '639'),
        exact: true
      },
      {
        path: '/docs/modules/nvidia-isaac-perception/isaac-validation-notes',
        component: ComponentCreator('/docs/modules/nvidia-isaac-perception/isaac-validation-notes', '337'),
        exact: true
      },
      {
        path: '/docs/modules/ros2-foundations/basic-ros2-tutorial-steps',
        component: ComponentCreator('/docs/modules/ros2-foundations/basic-ros2-tutorial-steps', '5b8'),
        exact: true
      },
      {
        path: '/docs/modules/ros2-foundations/chapter-1-intro-ros2',
        component: ComponentCreator('/docs/modules/ros2-foundations/chapter-1-intro-ros2', '862'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/ros2-foundations/chapter-2-ros2-nodes',
        component: ComponentCreator('/docs/modules/ros2-foundations/chapter-2-ros2-nodes', '96c'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/ros2-foundations/chapter-3-ros2-actions',
        component: ComponentCreator('/docs/modules/ros2-foundations/chapter-3-ros2-actions', '990'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/ros2-foundations/intro',
        component: ComponentCreator('/docs/modules/ros2-foundations/intro', '1d0'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/ros2-foundations/ros2-troubleshooting',
        component: ComponentCreator('/docs/modules/ros2-foundations/ros2-troubleshooting', '1f5'),
        exact: true
      },
      {
        path: '/docs/modules/ros2-foundations/validation-notes',
        component: ComponentCreator('/docs/modules/ros2-foundations/validation-notes', '4dd'),
        exact: true
      },
      {
        path: '/docs/modules/vla-pipelines/chapter-1-whisper-integration/',
        component: ComponentCreator('/docs/modules/vla-pipelines/chapter-1-whisper-integration/', 'adb'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/vla-pipelines/chapter-2-gpt-ros-bridge/',
        component: ComponentCreator('/docs/modules/vla-pipelines/chapter-2-gpt-ros-bridge/', '53c'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/vla-pipelines/chapter-3-vla-workflows/',
        component: ComponentCreator('/docs/modules/vla-pipelines/chapter-3-vla-workflows/', '157'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/vla-pipelines/intro',
        component: ComponentCreator('/docs/modules/vla-pipelines/intro', '301'),
        exact: true,
        sidebar: "curriculumSidebar"
      },
      {
        path: '/docs/modules/vla-pipelines/vla-pipeline-tutorial',
        component: ComponentCreator('/docs/modules/vla-pipelines/vla-pipeline-tutorial', '2a1'),
        exact: true
      },
      {
        path: '/docs/modules/vla-pipelines/vla-troubleshooting',
        component: ComponentCreator('/docs/modules/vla-pipelines/vla-troubleshooting', '437'),
        exact: true
      },
      {
        path: '/docs/modules/vla-pipelines/vla-validation-notes',
        component: ComponentCreator('/docs/modules/vla-pipelines/vla-validation-notes', 'b79'),
        exact: true
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '487'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
