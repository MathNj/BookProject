module.exports = {
  mainSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Nervous System (ROS 2)',
      items: [
        'nervous-system/nervous-system-overview',
        'nervous-system/intro-to-ros-2',
        'nervous-system/installation',
        'nervous-system/nodes-and-topics',
        'nervous-system/urdf-modeling',
        'nervous-system/STUDENT-CHECKLIST',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Simulation)',
      items: [
        'digital-twin-sim/getting-started',
        'digital-twin-sim/intro-digital-twin',
        'digital-twin-sim/gazebo-fortress-setup',
        'digital-twin-sim/simulating-sensors',
        'digital-twin-sim/unity-visualization',
        'digital-twin-sim/STUDENT-CHECKLIST',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'robot-brain/isaac-sim-bridge',
        'robot-brain/synthetic-data-generation',
        'robot-brain/visual-slam',
        'robot-brain/nav2-integration',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: The Mind (Vision Language Models)',
      items: [
        'the-mind/the-mind-overview',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 5: Capstone Project',
      items: [
        'capstone/capstone-overview',
      ],
      collapsed: false,
    },
  ],
};
