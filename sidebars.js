/**
 * Creating a sidebar enables you to:
 - Create an ordered group of docs
 - Render a sidebar for each doc of that group
 - By default, Docusaurus uses a sidebar from the docs folder.
 - You can change this to use a different sidebar

Your sidebar can be a:
- Array of filenames (the order matters)
- Array of objects with 'label' and 'items' keys

For more information, see: https://docusaurus.io/docs/sidebar#sidebar-objects

Important:
If you're using a single project, `docs` and `static` at repository root, you will usually use `sidebar.js` from `docs` folder.
If you're using a web app, `backend/src`, `frontend/src`, `sidebar.js` can be separate for each part.

*/


const sidebars = {
  // By default, Docusaurus uses a sidebar from the docs folder.
  // You can change this to use a different sidebar
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System (ROS 2)',
      items: [
        {
          type: 'doc',
          id: 'module1/chapter1',
          label: 'Chapter 1: The Robotic Nervous System (ROS 2) - Introduction and Core Concepts',
        },
        {
          type: 'doc',
          id: 'module1/chapter2',
          label: 'Chapter 2: Understanding URDF and TF for Humanoids',
        },
        {
          type: 'doc',
          id: 'module1/chapter3',
          label: 'Chapter 3: Joint Control Examples',
        },
        'module1/summary',
        'module1/quiz',
        'module1/mini-project',
        'module1/practical-demos'
      ],
    },
    {
      type: 'category',
      label: 'Module 2 - Digital Twin Simulation (Gazebo + Unity)',
      items: [
        {
          type: 'doc',
          id: 'module2/chapter1',
          label: 'Chapter 1: Humanoid URDF in Gazebo',
        },
        {
          type: 'doc',
          id: 'module2/chapter2',
          label: 'Chapter 2: Physics and Sensors',
        },
        {
          type: 'doc',
          id: 'module2/chapter3',
          label: 'Chapter 3: ROS-Gazebo Bridge',
        },
        'module2/summary',
        'module2/quiz',
        'module2/mini-project',
        'module2/practical-demos'
      ],
    },
    {
      type: 'category',
      label: 'Module 3 - AI-Robot Brain',
      items: [
        {
          type: 'doc',
          id: 'module3/chapter1',
          label: 'Chapter 1: Perception, Mapping, and Navigation',
        },
        {
          type: 'doc',
          id: 'module3/chapter2',
          label: 'Chapter 2: Visual SLAM (Simultaneous Localization and Mapping)',
        },
        {
          type: 'doc',
          id: 'module3/chapter3',
          label: 'Chapter 3: Nav2 Humanoid Control',
        },
        {
          type: 'doc',
          id: 'module3/chapter4',
          label: 'Chapter 4: Synthetic Data Generation',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action Robotics',
      items: [
        {
          type: 'doc',
          id: 'module4/chapter1',
          label: 'Chapter 1: Understanding Vision-Language-Action Models',
        },
        {
          type: 'doc',
          id: 'module4/chapter2',
          label: 'Chapter 2: Natural Language Processing for Robotics',
        },
        {
          type: 'doc',
          id: 'module4/chapter3',
          label: 'Chapter 3: Action Planning and Execution',
        },
        {
          type: 'doc',
          id: 'module4/chapter4',
          label: 'Chapter 4: Human-Robot Interaction and Communication',
        },
        'module4/summary',
        'module4/quiz',
        'module4/mini-project',
        'module4/practical-demos'
      ],
    },
  ],
};

module.exports = sidebars;
