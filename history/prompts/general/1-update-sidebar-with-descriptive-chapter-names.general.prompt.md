---
id: 1
title: Update Sidebar with Descriptive Chapter Names
stage: general
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-robotics-book-spec
user: Claude
command: general
labels: ["sidebar", "navigation", "documentation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "sidebars.js"
  - "docusaurus.config.js"
tests: []
---

# Update Sidebar with Descriptive Chapter Names

## Context
Updated the Docusaurus sidebar to include descriptive names for all chapters across all modules, making the navigation more user-friendly and informative. The user requested that modules 1 and 2 should have chapter names with proper descriptions similar to how module 3 was organized.

## Changes Made

### 1. Updated sidebars.js
- Added descriptive labels for all chapters in modules 1 and 2
- Applied consistent formatting to modules 3 and 4 as well
- Used the format: `{ type: 'doc', id: 'moduleX/chapterY', label: 'Chapter Y: Descriptive Title' }`
- Maintained existing summary, quiz, mini-project, and practical-demos links

### 2. Fixed broken links
- Removed blog references from docusaurus.config.js that were causing build failures
- Removed the blog configuration from the presets
- Removed blog links from navbar and footer

## Files Modified

### sidebars.js
- Added descriptive chapter names for modules 1 and 2:
  - Module 1: "The Robotic Nervous System (ROS 2)"
    - Chapter 1: "The Robotic Nervous System (ROS 2) - Introduction and Core Concepts"
    - Chapter 2: "Understanding URDF and TF for Humanoids"
    - Chapter 3: "Joint Control Examples"
  - Module 2: "Digital Twin Simulation (Gazebo + Unity)"
    - Chapter 1: "Humanoid URDF in Gazebo"
    - Chapter 2: "Physics and Sensors"
    - Chapter 3: "ROS-Gazebo Bridge"
  - Module 3: "AI-Robot Brain"
    - Chapter 1: "Perception, Mapping, and Navigation"
    - Chapter 2: "Visual SLAM (Simultaneous Localization and Mapping)"
    - Chapter 3: "Nav2 Humanoid Control"
    - Chapter 4: "Synthetic Data Generation"
  - Module 4: "Vision-Language-Action Robotics"
    - Chapter 1: "Understanding Vision-Language-Action Models"
    - Chapter 2: "Natural Language Processing for Robotics"
    - Chapter 3: "Action Planning and Execution"
    - Chapter 4: "Human-Robot Interaction and Communication"

### docusaurus.config.js
- Removed blog configuration from presets
- Removed blog link from navbar
- Removed blog link from footer

## Outcome
- Sidebar now displays descriptive chapter names instead of generic "chapter1", "chapter2", etc.
- Navigation is more intuitive and user-friendly
- Site builds successfully without broken links
- Consistent formatting across all modules

## Validation
- Site builds successfully with `npx docusaurus build`
- Local server runs properly with `npx docusaurus serve`
- All navigation links work correctly
- Sidebar displays proper chapter names