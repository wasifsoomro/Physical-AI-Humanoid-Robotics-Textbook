---
id: 7
title: Update Config for Vercel Deployment
stage: general
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Claude
command: general
labels: ["deployment", "configuration", "vercel", "documentation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "docusaurus.config.js"
tests: []
---

# Update Config for Vercel Deployment

## Context
Updated the Docusaurus configuration to work properly with Vercel deployment after encountering a baseUrl error. The site was showing "Your Docusaurus site did not load properly" with a suggestion to change baseUrl from /Physical-AI-Humanoid-Robotics-Textbook/ to /.

## Changes Made

### 1. Updated baseUrl
- Changed baseUrl from '/Physical-AI-Humanoid-Robotics-Textbook/' to '/'
- This is required for Vercel deployment where the site is served from root

### 2. Updated production URL
- Changed URL from 'https://wasifsoomro.github.io' to 'https://[your-vercel-project-url].vercel.app'
- This reflects the Vercel deployment domain

### 3. Updated comments
- Changed GitHub Pages specific comments to more generic deployment comments
- Maintained organizationName and projectName for potential multi-platform use

## Outcome
- Configuration now properly set for Vercel deployment
- baseUrl set to root ('/') as required by Vercel
- Site should load properly when deployed to Vercel
- Codebase updated and pushed to GitHub repository

## Validation
- Changes committed with message: "fix: update baseUrl for Vercel deployment"
- Pushed to main branch on GitHub
- Configuration ready for Vercel import and deployment

## Next Steps for Vercel Deployment
1. Import the repository in Vercel dashboard
2. Configure build settings (Framework: Docusaurus, Output: build)
3. Deploy the site
4. Site will be available at a Vercel URL