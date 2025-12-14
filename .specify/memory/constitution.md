<!--
Sync Impact Report:
Version change: 0.1.0 → 0.2.0 (Minor: Added Phase 2 RAG Chatbot Development section)
Modified principles: None
Added sections: Phase 2: Integrated RAG Chatbot Development
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
- runtime guidance docs (README.md, docs/quickstart.md, etc.): ⚠ pending
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Educational Book Constitution

## Core Principles

### Accuracy
All AI and humanoid robotics concepts MUST be verified from primary sources: robotics textbooks, research papers, Docusaurus docs, Context7 docs, Spec-Kit Plus documentation, and Claude Code references.

### Clarity
Content MUST be understandable for Class 5–8 students. Writing style MUST be conversational, educational, and practical.

### Reproducibility
All exercises, code examples, and tasks MUST work as described and be tested and runnable. Step-by-step instructions MUST be explicit and easy to follow.

### Engagement
Examples, illustrations, and practical activities SHOULD make learning interactive and fun.

### Structure
Each chapter MUST follow a predictable, modular, and reusable layout, including an Introduction, Core Concepts, Step-by-step Instructions, Practical Activity or Task, and Summary.

### AI-Assisted Writing Discipline
The Spec-Kit Plus workflow MUST be strictly followed, including Spec → Plan → Tasks → Implementation. No step of the pipeline may be skipped.

## Key Standards

All factual claims MUST be traceable to sources: robotics textbooks, research papers, Docusaurus docs, Context7 docs, Spec-Kit Plus documentation, and Claude Code references.
Writing style: conversational, educational, and practical.
Step-by-step instructions MUST be explicit and easy to follow.
Code samples and practical tasks MUST be tested and runnable.
Tools used: Docusaurus, Context7 MCP Server, Claude Code, Spec-Kit Plus.

## Constraints

Minimum 10 chapters covering key topics of Physical AI and Humanoid Robotics, including:
- Introduction to AI & Robotics
- Physical AI Concepts
- Humanoid Robot Design
- Sensors and Actuators
- Robotics Programming Basics
- AI Integration in Humanoids
- Motion & Navigation
- Voice & Gesture Interaction
- Mini Humanoid Projects
- Future of Physical AI & Robotics
Word count: 6,000–12,000 words total
Book format: Markdown files compatible with Docusaurus
Deployment target: GitHub Pages

## Success Criteria

Book fully generated following Spec → Plan → Tasks → Implementation workflow.
Docusaurus site builds successfully without errors.
GitHub Pages deployment works correctly.
All chapters are clear, reproducible, and understandable by Class 5–8 students.
All exercises and code examples tested and verified.
Zero hallucinated commands, tools, or concepts.

## Non-Negotiables

No step of the Spec-Kit Plus pipeline may be skipped.
No fictional commands, frameworks, or features.
All instructions and examples MUST be realistic, reproducible, and verifiable.

## Phase 2: Integrated RAG Chatbot Development

### Core Principles

#### Retrieval Accuracy
The chatbot MUST answer strictly from book content or selected text. No hallucinated responses or fabricated information are allowed.

#### Performance
Responses SHOULD be low-latency and efficient. API endpoints MUST respond within reasonable timeframes for a good user experience.

#### Security & Privacy
The system MUST safely handle user queries, embeddings, and logs. No sensitive data should be exposed or improperly stored.

#### Reproducibility
The RAG pipeline and API endpoints MUST be deterministic, producing consistent results for identical inputs.

### Key Standards

- Use OpenAI Agents/ChatKit for AI capabilities
- Use FastAPI for backend API development
- Use Neon Postgres for metadata, chat history, and context storage
- Use Qdrant Cloud Free Tier for vector storage
- Chunk size: 300–800 tokens with 50–100 token overlap for optimal retrieval

### Constraints

- The chatbot MUST run inside the existing Docusaurus site
- System MUST support both "global retrieval" and "selected text retrieval" modes
- Implementation MUST respect token limits, Qdrant limits, and CORS rules
- All components MUST integrate seamlessly with the existing book infrastructure

### Success Criteria

- Chatbot embedded directly in the deployed book interface
- Correct retrieval from book content and highlighted text
- Fast, accurate responses that enhance the learning experience
- Successful integration with the existing Docusaurus site
- Proper handling of context and conversation history

## Governance

This constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews MUST verify compliance. Complexity MUST be justified.

**Version**: 0.2.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-08
