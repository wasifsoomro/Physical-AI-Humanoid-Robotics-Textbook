---
id: 1
title: "Update Constitution for RAG Chatbot"
stage: "constitution"
date: "2025-12-08"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "main"
user: "wasifsoomro"
command: "/sp.constitution --append Add a new constitution section for Phase 2: Integrated RAG Chatbot Development."
labels: ["constitution", "rag", "chatbot", "phase2"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
outcome: "Successfully added Phase 2: Integrated RAG Chatbot Development section to the constitution"
evaluation: "Constitution updated with new section containing Core Principles, Key Standards, Constraints, and Success Criteria for the RAG chatbot development phase"
---

# Update Constitution for RAG Chatbot

## User Prompt
```
--append Add a new constitution section for Phase 2: Integrated RAG Chatbot Development.

Do NOT modify or replace any existing constitution content. Only append a new amendment.

Core Principles:
- Retrieval accuracy: chatbot must answer strictly from book content or selected text.
- Performance: responses should be low-latency and efficient.
- Security & privacy: safely handle user queries, embeddings, and logs.
- Reproducibility: RAG pipeline and API endpoints must be deterministic.

Key Standards:
- Use OpenAI Agents/ChatKit for AI.
- Use FastAPI for backend.
- Use Neon Postgres for metadata, chat history, and context.
- Use Qdrant Cloud Free Tier for vector storage.
- Chunk size: 300–800 tokens; overlap: 50–100.

Constraints:
- Must run inside the existing Docusaurus site.
- Must support “global retrieval” and “selected text retrieval”.
- Must respect token limits, Qdrant limits, and CORS rules.

Success Criteria:
- Chatbot embedded in the deployed book.
- Correct retrieval from book content and highl
```

## Response Summary
Successfully updated the constitution file by appending a new section for Phase 2: Integrated RAG Chatbot Development. The new section includes:

- Core Principles (Retrieval Accuracy, Performance, Security & Privacy, Reproducibility)
- Key Standards (specifying OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud)
- Constraints (integration with Docusaurus, retrieval modes, technical limits)
- Success Criteria (embedded chatbot, correct retrieval, fast responses)

The constitution version was updated from 0.1.0 to 0.2.0 with an updated Sync Impact Report and amendment date.