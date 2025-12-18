# API Contracts: RAG Chatbot

**Feature**: 002-rag-chatbot
**Date**: 2025-12-17
**Base URL**: `https://api.{domain}/v1`

## Overview

This document defines the REST API contracts for the RAG Chatbot backend. All endpoints use JSON for request/response bodies unless otherwise specified.

## Authentication

| Header | Value | Required |
|--------|-------|----------|
| `X-API-Key` | API key for admin endpoints | `/ingest`, `/metrics` only |
| `Content-Type` | `application/json` | All POST requests |

Public endpoints (`/query`, `/health`) do not require authentication.

---

## Endpoints

### 1. POST `/query`

Submit a question and receive a grounded answer based on book content.

#### Request

```http
POST /v1/query HTTP/1.1
Content-Type: application/json

{
  "question": "What is ROS2 and how does it differ from ROS1?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "stream": false
}
```

| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| `question` | string | Yes | 1-1000 chars | User's question |
| `session_id` | string | No | UUID format | Session ID for context |
| `stream` | boolean | No | Default: false | Enable SSE streaming |

#### Response (Non-Streaming)

```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "answer": "ROS2 (Robot Operating System 2) is the next generation of the ROS framework...",
  "sources": [
    "Chapter 1: Introduction to ROS2",
    "Chapter 2: ROS2 vs ROS1 Comparison"
  ],
  "confidence": "high",
  "retrieval_latency_ms": 87,
  "generation_latency_ms": 1245
}
```

| Field | Type | Description |
|-------|------|-------------|
| `answer` | string | Generated response text |
| `sources` | string[] | Chapter/section references |
| `confidence` | string | `high` (score ≥0.8), `medium` (≥0.7), `low` (<0.7) |
| `retrieval_latency_ms` | integer | Time for vector search |
| `generation_latency_ms` | integer | Time for LLM response |

#### Response (Streaming)

When `stream: true`, response uses Server-Sent Events:

```http
HTTP/1.1 200 OK
Content-Type: text/event-stream
Cache-Control: no-cache

event: token
data: {"token": "ROS2"}

event: token
data: {"token": " (Robot"}

event: token
data: {"token": " Operating"}

...

event: sources
data: {"sources": ["Chapter 1: Introduction to ROS2"]}

event: done
data: {"retrieval_latency_ms": 87, "generation_latency_ms": 1245}
```

#### Error Responses

| Status | Code | Description |
|--------|------|-------------|
| 400 | `INVALID_QUERY` | Empty or too long question |
| 429 | `RATE_LIMITED` | Too many requests |
| 503 | `SERVICE_UNAVAILABLE` | Backend services down |

```json
{
  "error": {
    "code": "INVALID_QUERY",
    "message": "Question must be between 1 and 1000 characters",
    "details": {"length": 1523}
  }
}
```

#### Fallback Response

When no relevant content is found (top score < 0.7):

```json
{
  "answer": "I don't know based on the book content.",
  "sources": [],
  "confidence": "low",
  "retrieval_latency_ms": 92,
  "generation_latency_ms": 0
}
```

---

### 2. POST `/ingest`

Trigger ingestion of book content into the vector store.

#### Request

```http
POST /v1/ingest HTTP/1.1
Content-Type: application/json
X-API-Key: sk-admin-xxxxx

{
  "source_directory": "docs/",
  "force_reindex": false
}
```

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `source_directory` | string | No | `docs/` | Path to content directory |
| `force_reindex` | boolean | No | false | Re-embed unchanged content |

#### Response

```http
HTTP/1.1 202 Accepted
Content-Type: application/json

{
  "run_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "status": "running",
  "message": "Ingestion started. Check /ingest/{run_id} for progress."
}
```

#### Progress Check

```http
GET /v1/ingest/7c9e6679-7425-40de-944b-e07fc1f90ae7 HTTP/1.1
X-API-Key: sk-admin-xxxxx
```

```json
{
  "run_id": "7c9e6679-7425-40de-944b-e07fc1f90ae7",
  "status": "completed",
  "started_at": "2025-12-17T10:30:00Z",
  "completed_at": "2025-12-17T10:45:23Z",
  "files_processed": 52,
  "chunks_created": 847,
  "chunks_updated": 23,
  "chunks_deleted": 5,
  "duration_seconds": 923.4
}
```

#### Status Values

| Status | Description |
|--------|-------------|
| `running` | Ingestion in progress |
| `completed` | Successfully finished |
| `failed` | Error occurred (see `error_message`) |

#### Error Responses

| Status | Code | Description |
|--------|------|-------------|
| 401 | `UNAUTHORIZED` | Missing or invalid API key |
| 404 | `DIRECTORY_NOT_FOUND` | Source directory doesn't exist |
| 409 | `INGESTION_IN_PROGRESS` | Another ingestion is running |

---

### 3. GET `/health`

Check system health and component status.

#### Request

```http
GET /v1/health HTTP/1.1
```

#### Response

```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "status": "healthy",
  "components": {
    "qdrant": {
      "status": "connected",
      "latency_ms": 12
    },
    "postgres": {
      "status": "connected",
      "latency_ms": 8
    },
    "openai": {
      "status": "available",
      "model": "gpt-4o-mini"
    },
    "embedding": {
      "status": "available",
      "model": "qwen",
      "fallback_available": true
    }
  },
  "timestamp": "2025-12-17T10:30:00Z"
}
```

#### Status Values

| Overall Status | Meaning |
|----------------|---------|
| `healthy` | All components operational |
| `degraded` | Some components impaired but functional |
| `unhealthy` | Critical components down |

#### Degraded Example

```json
{
  "status": "degraded",
  "components": {
    "qdrant": {"status": "connected", "latency_ms": 12},
    "postgres": {"status": "connected", "latency_ms": 8},
    "openai": {"status": "available", "model": "gpt-4o-mini"},
    "embedding": {
      "status": "fallback",
      "model": "bonsai",
      "fallback_reason": "Qwen service timeout"
    }
  },
  "timestamp": "2025-12-17T10:30:00Z"
}
```

---

### 4. GET `/metrics`

Retrieve system performance metrics.

#### Request

```http
GET /v1/metrics HTTP/1.1
X-API-Key: sk-admin-xxxxx
```

#### Response

```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "indexing": {
    "total_chunks": 2847,
    "total_files": 52,
    "last_ingestion": "2025-12-17T08:00:00Z",
    "index_size_mb": 45.2
  },
  "queries": {
    "total_24h": 1523,
    "avg_latency_ms": 1847,
    "p95_latency_ms": 2890,
    "fallback_rate_percent": 2.3
  },
  "retrieval": {
    "avg_top_score": 0.82,
    "avg_precision_at_5": 0.78,
    "out_of_scope_rate_percent": 8.7
  },
  "system": {
    "uptime_hours": 168.5,
    "error_rate_percent": 0.12
  },
  "timestamp": "2025-12-17T10:30:00Z"
}
```

#### Error Responses

| Status | Code | Description |
|--------|------|-------------|
| 401 | `UNAUTHORIZED` | Missing or invalid API key |

---

## Error Response Format

All errors follow a consistent structure:

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable description",
    "details": {}
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_QUERY` | 400 | Malformed query input |
| `INVALID_REQUEST` | 400 | Malformed request body |
| `UNAUTHORIZED` | 401 | Missing/invalid API key |
| `NOT_FOUND` | 404 | Resource not found |
| `RATE_LIMITED` | 429 | Too many requests |
| `INGESTION_IN_PROGRESS` | 409 | Conflict with running job |
| `INTERNAL_ERROR` | 500 | Unexpected server error |
| `SERVICE_UNAVAILABLE` | 503 | Dependencies unavailable |

---

## Rate Limiting

| Endpoint | Limit | Window |
|----------|-------|--------|
| `/query` | 10 requests | per minute per IP |
| `/ingest` | 1 request | (only one concurrent) |
| `/health` | 60 requests | per minute per IP |
| `/metrics` | 10 requests | per minute per API key |

Rate limit headers included in responses:

```http
X-RateLimit-Limit: 10
X-RateLimit-Remaining: 7
X-RateLimit-Reset: 1702812600
```

When rate limited:

```http
HTTP/1.1 429 Too Many Requests
Retry-After: 45

{
  "error": {
    "code": "RATE_LIMITED",
    "message": "Rate limit exceeded. Try again in 45 seconds.",
    "details": {
      "limit": 10,
      "window_seconds": 60,
      "retry_after": 45
    }
  }
}
```

---

## Queue Status (Rate Limit Exceeded)

When the system is at capacity, queries are queued:

```http
HTTP/1.1 202 Accepted
Content-Type: application/json

{
  "status": "queued",
  "queue_position": 3,
  "estimated_wait_seconds": 15,
  "message": "Processing... You are #3 in queue."
}
```

Client should poll or maintain SSE connection for response.

---

## CORS Configuration

```http
Access-Control-Allow-Origin: https://your-docusaurus-site.github.io
Access-Control-Allow-Methods: GET, POST, OPTIONS
Access-Control-Allow-Headers: Content-Type, X-API-Key
Access-Control-Max-Age: 86400
```

---

## OpenAPI Specification

Full OpenAPI 3.0 spec available at `/v1/openapi.json` when server is running.
