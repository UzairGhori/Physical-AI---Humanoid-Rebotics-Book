# Quickstart Guide: RAG Chatbot

**Feature**: 002-rag-chatbot
**Date**: 2025-12-17

This guide helps developers set up the RAG Chatbot development environment.

## Prerequisites

- **Python**: 3.11 or higher
- **Node.js**: 18+ (for Docusaurus)
- **Docker**: For local Qdrant and Postgres
- **Git**: Version control

## Quick Setup (5 minutes)

### 1. Clone and Navigate

```bash
cd Physical-AI-Humanoid-Roboticss-Hackathon
git checkout 002-rag-chatbot
```

### 2. Set Up Backend

```bash
cd Backend

# Create virtual environment
python -m venv .venv

# Activate (Windows)
.venv\Scripts\activate

# Activate (Unix/Mac)
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Configure Environment

```bash
# Copy example environment file
cp .env.example .env

# Edit .env with your API keys
```

Required environment variables:

```env
# OpenAI (Required)
OPENAI_API_KEY=sk-your-key-here

# Qdrant (Required)
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=  # Leave empty for local

# Postgres (Required)
DATABASE_URL=postgresql://dev:dev@localhost:5432/ragchat

# Optional Fallbacks
OPENROUTER_API_KEY=sk-or-your-key  # LLM fallback
BONSAI_API_KEY=your-key            # Embedding fallback

# Configuration
LOG_LEVEL=DEBUG
CORS_ORIGINS=http://localhost:3000
```

### 4. Start Infrastructure

```bash
# From repository root
docker-compose up -d qdrant postgres
```

This starts:
- Qdrant on `localhost:6333`
- Postgres on `localhost:5432`

### 5. Run Database Migrations

```bash
cd Backend
alembic upgrade head
```

### 6. Start the API Server

```bash
# Development mode with auto-reload
uvicorn app.main:app --reload --port 8000
```

API available at `http://localhost:8000`

### 7. Verify Setup

```bash
# Check health endpoint
curl http://localhost:8000/v1/health
```

Expected response:
```json
{
  "status": "healthy",
  "components": {
    "qdrant": {"status": "connected"},
    "postgres": {"status": "connected"},
    "openai": {"status": "available"},
    "embedding": {"status": "available", "model": "qwen"}
  }
}
```

## Running Tests

### Unit Tests

```bash
cd Backend
pytest tests/unit -v
```

### Integration Tests

```bash
# Requires running infrastructure
pytest tests/integration -v
```

### Contract Tests

```bash
# Requires running API server
pytest tests/contract -v
```

### All Tests with Coverage

```bash
pytest --cov=app --cov-report=html
```

## Ingesting Book Content

### Via CLI (Recommended)

```bash
cd Backend

# Ingest all docs
python -m cli.ingest --source ../docs/

# Ingest specific directory
python -m cli.ingest --source ../docs/modules/ros2-fundamentals/

# Force re-index (ignores cache)
python -m cli.ingest --source ../docs/ --force
```

### Via API

```bash
curl -X POST http://localhost:8000/v1/ingest \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-admin-key" \
  -d '{"source_directory": "docs/", "force_reindex": false}'
```

## Testing the Chatbot

### Query via cURL

```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS2?"}'
```

### Query with Streaming

```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS2?", "stream": true}'
```

### Interactive Testing

Open `http://localhost:8000/docs` for Swagger UI interactive testing.

## Frontend Development

### Start Docusaurus

```bash
# From repository root
npm install
npm start
```

Site available at `http://localhost:3000`

### Chat Widget Integration

The chat widget is automatically loaded on all pages. To test locally, ensure:

1. Backend is running on port 8000
2. CORS is configured for `http://localhost:3000`
3. Chat embed script is included in Docusaurus

## Common Issues

### "Connection refused" to Qdrant/Postgres

```bash
# Check if containers are running
docker ps

# Restart if needed
docker-compose restart
```

### "Invalid API key" errors

- Verify `.env` file exists and contains valid keys
- Ensure no trailing whitespace in API keys
- Restart the server after `.env` changes

### Slow embeddings

- First request may be slow (model loading)
- Subsequent requests should be <200ms
- Check if Qwen is falling back to Bonsai (see logs)

### "No relevant content found"

- Ensure book content is ingested: `GET /v1/metrics`
- Check `total_chunks` is > 0
- Try lowering relevance threshold temporarily for debugging

## Development Workflow

### Making Changes

1. Create feature branch from `002-rag-chatbot`
2. Make changes with tests
3. Run `pytest` before committing
4. Submit PR for review

### Code Style

```bash
# Format code
black app/ tests/

# Check linting
ruff check app/ tests/

# Type checking
mypy app/
```

### Logging

Logs use structured JSON format:

```python
import structlog
logger = structlog.get_logger()

logger.info("query_processed", question="What is ROS2?", latency_ms=1234)
```

View logs with:
```bash
# Pretty print in development
LOG_FORMAT=console uvicorn app.main:app --reload
```

## Useful Commands

| Command | Purpose |
|---------|---------|
| `uvicorn app.main:app --reload` | Start dev server |
| `pytest -v` | Run all tests |
| `python -m cli.ingest --source docs/` | Ingest content |
| `docker-compose up -d` | Start infrastructure |
| `docker-compose down -v` | Stop and clean up |
| `alembic upgrade head` | Apply migrations |
| `alembic downgrade -1` | Rollback migration |

## Getting Help

- **API Docs**: `http://localhost:8000/docs`
- **Spec**: `specs/002-rag-chatbot/spec.md`
- **Plan**: `specs/002-rag-chatbot/plan.md`
- **Contracts**: `specs/002-rag-chatbot/contracts/api-contracts.md`
