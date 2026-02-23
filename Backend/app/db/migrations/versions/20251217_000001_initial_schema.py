"""Initial schema - chunk_metadata, ingestion_runs, retrieval_logs, system_metrics

Revision ID: 20251217_000001
Revises:
Create Date: 2025-12-17

T009-T012: Create all database tables per data-model.md
"""

from typing import Sequence, Union

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "20251217_000001"
down_revision: Union[str, None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # -------------------------------------------------------------------------
    # T009: chunk_metadata table
    # -------------------------------------------------------------------------
    op.create_table(
        "chunk_metadata",
        sa.Column("id", sa.UUID(), primary_key=True, server_default=sa.text("gen_random_uuid()")),
        sa.Column("vector_id", sa.UUID(), nullable=False, unique=True),
        sa.Column("chunk_id", sa.String(255), nullable=False, unique=True),
        sa.Column("source_file", sa.String(500), nullable=False),
        sa.Column("chapter", sa.String(255), nullable=False),
        sa.Column("section", sa.String(255), nullable=True),
        sa.Column("position", sa.Integer(), nullable=False),
        sa.Column("content_hash", sa.String(64), nullable=False),
        sa.Column("char_count", sa.Integer(), nullable=False),
        sa.Column("token_count", sa.Integer(), nullable=False),
        sa.Column("embedding_model", sa.String(50), nullable=False, server_default="qwen"),
        sa.Column("created_at", sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text("NOW()")),
        sa.Column("updated_at", sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text("NOW()")),
        sa.CheckConstraint("position >= 0", name="valid_position"),
        sa.CheckConstraint("char_count > 0 AND token_count > 0", name="valid_counts"),
    )
    op.create_index("idx_chunk_metadata_chapter", "chunk_metadata", ["chapter"])
    op.create_index("idx_chunk_metadata_source", "chunk_metadata", ["source_file"])
    op.create_index("idx_chunk_metadata_hash", "chunk_metadata", ["content_hash"])

    # -------------------------------------------------------------------------
    # T010: ingestion_runs table
    # -------------------------------------------------------------------------
    op.create_table(
        "ingestion_runs",
        sa.Column("id", sa.UUID(), primary_key=True, server_default=sa.text("gen_random_uuid()")),
        sa.Column("started_at", sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text("NOW()")),
        sa.Column("completed_at", sa.TIMESTAMP(timezone=True), nullable=True),
        sa.Column("status", sa.String(20), nullable=False, server_default="running"),
        sa.Column("source_directory", sa.String(500), nullable=False),
        sa.Column("files_processed", sa.Integer(), server_default="0"),
        sa.Column("chunks_created", sa.Integer(), server_default="0"),
        sa.Column("chunks_updated", sa.Integer(), server_default="0"),
        sa.Column("chunks_deleted", sa.Integer(), server_default="0"),
        sa.Column("error_message", sa.Text(), nullable=True),
        sa.CheckConstraint("status IN ('running', 'completed', 'failed')", name="valid_status"),
    )
    op.create_index("idx_ingestion_runs_status", "ingestion_runs", ["status"])
    op.create_index("idx_ingestion_runs_started", "ingestion_runs", [sa.text("started_at DESC")])

    # -------------------------------------------------------------------------
    # T011: retrieval_logs table
    # -------------------------------------------------------------------------
    op.create_table(
        "retrieval_logs",
        sa.Column("id", sa.UUID(), primary_key=True, server_default=sa.text("gen_random_uuid()")),
        sa.Column("query_text", sa.Text(), nullable=False),
        sa.Column("query_embedding_model", sa.String(50), nullable=False),
        sa.Column("retrieved_chunk_ids", sa.ARRAY(sa.UUID()), nullable=False),
        sa.Column("relevance_scores", sa.ARRAY(sa.Float()), nullable=False),
        sa.Column("top_score", sa.Float(), nullable=False),
        sa.Column("response_generated", sa.Boolean(), nullable=False, server_default="true"),
        sa.Column("fallback_triggered", sa.Boolean(), nullable=False, server_default="false"),
        sa.Column("latency_ms", sa.Integer(), nullable=False),
        sa.Column("created_at", sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text("NOW()")),
    )
    op.create_index("idx_retrieval_logs_created", "retrieval_logs", [sa.text("created_at DESC")])
    op.create_index(
        "idx_retrieval_logs_fallback",
        "retrieval_logs",
        ["fallback_triggered"],
        postgresql_where=sa.text("fallback_triggered = true"),
    )

    # -------------------------------------------------------------------------
    # T012: system_metrics table
    # -------------------------------------------------------------------------
    op.create_table(
        "system_metrics",
        sa.Column("id", sa.UUID(), primary_key=True, server_default=sa.text("gen_random_uuid()")),
        sa.Column("metric_name", sa.String(100), nullable=False),
        sa.Column("metric_value", sa.Float(), nullable=False),
        sa.Column("recorded_at", sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text("NOW()")),
    )
    op.create_index("idx_metrics_name_time", "system_metrics", ["metric_name", sa.text("recorded_at DESC")])


def downgrade() -> None:
    op.drop_table("system_metrics")
    op.drop_table("retrieval_logs")
    op.drop_table("ingestion_runs")
    op.drop_table("chunk_metadata")
