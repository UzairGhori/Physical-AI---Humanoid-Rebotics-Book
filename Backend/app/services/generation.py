# =============================================================================
# RAG Chatbot Backend - Generation Service
# =============================================================================
# LLM response generation with OpenAI
# T026-T029: Generation service with streaming and source attribution
# =============================================================================

import time
from typing import AsyncGenerator, Optional

from openai import AsyncOpenAI

from app.config import get_settings
from app.models.entities import RetrievedChunk
from app.utils.logging import service_logger

logger = service_logger

# System prompt for RAG chatbot
RAG_SYSTEM_PROMPT = """You are an expert assistant for the "Physical AI & Humanoid Robotics" book. Your role is to answer questions accurately based ONLY on the provided book content.

CRITICAL RULES:
1. ONLY use information from the provided context to answer questions
2. If the context doesn't contain relevant information, say "I don't know based on the book content."
3. Always cite the chapter/section when providing information
4. Be concise but thorough in your explanations
5. If the question is unclear, ask for clarification
6. Do not make up or hallucinate information not present in the context

You are helping readers understand concepts from the book about physical AI, humanoid robotics, ROS2, and related topics."""

FALLBACK_RESPONSE = "I don't know based on the book content."


class GenerationService:
    """
    Service for generating LLM responses using OpenAI.

    Supports both streaming and non-streaming responses.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "gpt-4o-mini",
        openrouter_api_key: Optional[str] = None,
        openrouter_model: str = "openai/gpt-4o-mini",
    ):
        """
        Initialize generation service.

        Args:
            api_key: OpenAI API key
            model: OpenAI model to use
            openrouter_api_key: OpenRouter API key for fallback
            openrouter_model: OpenRouter model for fallback
        """
        self._api_key = api_key
        self._model = model
        self._openrouter_api_key = openrouter_api_key
        self._openrouter_model = openrouter_model
        self._client: Optional[AsyncOpenAI] = None
        self._openrouter_client: Optional[AsyncOpenAI] = None

    @property
    def client(self) -> AsyncOpenAI:
        """Get OpenAI client."""
        if self._client is None:
            self._client = AsyncOpenAI(api_key=self._api_key)
        return self._client

    @property
    def openrouter_client(self) -> Optional[AsyncOpenAI]:
        """Get OpenRouter client for fallback."""
        if self._openrouter_api_key and self._openrouter_client is None:
            self._openrouter_client = AsyncOpenAI(
                api_key=self._openrouter_api_key,
                base_url="https://openrouter.ai/api/v1",
            )
        return self._openrouter_client

    def build_prompt(
        self,
        question: str,
        chunks: list[RetrievedChunk],
    ) -> tuple[str, str]:
        """
        Build prompt with context from retrieved chunks.

        Args:
            question: User's question
            chunks: Retrieved relevant chunks

        Returns:
            Tuple of (system_prompt, user_message)
        """
        # Build context from chunks
        context_parts = []
        for i, chunk in enumerate(chunks, 1):
            source = f"{chunk.chapter}"
            if chunk.section:
                source += f": {chunk.section}"
            context_parts.append(
                f"[Source {i}: {source}]\n{chunk.content}"
            )

        context = "\n\n".join(context_parts)

        user_message = f"""Based on the following book content, please answer the question.

BOOK CONTENT:
{context}

QUESTION: {question}

Please provide a clear, accurate answer based only on the content above. Cite relevant sources."""

        return RAG_SYSTEM_PROMPT, user_message

    async def generate(
        self,
        question: str,
        chunks: list[RetrievedChunk],
        confidence: str = "high",
    ) -> tuple[str, int]:
        """
        Generate a response for the question.

        Args:
            question: User's question
            chunks: Retrieved relevant chunks
            confidence: Confidence level of the retrieved chunks (high, medium, low)

        Returns:
            Tuple of (answer, latency_ms)
        """
        # Return fallback if not relevant
        if confidence == "low" or not chunks:
            return FALLBACK_RESPONSE, 0

        start_time = time.perf_counter()

        system_prompt, user_message = self.build_prompt(question, chunks)

        try:
            response = await self.client.chat.completions.create(
                model=self._model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message},
                ],
                temperature=0.3,
                max_tokens=1000,
            )

            answer = response.choices[0].message.content or FALLBACK_RESPONSE
            if confidence == "medium":
                sources = self.get_source_attribution(chunks)
                sources_text = "\n - ".join(sources)
                answer += f"\n\nI'm not fully confident in this answer. You may want to review these sections:\n - {sources_text}"
            latency_ms = int((time.perf_counter() - start_time) * 1000)

            logger.info(
                "Generation completed",
                model=self._model,
                latency_ms=latency_ms,
                tokens=response.usage.total_tokens if response.usage else 0,
            )

            return answer, latency_ms

        except Exception as e:
            logger.error("OpenAI generation failed", error=str(e))

            # Try OpenRouter fallback
            if self.openrouter_client:
                logger.info("Attempting OpenRouter fallback")
                return await self._generate_with_openrouter(
                    system_prompt, user_message, start_time
                )

            raise

    async def _generate_with_openrouter(
        self,
        system_prompt: str,
        user_message: str,
        start_time: float,
    ) -> tuple[str, int]:
        """Generate using OpenRouter fallback."""
        if not self.openrouter_client:
            raise RuntimeError("OpenRouter not configured")

        response = await self.openrouter_client.chat.completions.create(
            model=self._openrouter_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message},
            ],
            temperature=0.3,
            max_tokens=1000,
        )

        answer = response.choices[0].message.content or FALLBACK_RESPONSE
        latency_ms = int((time.perf_counter() - start_time) * 1000)

        logger.info(
            "OpenRouter generation completed",
            model=self._openrouter_model,
            latency_ms=latency_ms,
        )

        return answer, latency_ms

    async def generate_stream(
        self,
        question: str,
        chunks: list[RetrievedChunk],
        confidence: str = "high",
    ) -> AsyncGenerator[str, None]:
        """
        Generate a streaming response for the question.

        Args:
            question: User's question
            chunks: Retrieved relevant chunks
            confidence: Confidence level of the retrieved chunks (high, medium, low)

        Yields:
            Token strings as they are generated
        """
        # Return fallback if not relevant
        if confidence == "low" or not chunks:
            yield FALLBACK_RESPONSE
            return

        system_prompt, user_message = self.build_prompt(question, chunks)

        try:
            stream = await self.client.chat.completions.create(
                model=self._model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message},
                ],
                temperature=0.3,
                max_tokens=1000,
                stream=True,
            )

            async for chunk in stream:
                if chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content

            if confidence == "medium":
                sources = self.get_source_attribution(chunks)
                sources_text = "\n - ".join(sources)
                yield f"\n\nI'm not fully confident in this answer. You may want to review these sections:\n - {sources_text}"

        except Exception as e:
            logger.error("Streaming generation failed", error=str(e))
            yield FALLBACK_RESPONSE

    def get_source_attribution(self, chunks: list[RetrievedChunk]) -> list[str]:
        """
        Extract unique source attributions from chunks.

        Args:
            chunks: Retrieved chunks

        Returns:
            List of "Chapter: Section" strings
        """
        sources = []
        seen = set()

        for chunk in chunks:
            if chunk.section:
                source = f"{chunk.chapter}: {chunk.section}"
            else:
                source = chunk.chapter

            if source not in seen:
                sources.append(source)
                seen.add(source)

        return sources


# Global service instance
_generation_service: Optional[GenerationService] = None


def get_generation_service() -> GenerationService:
    """Get the global generation service instance."""
    global _generation_service
    if _generation_service is None:
        settings = get_settings()
        _generation_service = GenerationService(
            api_key=settings.openai_api_key,
            model=settings.openai_model,
            openrouter_api_key=settings.openrouter_api_key,
            openrouter_model=settings.openrouter_model,
        )
    return _generation_service


async def generate_response(
    question: str,
    chunks: list[RetrievedChunk],
    confidence: str = "high",
) -> tuple[str, int]:
    """
    Convenience function to generate response using global service.

    Args:
        question: User's question
        chunks: Retrieved chunks
        confidence: Confidence level of the retrieved chunks

    Returns:
        Tuple of (answer, latency_ms)
    """
    service = get_generation_service()
    return await service.generate(question, chunks, confidence)
