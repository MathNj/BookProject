"""
RAG Agent using OpenAI Agents framework with Gemini API.

Implements an agent with access to vector store search tool for retrieval-augmented generation.
The agent answers questions about the robotics textbook using semantic search.
"""

import json
import logging
import asyncio
from typing import Any, Optional
from openai import AsyncOpenAI
from agents import Agent, function_tool, Runner, OpenAIChatCompletionsModel
from src.services.vector_store import get_vector_store
from src.config import settings

logger = logging.getLogger(__name__)

# Configure AsyncOpenAI client with Gemini API
api_key = settings.api_key or settings.openai_api_key
gemini_client = AsyncOpenAI(
    api_key=api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Configure the model with the Gemini client
model = OpenAIChatCompletionsModel(
    model=settings.chat_model,
    openai_client=gemini_client
)

vector_store = get_vector_store()

# Define the search tool
@function_tool
def search_book_tool(query: str) -> str:
    """
    Search the textbook for relevant content using semantic similarity.

    This tool queries the vector store to find passages related to the user's question.

    Args:
        query: The search query (user's question or topic)

    Returns:
        str: JSON formatted search results with book passages and metadata
    """
    try:
        results = vector_store.search(query, limit=3, min_similarity=0.85)

        if not results:
            return json.dumps({
                "success": False,
                "message": "No relevant passages found in the textbook",
                "results": []
            })

        # Format results for agent consumption
        formatted_results = []
        for result in results:
            formatted_results.append({
                "text": result.get("text", ""),
                "source": result.get("source", "unknown"),
                "chapter": result.get("chapter", "unknown"),
                "section": result.get("section", "unknown"),
                "relevance_score": round(result.get("score", 0), 3)
            })

        return json.dumps({
            "success": True,
            "count": len(formatted_results),
            "results": formatted_results
        })
    except Exception as e:
        logger.error(f"Search tool error: {e}")
        return json.dumps({
            "success": False,
            "error": str(e),
            "results": []
        })
# System prompt for the agent
SYSTEM_PROMPT = """You are an expert Robotics Professor specializing in Physical AI and Humanoid Robotics.

You have access to a comprehensive textbook on these topics. Your role is to answer student questions accurately and helpfully.

IMPORTANT RULES:
1. Answer ONLY based on the provided book context from the search results.
2. If the search results don't contain relevant information, say "I don't have information about that in the textbook."
3. Always cite the chapter and section when referencing the textbook.
4. Be clear, concise, and educational in your explanations.
5. If a question requires information outside the textbook, politely decline and refocus on textbook content.

Always use the search_book tool to find relevant passages before answering."""


class RAGAgent:
    """Agent for retrieval-augmented generation using OpenAI Agents framework with Gemini API."""

    def __init__(self, model_obj: OpenAIChatCompletionsModel = None):
        """
        Initialize the RAG Agent.

        Args:
            model_obj: OpenAIChatCompletionsModel to use for the agent (default from config)
        """
        self.model_obj = model_obj or model
        self.system_prompt = SYSTEM_PROMPT
        self.vector_store = vector_store

        # Create the agent with the search_book tool
        self.agent = Agent(
            name="Physical AI Textbook Assistant",
            instructions=self.system_prompt,
            model=self.model_obj,
            tools=[search_book_tool]
        )

    async def run_sync(self, messages: list[dict[str, str]]) -> str:
        """
        Run the agent with the given messages and return the final response.

        Args:
            messages: List of message dicts with 'role' and 'content' keys
                     Example: [{"role": "user", "content": "What is ROS 2?"}]

        Returns:
            str: The agent's response to the user
        """
        try:
            logger.info(f"Calling model with agents framework")

            # Extract the user's query from the messages
            user_message = messages[-1].get("content", "") if messages else ""

            # Run the agent using Runner.run (async)
            result = await Runner.run(
                starting_agent=self.agent,
                input=user_message
            )

            # Extract final response text from the result
            final_response = result.final_output if hasattr(result, 'final_output') else str(result)

            if final_response:
                logger.info("Successfully generated response")
                return final_response
            else:
                logger.warning("Model returned empty response")
                return "I couldn't generate a response. Please try again."

        except Exception as e:
            logger.error(f"Error in agent.run_sync: {e}")
            raise


# Singleton instance
_agent_instance: Optional[RAGAgent] = None


def get_agent(model_obj: OpenAIChatCompletionsModel = None) -> RAGAgent:
    """
    Get the singleton RAG Agent instance.

    Args:
        model_obj: OpenAIChatCompletionsModel to use (only used on first call)

    Returns:
        RAGAgent: The singleton agent instance
    """
    global _agent_instance
    if _agent_instance is None:
        _agent_instance = RAGAgent(model_obj=model_obj or model)
    return _agent_instance
