"""
RAG Agent using OpenAI.

Implements an agent with access to vector store search tool for retrieval-augmented generation.
The agent answers questions about the robotics textbook using semantic search.
"""

import json
import logging
from typing import Any, Optional
from openai import OpenAI
from src.services.vector_store import get_vector_store
from src.config import settings

logger = logging.getLogger(__name__)

# Configure OpenAI client
api_key = settings.api_key or settings.openai_api_key
client = OpenAI(api_key=api_key)
vector_store = get_vector_store()

# Define the search tool
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


# Tool definition for OpenAI Agents API
SEARCH_TOOL_DEFINITION = {
    "type": "function",
    "function": {
        "name": "search_book",
        "description": "Search the Physical AI & Humanoid Robotics textbook for relevant content. Use this to find passages that answer the user's question.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The search query or question to find relevant book passages"
                }
            },
            "required": ["query"]
        }
    }
}

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
    """Agent for retrieval-augmented generation using OpenAI."""

    def __init__(self, model: str = None):
        """
        Initialize the RAG Agent.

        Args:
            model: OpenAI model to use for the agent (default from config)
        """
        self.model = model or settings.chat_model
        self.system_prompt = SYSTEM_PROMPT
        self.vector_store = vector_store
        self.client = client

    def _process_tool_call(self, tool_name: str, tool_input: dict) -> str:
        """
        Process a tool call from the agent.

        Args:
            tool_name: Name of the tool to call
            tool_input: Input parameters for the tool

        Returns:
            str: Tool execution result
        """
        if tool_name == "search_book":
            return search_book_tool(tool_input.get("query", ""))
        else:
            return json.dumps({"error": f"Unknown tool: {tool_name}"})

    def run(self, messages: list[dict[str, str]]) -> str:
        """
        Run the agent with the given messages and return the final response.

        Args:
            messages: List of message dicts with 'role' and 'content' keys
                     Example: [{"role": "user", "content": "What is ROS 2?"}]

        Returns:
            str: The agent's response to the user
        """
        try:
            # Prepare messages with system prompt
            system_message = {"role": "system", "content": self.system_prompt}
            all_messages = [system_message] + messages

            # Call OpenAI with tools
            logger.info(f"Calling OpenAI model: {self.model}")
            response = self.client.chat.completions.create(
                model=self.model,
                messages=all_messages,
                tools=[SEARCH_TOOL_DEFINITION],
                tool_choice="auto",
            )

            # Process the response
            while response.choices[0].finish_reason == "tool_calls":
                # Extract tool calls
                tool_calls = response.choices[0].message.tool_calls

                # Process each tool call
                tool_results = []
                for tool_call in tool_calls:
                    logger.info(f"Processing tool call: {tool_call.function.name}")
                    tool_result = self._process_tool_call(
                        tool_call.function.name,
                        json.loads(tool_call.function.arguments)
                    )
                    tool_results.append({
                        "type": "tool",
                        "tool_use_id": tool_call.id,
                        "content": tool_result
                    })

                # Continue conversation with tool results
                all_messages.append(response.choices[0].message)
                all_messages.append({
                    "role": "user",
                    "content": tool_results
                })

                # Get next response
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=all_messages,
                    tools=[SEARCH_TOOL_DEFINITION],
                    tool_choice="auto",
                )

            # Extract final response text
            final_response = response.choices[0].message.content
            if final_response:
                logger.info("Successfully generated response from OpenAI")
                return final_response
            else:
                logger.warning("OpenAI returned empty response")
                return "I couldn't generate a response. Please try again."

        except Exception as e:
            logger.error(f"Error in agent.run: {e}")
            raise


# Singleton instance
_agent_instance: Optional[RAGAgent] = None


def get_agent(model: str = None) -> RAGAgent:
    """
    Get the singleton RAG Agent instance.

    Args:
        model: OpenAI model to use (only used on first call)

    Returns:
        RAGAgent: The singleton agent instance
    """
    global _agent_instance
    if _agent_instance is None:
        _agent_instance = RAGAgent(model=model)
    return _agent_instance
