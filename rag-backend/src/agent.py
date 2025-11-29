"""
RAG Agent using OpenAI Agents SDK.

Implements an agent with access to vector store search tool for retrieval-augmented generation.
The agent answers questions about the robotics textbook using semantic search.
"""

import json
import logging
from typing import Any, Optional
from openai import OpenAI, APIError
from src.services.vector_store import get_vector_store

logger = logging.getLogger(__name__)

# Initialize clients
openai_client = OpenAI()
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
    """Agent for retrieval-augmented generation using OpenAI Agents SDK."""

    def __init__(self, model: str = "gpt-4o-mini"):
        """
        Initialize the RAG Agent.

        Args:
            model: OpenAI model to use for the agent (default: gpt-4o-mini)
        """
        self.model = model
        self.client = openai_client
        self.tools = [SEARCH_TOOL_DEFINITION]
        self.system_prompt = SYSTEM_PROMPT

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

        Raises:
            APIError: If the OpenAI API call fails
        """
        try:
            # Add system prompt to messages
            system_message = {
                "role": "system",
                "content": self.system_prompt
            }

            # Create conversation with system message
            conversation_messages = [system_message] + messages

            # Initial API call
            response = self.client.chat.completions.create(
                model=self.model,
                messages=conversation_messages,
                tools=self.tools,
                tool_choice="auto",
            )

            # Process response and handle tool calls in agentic loop
            max_iterations = 10
            iteration = 0

            while iteration < max_iterations:
                iteration += 1

                # Check if we got a response or need to process tool calls
                if response.stop_reason == "stop":
                    # Agent finished, return the final message
                    final_message = response.choices[0].message.content
                    return final_message or "No response generated"

                if response.stop_reason == "tool_calls":
                    # Process tool calls
                    assistant_message = response.choices[0].message

                    # Add assistant's message to conversation
                    conversation_messages.append({
                        "role": "assistant",
                        "content": assistant_message.content,
                        "tool_calls": [
                            {
                                "id": tc.id,
                                "type": "function",
                                "function": {
                                    "name": tc.function.name,
                                    "arguments": tc.function.arguments
                                }
                            }
                            for tc in assistant_message.tool_calls
                        ] if assistant_message.tool_calls else []
                    })

                    # Process each tool call
                    tool_results = []
                    for tool_call in assistant_message.tool_calls or []:
                        try:
                            tool_input = json.loads(tool_call.function.arguments)
                            tool_result = self._process_tool_call(
                                tool_call.function.name,
                                tool_input
                            )

                            tool_results.append({
                                "type": "tool",
                                "tool_use_id": tool_call.id,
                                "content": tool_result
                            })
                        except json.JSONDecodeError as e:
                            logger.error(f"Failed to parse tool arguments: {e}")
                            tool_results.append({
                                "type": "tool",
                                "tool_use_id": tool_call.id,
                                "content": json.dumps({"error": "Invalid tool input"})
                            })

                    # Add tool results to conversation
                    for result in tool_results:
                        conversation_messages.append({
                            "role": "user",
                            "content": result["content"]
                        })

                    # Continue the conversation with tool results
                    response = self.client.chat.completions.create(
                        model=self.model,
                        messages=conversation_messages,
                        tools=self.tools,
                        tool_choice="auto",
                    )
                else:
                    # Unexpected stop reason
                    logger.warning(f"Unexpected stop_reason: {response.stop_reason}")
                    return response.choices[0].message.content or "No response generated"

            # Max iterations reached
            logger.warning("Agent reached maximum iterations without completing")
            return "Agent loop exceeded maximum iterations. Please try again."

        except APIError as e:
            logger.error(f"OpenAI API error in agent.run: {e}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error in agent.run: {e}")
            raise


# Singleton instance
_agent_instance: Optional[RAGAgent] = None


def get_agent(model: str = "gpt-4o-mini") -> RAGAgent:
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
