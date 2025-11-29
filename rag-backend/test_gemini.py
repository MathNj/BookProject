#!/usr/bin/env python3
"""
Test script to verify Gemini API connection with OpenAI SDK.
"""

import os
import asyncio
from dotenv import load_dotenv
from openai import AsyncOpenAI

# Load .env file
load_dotenv()

# Load environment
api_key = os.getenv("API_KEY") or os.getenv("GEMINI_API_KEY")
if not api_key:
    print("ERROR: API_KEY or GEMINI_API_KEY not set in .env")
    exit(1)

print(f"API Key (first 20 chars): {api_key[:20]}...")
print(f"Base URL: https://generativelanguage.googleapis.com/v1beta/openai/")
print()

async def test_connection():
    """Test basic connection to Gemini API."""
    client = AsyncOpenAI(
        api_key=api_key,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )

    try:
        print("Testing Gemini connection with simple completion...")
        response = await client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=[
                {"role": "user", "content": "Say hello"}
            ]
        )

        print("✓ Connection successful!")
        print(f"Response: {response.choices[0].message.content}")
        return True

    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_connection())
    exit(0 if success else 1)
