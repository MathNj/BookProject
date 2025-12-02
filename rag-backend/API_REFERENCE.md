# RAG Backend API Reference

Complete API documentation for the Physical AI & Humanoid Robotics Textbook RAG Backend.

## Base URL

```
http://localhost:8000  (Development)
https://rag-api.example.com  (Production)
```

## Authentication

Currently, the API does not require authentication. Future versions will include JWT-based authentication.

## Endpoints

### 1. Health Check

Check if the RAG backend is running and healthy.

**Endpoint:**
```
GET /health
```

**Response (200 OK):**
```json
{
  "status": "ok",
  "service": "rag-backend",
  "version": "0.1.0"
}
```

**Example:**
```bash
curl http://localhost:8000/health
```

---

### 2. Root Information

Get service information and documentation links.

**Endpoint:**
```
GET /
```

**Response (200 OK):**
```json
{
  "message": "Physical AI & Humanoid Robotics RAG Backend",
  "docs": "/docs",
  "health": "/health"
}
```

**Example:**
```bash
curl http://localhost:8000/
```

---

### 3. Chat (RAG Query)

Query the textbook using natural language with RAG-powered responses.

**Endpoint:**
```
POST /chat
```

**Request Body:**
```json
{
  "messages": [
    {
      "role": "user",
      "content": "What is ROS 2 and why is it important for robotics?"
    }
  ],
  "model": "gemini-2.5-flash"
}
```

**Request Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `messages` | Array | Yes | List of message objects with `role` and `content` |
| `model` | String | No | Chat model to use (default: `gemini-2.5-flash`) |

**Message Object:**
```json
{
  "role": "user|assistant",
  "content": "Message text"
}
```

**Response (200 OK):**
```json
{
  "message": "ROS 2 (Robot Operating System 2) is a flexible middleware for robot software development...",
  "model": "gemini-2.5-flash",
  "success": true
}
```

**Response Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `message` | String | AI-generated response based on textbook context |
| `model` | String | Model used for generation |
| `success` | Boolean | Whether the request was successful |

**Error Response (400 Bad Request):**
```json
{
  "detail": "At least one message is required"
}
```

**Error Response (500 Internal Server Error):**
```json
{
  "detail": "Error processing chat request: [error message]"
}
```

**Example:**
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {
        "role": "user",
        "content": "What is ROS 2?"
      }
    ]
  }'
```

**Example with Multiple Messages (Conversation):**
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {
        "role": "user",
        "content": "What is ROS 2?"
      },
      {
        "role": "assistant",
        "content": "ROS 2 is a flexible middleware..."
      },
      {
        "role": "user",
        "content": "How do I install it?"
      }
    ]
  }'
```

---

### 4. Semantic Search

Search the textbook for passages similar to a query.

**Endpoint:**
```
POST /search
```

**Query Parameters:**

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `query` | String | Yes | - | Search query string |
| `limit` | Integer | No | 3 | Max number of results (1-10) |

**Response (200 OK):**
```json
{
  "results": [
    {
      "text": "The Nervous System (ROS 2) connects your robot's computational brain...",
      "source": "01-intro.md",
      "chapter": "The Nervous System (ROS 2)",
      "section": "Section 1",
      "relevance_score": 0.927
    },
    {
      "text": "ROS 2 nodes are independent processes that perform specific tasks...",
      "source": "02-nodes-and-topics.md",
      "chapter": "Nodes and Topics",
      "section": "Section 2",
      "relevance_score": 0.891
    }
  ],
  "count": 2,
  "query": "ROS 2"
}
```

**Response Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `results` | Array | List of matching passages |
| `count` | Integer | Number of results returned |
| `query` | String | Original search query |

**Search Result Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `text` | String | Passage text (up to 500 chars) |
| `source` | String | Source file name |
| `chapter` | String | Chapter title |
| `section` | String | Section number/title |
| `relevance_score` | Float | Cosine similarity score (0-1) |

**Error Response (400 Bad Request):**
```json
{
  "detail": "Query cannot be empty"
}
```

**Example:**
```bash
curl -X POST "http://localhost:8000/search?query=ROS%202&limit=3"
```

**Example with Python:**
```python
import requests

response = requests.post(
    "http://localhost:8000/search",
    params={
        "query": "ROS 2",
        "limit": 3
    }
)
results = response.json()
for result in results["results"]:
    print(f"- {result['chapter']}: {result['text'][:100]}...")
```

---

## Response Status Codes

| Code | Description |
|------|-------------|
| 200 | OK - Request successful |
| 400 | Bad Request - Invalid parameters |
| 500 | Internal Server Error - Server-side error |

## Rate Limiting

Currently, there are no rate limits. Future versions will implement:
- Per-IP rate limiting (1000 requests/hour)
- Per-API-key rate limiting (10000 requests/hour)
- Token-based fair-use policy

## CORS Headers

The API includes CORS headers for cross-origin requests:

```
Access-Control-Allow-Origin: http://localhost:3000
Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
Access-Control-Allow-Headers: *
```

## Interactive API Documentation

Once the server is running, access the interactive API documentation:

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

## Example Requests

### Python

```python
import requests

# Chat request
response = requests.post(
    "http://localhost:8000/chat",
    json={
        "messages": [
            {"role": "user", "content": "What is ROS 2?"}
        ]
    }
)
print(response.json()["message"])

# Search request
response = requests.post(
    "http://localhost:8000/search",
    params={"query": "ROS 2", "limit": 3}
)
for result in response.json()["results"]:
    print(f"{result['chapter']}: {result['relevance_score']}")
```

### JavaScript/TypeScript

```javascript
// Chat request
const response = await fetch('http://localhost:8000/chat', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    messages: [
      { role: 'user', content: 'What is ROS 2?' }
    ]
  })
});
const data = await response.json();
console.log(data.message);

// Search request
const params = new URLSearchParams({
  query: 'ROS 2',
  limit: '3'
});
const response = await fetch(
  `http://localhost:8000/search?${params}`
);
const data = await response.json();
data.results.forEach(r => {
  console.log(`${r.chapter}: ${r.relevance_score}`);
});
```

### cURL

```bash
# Chat
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"messages":[{"role":"user","content":"What is ROS 2?"}]}'

# Search
curl "http://localhost:8000/search?query=ROS%202&limit=3"
```

## Common Use Cases

### 1. Getting Help with a Topic

```python
response = requests.post(
    "http://localhost:8000/chat",
    json={
        "messages": [
            {"role": "user", "content": "Explain digital twins in robotics"}
        ]
    }
)
print(response.json()["message"])
```

### 2. Finding References

```python
response = requests.post(
    "http://localhost:8000/search",
    params={"query": "Isaac Sim", "limit": 5}
)
for result in response.json()["results"]:
    print(f"Found in {result['chapter']}: {result['source']}")
```

### 3. Building a Conversational Experience

```python
messages = []
while True:
    user_input = input("You: ")
    messages.append({"role": "user", "content": user_input})

    response = requests.post(
        "http://localhost:8000/chat",
        json={"messages": messages}
    )
    assistant_message = response.json()["message"]
    print(f"Assistant: {assistant_message}\n")

    messages.append({"role": "assistant", "content": assistant_message})
```

## Limits and Constraints

| Constraint | Value | Notes |
|-----------|-------|-------|
| Max query length | 8192 tokens | OpenAI API limit |
| Max results per search | 10 | Hard limit |
| Default search results | 3 | Balanced quality/speed |
| Min similarity threshold | 0.85 | Strict relevance |
| Chunk size | 500 chars | From ingestion |
| Vector dimensions | 1536 | OpenAI embedding size |

## Troubleshooting

### "Connection refused" Error

**Problem:** Cannot connect to `http://localhost:8000`

**Solution:**
```bash
# Check if server is running
curl http://localhost:8000/health

# If not, start it
cd rag-backend
poetry run python -m src.main
```

### "Query cannot be empty" Error

**Problem:** Sending empty search query

**Solution:**
```bash
# ❌ Wrong
curl -X POST "http://localhost:8000/search?query="

# ✅ Correct
curl -X POST "http://localhost:8000/search?query=ROS%202"
```

### "At least one message is required" Error

**Problem:** Sending chat request with empty messages

**Solution:**
```json
{
  "messages": [
    {
      "role": "user",
      "content": "Your question here"
    }
  ]
}
```

### Results have low relevance scores

**Problem:** Search results not relevant (scores < 0.7)

**Solution:**
- Try rephrasing the query
- Use more specific keywords
- Check if textbook was ingested correctly
- Increase limit to see more results

## Future Enhancements

- ✅ Chat with conversation history
- ✅ Semantic search
- 🔜 User authentication (JWT)
- 🔜 Rate limiting
- 🔜 Result reranking
- 🔜 Feedback tracking
- 🔜 Batch search
- 🔜 Export results (PDF, markdown)
