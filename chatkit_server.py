from dotenv import load_dotenv
load_dotenv()

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
import os
import google.generativeai as genai
from qdrant_client import QdrantClient

# -------------------------------------------------
# Environment Variables
# -------------------------------------------------
QDRANT_URL = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "my_collection")
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")

# -------------------------------------------------
# Initialize Qdrant
# -------------------------------------------------
qdrant = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
)
print("✅ Connected to Qdrant")

# -------------------------------------------------
# Gemini Setup
# -------------------------------------------------
genai.configure(api_key=GOOGLE_API_KEY)

EMBED_MODEL = "models/text-embedding-004"
LLM = genai.GenerativeModel("gemini-2.5-flash-lite")

# -------------------------------------------------
# FastAPI App
# -------------------------------------------------
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------------------------------------
# Helper: Greeting Detection
# -------------------------------------------------
def is_greeting(text: str) -> bool:
    greetings = [
        "hi", "hello", "hey", "assalam", "assalamualaikum",
        "salam", "aoa", "good morning", "good evening", "good afternoon"
    ]
    text = text.lower().strip()
    return any(text == g or text.startswith(g + " ") for g in greetings)

# -------------------------------------------------
# RAG + Fallback (BOOK FIRST)
# -------------------------------------------------
async def rag_query(question: str, selected_text: str = ""):
    print("\n🟡 USER QUESTION:", question)

    context_chunks = []

    try:
        # 1️⃣ Generate embedding
        emb_response = genai.embed_content(
            model=EMBED_MODEL,
            content=question
        )
        user_vector = emb_response["embedding"]

        # 2️⃣ Query Qdrant
        search_results = qdrant.query_points(
            collection_name=COLLECTION_NAME,
            query=user_vector,
            limit=5
        )

        # 3️⃣ Extract textbook context using similarity score
        MIN_SCORE = 0.35   # tune: 0.30 – 0.45

        for point in search_results.points:
            score = point.score or 0
            text = point.payload.get("text", "")

            if score >= MIN_SCORE and text:
                context_chunks.append(text)

    except Exception as e:
        print("❌ Qdrant / Embedding error:", e)

    # Add user-selected text (optional)
    if selected_text:
        context_chunks.insert(0, f"User selected text:\n{selected_text}")

    full_context = (
        "\n\n".join(context_chunks)
        if context_chunks
        else "No textbook context found."
    )

    # -------------------------------------------------
    # Prompt (Textbook Priority)
    # -------------------------------------------------
    prompt = f"""
You are an expert tutor for **Physical AI and Humanoid Robotics**.

Rules:
1. If textbook context is provided, base your answer primarily on it.
2. If context is missing, answer using your own robotics knowledge.
3. Keep explanations clear and student-friendly.
4. Never refuse to answer.

Textbook Context:
{full_context}

Question:
{question}

Answer:
"""

    try:
        response = LLM.generate_content(prompt)
        return response.text
    except Exception as e:
        print("❌ LLM error:", e)
        return "Sorry, I couldn't generate an answer right now."

# -------------------------------------------------
# API Endpoint
# -------------------------------------------------
@app.post("/chat")
async def chat(request: Request):
    data = await request.json()

    user_query = data.get("user_query", "")
    selected_text = data.get("selected_text", "")

    if not user_query.strip():
        return {
            "answer": "Please ask a question related to Physical AI or Humanoid Robotics."
        }

    # 👋 Greeting → short reply
    if is_greeting(user_query):
        return {
            "answer": "Hello! 👋 You can ask me anything about Physical AI or Humanoid Robotics."
        }

    # 🤖 RAG / LLM response
    answer = await rag_query(user_query, selected_text)
    return {"answer": answer}

# -------------------------------------------------
# Health Check
# -------------------------------------------------
@app.get("/")
async def home():
    return {
        "message": "✅ Physical AI & Humanoid Robotics Chatbot Backend is running"
    }
