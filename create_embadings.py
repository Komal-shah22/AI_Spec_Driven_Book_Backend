import os
from glob import glob
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import google.generativeai as genai
from tqdm import tqdm
import uuid

# -------------------------------------------------
# Load environment variables
# -------------------------------------------------
load_dotenv()

COLLECTION_NAME = os.getenv("COLLECTION_NAME", "my_collection")
EMBEDDING_SIZE = 768
DOCS_PATH = "docs/**/*.md"

# -------------------------------------------------
# Gemini Config
# -------------------------------------------------
genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))
EMBED_MODEL = "models/text-embedding-004"

# -------------------------------------------------
# Qdrant Client
# -------------------------------------------------
QDRANT_URL = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# -------------------------------------------------
# Ensure Collection Exists
# -------------------------------------------------
try:
    client.get_collection(collection_name=COLLECTION_NAME)
    print(f"✅ Collection '{COLLECTION_NAME}' exists")
except Exception:
    print(f"⚠️ Creating collection '{COLLECTION_NAME}'")
    client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=EMBEDDING_SIZE,
            distance=Distance.COSINE
        )
    )

# -------------------------------------------------
# Chunking Function (IMPORTANT)
# -------------------------------------------------
def chunk_text(text, chunk_size=400, overlap=50):
    words = text.split()
    chunks = []

    start = 0
    while start < len(words):
        end = start + chunk_size
        chunk = " ".join(words[start:end])
        chunks.append(chunk)
        start += chunk_size - overlap

    return chunks

# -------------------------------------------------
# Read & Embed Markdown Files
# -------------------------------------------------
files = glob(DOCS_PATH, recursive=True)
points = []

print(f"\n📚 Found {len(files)} markdown files\n")

for file in tqdm(files, desc="Embedding markdown files", unit="file"):
    with open(file, "r", encoding="utf-8") as f:
        text = f.read()

    chunks = chunk_text(text)

    for idx, chunk in enumerate(chunks):
        emb = genai.embed_content(
            model=EMBED_MODEL,
            content=chunk
        )

        vector = emb["embedding"]

        points.append(
            PointStruct(
                id=str(uuid.uuid4()),
                vector=vector,
                payload={
                    "text": chunk,
                    "source_file": file,
                    "chunk_id": idx,
                    "type": "textbook"
                }
            )
        )

# -------------------------------------------------
# Upload to Qdrant
# -------------------------------------------------
if points:
    print(f"\n📤 Uploading {len(points)} chunks to Qdrant...")
    client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )
    print("✅ Embedding & upload completed successfully!")
else:
    print("⚠️ No content found to embed.")
