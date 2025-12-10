import os
import asyncio
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional
from uuid import UUID, uuid4
import argparse
import hashlib
from pathlib import Path
import requests
from urllib.parse import urljoin, urlparse

from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import Litellm for Gemini integration
from litellm import acompletion
import litellm
litellm.suppress_debug_info = True

# Import Qdrant and document processing libraries
from qdrant_client import QdrantClient
from qdrant_client.http import models
from langchain_text_splitters import RecursiveCharacterTextSplitter
import tiktoken

app = FastAPI()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize Qdrant client
# Use local mode for development if no URL is provided
qdrant_url = os.getenv("QDRANT_URL")
if qdrant_url:
    qdrant_client = QdrantClient(
        url=qdrant_url,
        api_key=os.getenv("QDRANT_API_KEY")
    )
else:
    # Use in-memory mode for development
    qdrant_client = QdrantClient(":memory:")

# Collection name for documentation
DOCUMENTATION_COLLECTION = "humanoid-robotics-book"

# Create the documentation collection if it doesn't exist
try:
    qdrant_client.get_collection(DOCUMENTATION_COLLECTION)
except:
    qdrant_client.create_collection(
        collection_name=DOCUMENTATION_COLLECTION,
        vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # Using OpenAI embedding size as reference
    )

# Text splitter for document chunking
text_splitter = RecursiveCharacterTextSplitter(
    chunk_size=1000,
    chunk_overlap=200,
    length_function=len,
    is_separator_regex=False,
)

# In-memory storage for threads and messages (in production, use a proper database)
threads: Dict[str, Dict] = {}
messages: Dict[str, List[Dict]] = {}

class ThreadItemConverter:
    @staticmethod
    def convert_to_thread_items(messages: List[Dict]) -> List[Dict]:
        thread_items = []
        for msg in messages:
            role = msg.get('role', 'user')
            content = msg.get('content', '')
            thread_item = {
                "id": str(uuid4()),
                "type": "message",
                "role": role,
                "content": content,
                "timestamp": msg.get('timestamp', ''),
                "user_id": msg.get('user_id', 'default_user')
            }
            thread_items.append(thread_item)
        return thread_items

# Document ingestion and RAG functionality
def get_embeddings(text: str) -> List[float]:
    """Generate embeddings for text using a simple approach"""
    # For this implementation, we'll use a simple approach that creates
    # deterministic vectors based on the content of the text
    import hashlib
    import math

    # Create a hash of the text to generate a deterministic vector
    text_hash = hashlib.sha256(text.encode()).hexdigest()

    # Convert the hash to a vector of floats
    vector = []
    for i in range(0, len(text_hash), 2):
        if i + 1 < len(text_hash):
            hex_pair = text_hash[i:i+2]
            value = int(hex_pair, 16) / 255.0  # Normalize to 0-1 range
            vector.append(value)

    # Ensure the vector has the correct size (pad or truncate)
    while len(vector) < 1536:
        vector.append(0.0)
    vector = vector[:1536]  # Truncate if too long

    return vector

async def ingest_documentation(file_path: str, doc_id: str = None):
    """Ingest documentation file into Qdrant vector store"""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Document file not found: {file_path}")

    # Read the document content
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # Generate document ID if not provided
    if not doc_id:
        doc_id = hashlib.md5(content.encode()).hexdigest()

    # Split the document into chunks
    chunks = text_splitter.split_text(content)

    points = []
    for i, chunk in enumerate(chunks):
        # Use integer ID instead of string ID to avoid format issues
        point_id = int(hashlib.md5(f"{doc_id}_{i}".encode()).hexdigest()[:8], 16) % (2**31)  # Ensure it fits in int32

        # Generate a vector representation based on the content
        vector = get_embeddings(chunk)

        points.append(
            models.PointStruct(
                id=point_id,
                vector=vector,
                payload={
                    "content": chunk,
                    "document_id": doc_id,
                    "chunk_index": i,
                    "source_file": file_path
                }
            )
        )

    # Upload chunks to Qdrant
    if points:
        qdrant_client.upsert(
            collection_name=DOCUMENTATION_COLLECTION,
            points=points
        )

    return {
        "document_id": doc_id,
        "chunks_ingested": len(chunks),
        "file_path": file_path
    }


def retrieve_context(query: str, top_k: int = 5) -> List[Dict]:
    """Retrieve relevant context from documentation using vector search"""
    try:
        query_vector = get_embeddings(query)

        # Correct Qdrant API
        search_results = qdrant_client.query_points(
            collection_name=DOCUMENTATION_COLLECTION,
            query_vector=query_vector,
            limit=top_k,
            with_payload=True
        )

        context_chunks = []  # Initialize the variable
        for result in search_results.points:
            context_chunks.append({
                "content": result.payload["content"],
                "document_id": result.payload["document_id"],
                "relevance_score": result.score
            })

        return context_chunks
    except Exception as e:
        print(f"Error in retrieve_context: {str(e)}")
        return []


# Enhanced LiteLLM model with RAG capabilities
class LitellmModel:
    def __init__(self):
        self.model_name = "gemini/gemini-2.5-flash"
        self.api_base = os.getenv("LiteLLMMODEL")
        if self.api_base:
            litellm.api_base = self.api_base

    async def generate_response(self, messages: List[Dict], user_id: str = "default_user", use_rag: bool = True) -> str:
        try:
            # Prepare messages for the LLM
            llm_messages = []
            for msg in messages:
                role = msg.get('role', 'user')
                content = msg.get('content', '')
                llm_messages.append({"role": role, "content": content})

            # If RAG is enabled, retrieve context and check relevance
            context_text = ""
            source_citations = []

            if use_rag and messages:
                # Use the latest user message as the query for RAG
                latest_user_message = messages[-1].get('content', '') if messages[-1].get('role') == 'user' else ''
                if latest_user_message:
                    retrieved_contexts = retrieve_context(latest_user_message)
                    if retrieved_contexts:
                        # Check if the contexts are relevant to the query by looking at relevance scores
                        # Only proceed if we have reasonably relevant results
                        relevant_contexts = [ctx for ctx in retrieved_contexts if ctx['relevance_score'] > 0.1]  # Adjust threshold as needed

                        if relevant_contexts:
                            context_parts = []
                            for ctx in relevant_contexts:
                                context_parts.append(ctx['content'])
                                # Add source citation
                                source_citations.append({
                                    "content_snippet": ctx['content'][:100] + "...",
                                    "document_id": ctx['document_id'],
                                    "relevance_score": ctx['relevance_score']
                                })

                            context_text = "\n\n".join(context_parts)
                            # Add context to the beginning of the messages
                            llm_messages.insert(0, {
                                "role": "system",
                                "content": f"Use the following documentation context to answer the user's question:\n\n{context_text}\n\nIf the context doesn't contain relevant information, inform the user that you can only answer questions related to the Physical AI & Humanoid Robotics documentation."
                            })
                        else:
                            # No relevant context found, return a message indicating the query is not related to documentation
                            return "I can only answer questions related to the Physical AI & Humanoid Robotics documentation. Your query doesn't seem to be related to the available documentation."
                    else:
                        # No contexts found, return a message indicating the query is not related to documentation
                        return "I can only answer questions related to the Physical AI & Humanoid Robotics documentation. Your query doesn't seem to be related to the available documentation."
                else:
                    # No user message to process
                    return "Please provide a question related to the Physical AI & Humanoid Robotics documentation."
            else:
                # No RAG or messages, return a default message
                return "I can only answer questions related to the Physical AI & Humanoid Robotics documentation."

            # Call the Gemini model via LiteLLM
            response = await acompletion(
                model=self.model_name,
                messages=llm_messages,
                api_key=os.getenv("GEMINI_API_KEY"),
                temperature=0.7
            )

            response_content = response.choices[0].message.content

            # Add source citations to the response if any were found
            if source_citations:
                response_content += f"\n\nSources: {len(source_citations)} document(s) referenced"

            return response_content
        except Exception as e:
            print(f"Error calling LLM: {str(e)}")
            # Check if it's a rate limit error and provide a helpful message
            if "rate" in str(e).lower() or "quota" in str(e).lower() or "429" in str(e):
                return "I've reached my API usage limit for now. Please try again later or check the project's billing details for the Gemini API."
            return f"Sorry, I encountered an error: {str(e)}"


# Initialize the RAG model
llm_model = LitellmModel()


# API endpoints
@app.post("/chatkit/token")
async def create_chat_token(request: Request):
    # In a real implementation, you'd generate a proper token
    # For this demo, returning a simple response
    form_data = await request.json()
    user_id = form_data.get('user_id', 'default_user')
    return {"token": f"token_for_{user_id}", "user_id": user_id}


@app.get("/debug/debug/threads")
async def debug_list_threads():
    return {"threads": list(threads.values())}


@app.get("/")
def read_root():
    return {"message": "Gemini RAG Chatbot Backend is running!"}


@app.post("/chat/{thread_id}")
async def chat(thread_id: str, request: Request):
    """Handle chat messages with RAG functionality"""
    data = await request.json()
    user_message = data.get("message", "")
    user_id = data.get("user_id", "default_user")

    # Create thread if it doesn't exist
    if thread_id not in threads:
        threads[thread_id] = {
            "id": thread_id,
            "name": f"Thread {thread_id}",
            "created_by": user_id,
            "messages": []
        }
        messages[thread_id] = []

    # Add user message to thread
    user_msg = {
        "id": str(uuid4()),
        "thread_id": thread_id,
        "user_id": user_id,
        "content": user_message,
        "role": "user",
        "timestamp": ""
    }
    messages[thread_id].append(user_msg)

    # Generate response using RAG model
    conversation_history = messages[thread_id]
    response_content = await llm_model.generate_response(
        [{"role": msg["role"], "content": msg["content"]} for msg in conversation_history],
        user_id=user_id,
        use_rag=True
    )

    # Add assistant response to thread
    assistant_msg = {
        "id": str(uuid4()),
        "thread_id": thread_id,
        "user_id": "assistant",
        "content": response_content,
        "role": "assistant",
        "timestamp": ""
    }
    messages[thread_id].append(assistant_msg)

    return {
        "id": assistant_msg["id"],
        "content": response_content,
        "role": "assistant",
        "thread_id": thread_id
    }


# Sitemap ingestion endpoint
@app.post("/ingest-sitemap")
async def ingest_sitemap(request: Request):
    """Ingest content from a sitemap URL into the vector database"""
    try:
        data = await request.json()
        sitemap_url = data.get("sitemap_url")

        if not sitemap_url:
            raise HTTPException(status_code=400, detail="sitemap_url is required")

        # Process the sitemap and ingest content
        result = await ingest_sitemap_content(sitemap_url)

        return {
            "status": "success",
            "message": f"Ingested {result['chunks_ingested']} chunks from {result['urls_processed']} URLs",
            "details": result
        }
    except Exception as e:
        print(f"Error ingesting sitemap: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error ingesting sitemap: {str(e)}")


# Function to parse sitemap and extract URLs
def parse_sitemap(sitemap_url: str) -> List[str]:
    """Parse sitemap.xml and extract URLs to be processed"""
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        root = ET.fromstring(response.content)

        # Handle both regular sitemap and sitemap index
        urls = []
        namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

        # Check if it's a sitemap index
        sitemap_index_namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        sitemap_elements = root.findall('sitemap:sitemap', sitemap_index_namespace)

        if sitemap_elements:
            # This is a sitemap index, need to fetch individual sitemaps
            for sitemap_elem in sitemap_elements:
                loc_elem = sitemap_elem.find('sitemap:loc', sitemap_index_namespace)
                if loc_elem is not None:
                    nested_sitemap_url = loc_elem.text
                    urls.extend(parse_sitemap(nested_sitemap_url))
        else:
            # This is a regular sitemap with URLs
            url_elements = root.findall('sitemap:url', namespace)
            for url_elem in url_elements:
                loc_elem = url_elem.find('sitemap:loc', namespace)
                if loc_elem is not None:
                    urls.append(loc_elem.text)

        return urls
    except Exception as e:
        print(f"Error parsing sitemap {sitemap_url}: {str(e)}")
        return []

async def fetch_and_process_url(url: str) -> str:
    """Fetch content from a URL and return text content"""
    try:
        response = requests.get(url)
        response.raise_for_status()

        # Extract text content from HTML
        from bs4 import BeautifulSoup
        soup = BeautifulSoup(response.text, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Get text content
        text = soup.get_text()

        # Clean up text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        return text
    except Exception as e:
        print(f"Error fetching content from {url}: {str(e)}")
        return ""

async def ingest_sitemap_content(sitemap_url: str, doc_id_prefix: str = "sitemap_") -> Dict:
    """Ingest content from all URLs in a sitemap into Qdrant"""
    print(f"Starting sitemap ingestion from: {sitemap_url}")

    urls = parse_sitemap(sitemap_url)
    print(f"Found {len(urls)} URLs in sitemap")

    total_chunks_ingested = 0
    failed_urls = []

    for i, url in enumerate(urls):
        print(f"Processing URL {i+1}/{len(urls)}: {url}")
        try:
            content = await fetch_and_process_url(url)

            if content.strip():
                # Generate document ID based on URL
                doc_id = f"{doc_id_prefix}{hashlib.md5(url.encode()).hexdigest()}"

                # Split content into chunks
                chunks = text_splitter.split_text(content)

                points = []
                for j, chunk in enumerate(chunks):
                    # Use integer ID instead of string ID to avoid format issues
                    point_id = int(hashlib.md5(f"{doc_id}_{j}".encode()).hexdigest()[:8], 16) % (2**31)  # Ensure it fits in int32

                    # Generate a vector representation based on the content
                    vector = get_embeddings(chunk)

                    points.append(
                        models.PointStruct(
                            id=point_id,
                            vector=vector,
                            payload={
                                "content": chunk,
                                "document_id": doc_id,
                                "chunk_index": j,
                                "source_url": url,
                                "source_type": "webpage"
                            }
                        )
                    )

                # Upload chunks to Qdrant
                if points:
                    qdrant_client.upsert(
                        collection_name=DOCUMENTATION_COLLECTION,
                        points=points
                    )
                    total_chunks_ingested += len(chunks)
                    print(f"  - Ingested {len(chunks)} chunks from {url}")
            else:
                print(f"  - No content found for {url}")
        except Exception as e:
            print(f"  - Error processing {url}: {str(e)}")
            failed_urls.append(url)

    print(f"Sitemap ingestion completed! Total chunks ingested: {total_chunks_ingested}, Failed URLs: {len(failed_urls)}")
    return {
        "sitemap_url": sitemap_url,
        "urls_processed": len(urls),
        "chunks_ingested": total_chunks_ingested,
        "failed_urls": failed_urls
    }

# Ingestion CLI function
def run_ingestion():
    """Run the documentation ingestion process"""
    print("Starting documentation ingestion process...")

    # Use configurable documentation directory
    docs_dir = Path("./docs")
    print(f"Looking for documentation in: {docs_dir.absolute()}")

    if not docs_dir.exists():
        print(f"Documentation directory '{docs_dir}' does not exist.")
        # Create the directory if it doesn't exist
        docs_dir.mkdir(parents=True, exist_ok=True)
        print(f"Created documentation directory: {docs_dir.absolute()}")

        # Create a sample documentation file
        sample_doc = docs_dir / "sample_documentation.txt"
        sample_doc.write_text("""# Sample Documentation
This is sample documentation for testing the RAG functionality.

## Features
- Feature 1: Description of feature 1
- Feature 2: Description of feature 2
- Feature 3: Description of feature 3

## Usage
To use this system, follow these steps:
1. Step 1
2. Step 2
3. Step 3

## API Reference
The API provides the following endpoints:
- GET /api/data - Retrieve data
- POST /api/data - Create new data
""")
        print(f"Created sample documentation at {sample_doc}")

    # Find all text-based documentation files
    doc_extensions = ['.txt', '.md', '.rst', '.py', '.js', '.ts', '.html', '.css']
    doc_files = []
    for ext in doc_extensions:
        doc_files.extend(docs_dir.rglob(f"*{ext}"))

    print(f"Found {len(doc_files)} documentation files to ingest")

    for doc_file in doc_files:
        print(f"Ingesting: {doc_file}")
        try:
            # Create asyncio event loop to run the async ingestion function
            import asyncio
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            result = loop.run_until_complete(ingest_documentation(str(doc_file)))
            print(f"  - Ingested {result['chunks_ingested']} chunks")
        except Exception as e:
            print(f"  - Error ingesting {doc_file}: {str(e)}")

    print("Ingestion process completed!")
    return {"status": "completed", "files_processed": len(doc_files)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Gemini RAG Chatbot Backend')
    parser.add_argument('--ingest', action='store_true', help='Run file-based ingestion process')
    parser.add_argument('--ingest-sitemap', type=str, help='URL of sitemap.xml to ingest')

    args = parser.parse_args()

    if args.ingest:
        run_ingestion()
    elif args.ingest_sitemap:
        print(f"Starting sitemap ingestion from: {args.ingest_sitemap}")
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        result = loop.run_until_complete(ingest_sitemap_content(args.ingest_sitemap))
        print(f"Sitemap ingestion completed: {result}")
    else:
        import uvicorn
        uvicorn.run(app, host="0.0.0.0", port=8000)