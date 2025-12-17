"""
Test script to verify the complete RAG flow from query to response.
"""
import asyncio
import os
from dotenv import load_dotenv

from utils.embeddings import get_embedding_service
from services.qdrant_service import get_qdrant_service
from services.context_injector import get_context_injector
from agent.rag_agent import get_rag_agent
from api.models import QueryRequest

# Load environment variables
load_dotenv()

def test_embedding_generation():
    """Test embedding generation for a sample query."""
    print("Testing embedding generation...")
    try:
        embedding_service = get_embedding_service()
        query = "What is ROS 2?"
        embedding = embedding_service.generate_embedding(query)
        print(f"SUCCESS: Generated embedding with {len(embedding)} dimensions")
        return True
    except Exception as e:
        print(f"ERROR: Error generating embedding: {str(e)}")
        return False

def test_qdrant_retrieval():
    """Test Qdrant retrieval with a sample embedding."""
    print("\nTesting Qdrant retrieval...")
    try:
        qdrant_service = get_qdrant_service()
        # Generate a test embedding
        embedding_service = get_embedding_service()
        query = "What is ROS 2?"
        query_embedding = embedding_service.generate_embedding(query)

        # Perform search
        results = qdrant_service.search(query_vector=query_embedding, top_k=3, threshold=0.5)
        print(f"SUCCESS: Retrieved {len(results)} documents from Qdrant")
        if results:
            print(f"  First result similarity: {results[0]['similarity_score']:.3f}")
        return True
    except Exception as e:
        print(f"ERROR: Error retrieving from Qdrant: {str(e)}")
        return False

def test_context_injection():
    """Test context injection with sample documents."""
    print("\nTesting context injection...")
    try:
        context_injector = get_context_injector()
        sample_docs = [
            {
                "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.",
                "source_url": "https://docs.ros.org/en/rolling/Concepts.html",
                "title": "ROS 2 Concepts",
                "similarity_score": 0.85
            },
            {
                "content": "ROS 2 provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior.",
                "source_url": "https://docs.ros.org/en/rolling/Overview.html",
                "title": "ROS 2 Overview",
                "similarity_score": 0.78
            }
        ]
        query = "What is ROS 2?"
        context = context_injector.format_context_for_agent(sample_docs, query)
        print(f"SUCCESS: Formatted context with {len(context)} characters")
        return True
    except Exception as e:
        print(f"ERROR: Error in context injection: {str(e)}")
        return False

def test_agent_query():
    """Test agent query with sample context."""
    print("\nTesting agent query...")
    try:
        rag_agent = get_rag_agent()
        sample_docs = [
            {
                "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.",
                "source_url": "https://docs.ros.org/en/rolling/Concepts.html",
                "title": "ROS 2 Concepts"
            }
        ]
        query = "What is ROS 2?"
        response = rag_agent.query(query, sample_docs)
        print(f"SUCCESS: Agent responded with {len(response)} characters")
        print(f"  Response preview: {response[:100]}...")
        return True
    except Exception as e:
        print(f"ERROR: Error in agent query: {str(e)}")
        return False

def test_complete_flow():
    """Test the complete RAG flow."""
    print("\nTesting complete RAG flow...")
    try:
        # Get all services
        embedding_service = get_embedding_service()
        qdrant_service = get_qdrant_service()
        context_injector = get_context_injector()
        rag_agent = get_rag_agent()

        # Sample query
        query = "What is ROS 2?"

        # Generate embedding
        query_embedding = embedding_service.generate_embedding(query)

        # Retrieve documents
        retrieved_docs = qdrant_service.search(
            query_vector=query_embedding,
            top_k=3,
            threshold=0.5
        )

        # Format context
        formatted_context = context_injector.format_context_for_agent(retrieved_docs, query)

        # Get agent response
        agent_response = rag_agent.query(query, retrieved_docs)

        print(f"SUCCESS: Complete RAG flow successful!")
        print(f"  Query: {query}")
        print(f"  Retrieved {len(retrieved_docs)} documents")
        print(f"  Agent response length: {len(agent_response)} characters")

        return True
    except Exception as e:
        print(f"ERROR: Error in complete RAG flow: {str(e)}")
        return False

async def test_async_api_flow():
    """Test the async API flow similar to what the FastAPI endpoint does."""
    print("\nTesting async API flow...")
    try:
        from api.router import query_endpoint
        from api.models import QueryRequest

        request = QueryRequest(query="What is ROS 2?", top_k=3, threshold=0.5)
        response = await query_endpoint(request)

        print(f"SUCCESS: Async API flow successful!")
        print(f"  Answer length: {len(response.answer)} characters")
        print(f"  Retrieved documents: {len(response.retrieved_documents) if response.retrieved_documents else 0}")
        print(f"  Processing time: {response.processing_time:.3f}s")

        return True
    except Exception as e:
        print(f"ERROR: Error in async API flow: {str(e)}")
        return False

def main():
    """Run all tests."""
    print("Starting RAG System Tests...\n")

    tests = [
        test_embedding_generation,
        test_qdrant_retrieval,
        test_context_injection,
        test_agent_query,
        test_complete_flow,
    ]

    results = []
    for test in tests:
        results.append(test())

    # Run async test separately
    print("\n" + "="*50)
    print("Running async API test...")
    async_results = [asyncio.run(test_async_api_flow())]

    all_results = results + async_results
    passed = sum(all_results)
    total = len(all_results)

    print(f"\n{'='*50}")
    print(f"Test Results: {passed}/{total} tests passed")

    if passed == total:
        print("SUCCESS: All tests passed! The RAG system is working correctly.")
    else:
        print("FAILURE: Some tests failed. Please check the implementation.")

if __name__ == "__main__":
    main()