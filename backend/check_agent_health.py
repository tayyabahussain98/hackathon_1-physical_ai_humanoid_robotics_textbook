"""
Script to check the health of the RAG agent specifically.
"""
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def check_agent_health():
    """Check the health of the RAG agent."""
    print("Checking RAG Agent Health...\n")

    try:
        from agent.rag_agent import RAGAgent
        print("SUCCESS: Successfully imported RAGAgent")

        # Create an agent instance
        agent = RAGAgent()
        print("SUCCESS: Successfully created RAGAgent instance")

        # Check health
        print("\nRunning health check...")
        is_healthy = agent.health_check()

        if is_healthy:
            print("SUCCESS: Agent health check: PASSED")
            print("SUCCESS: The agent is healthy and can connect to Gemini API")
        else:
            print("ERROR: Agent health check: FAILED")
            print("ERROR: The agent cannot connect to Gemini API or is experiencing issues")

        # Try a simple test query to verify functionality
        print("\nTesting agent query functionality...")
        try:
            sample_docs = [
                {
                    "content": "This is a test document for the RAG system.",
                    "source_url": "test://sample-document",
                    "title": "Test Document"
                }
            ]
            response = agent.query("What is this system about?", sample_docs)
            print(f"SUCCESS: Agent query successful: {len(response)} characters returned")
            print(f"  Response preview: {response[:100]}...")
        except Exception as e:
            print(f"ERROR: Agent query failed: {str(e)}")

        return is_healthy

    except Exception as e:
        print(f"ERROR: Error checking agent health: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def check_environment():
    """Check if all required environment variables are set."""
    print("\nChecking environment variables...\n")

    required_vars = [
        'GEMINI_API_KEY',
        'GEMINI_MODEL_NAME',
        'QDRANT_URL',
        'QDRANT_API_KEY',
        'COHERE_API_KEY'
    ]

    all_set = True
    for var in required_vars:
        value = os.getenv(var)
        if value:
            print(f"SUCCESS: {var}: SET ({value[:10]}...)")
        else:
            print(f"ERROR: {var}: NOT SET")
            all_set = False

    return all_set

if __name__ == "__main__":
    print("RAG Agent Health Check Tool\n")
    print("="*50)

    env_ok = check_environment()
    agent_healthy = check_agent_health()

    print("\n" + "="*50)
    print("SUMMARY:")
    print(f"Environment Variables: {'OK' if env_ok else 'MISSING'}")
    print(f"Agent Health: {'HEALTHY' if agent_healthy else 'UNHEALTHY'}")

    if env_ok and agent_healthy:
        print("\n🎉 All systems are ready! The agent should be healthy.")
    else:
        print("\n⚠️  Issues detected. Please check the output above.")

    if not agent_healthy:
        print("\nTroubleshooting agent health issues:")
        print("1. Verify GEMINI_API_KEY has proper permissions in Google Cloud Console")
        print("2. Ensure the Gemini API is enabled for your project")
        print("3. Check that the GEMINI_MODEL_NAME is valid (e.g., gemini-2.5-flash)")
        print("4. Verify internet connectivity to Google's API servers")