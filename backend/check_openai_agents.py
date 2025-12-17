"""
Test script to check the openai_agents package structure.
"""
try:
    import openai_agents
    print("openai_agents module loaded successfully")
    print("Available attributes:", dir(openai_agents))

    # Try to import specific classes
    from openai_agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
    print("Successfully imported Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI")

except ImportError as e:
    print(f"Import error: {e}")

    # Try alternative import paths
    try:
        from openai_agents.agent import Agent
        from openai_agents.runner import Runner
        from openai_agents.models import OpenAIChatCompletionsModel
        from openai_agents.client import AsyncOpenAI
        print("Successfully imported from submodules")
    except ImportError as e2:
        print(f"Alternative import error: {e2}")

        # Check what's in the package
        import openai_agents
        print(f"Package location: {openai_agents.__file__}")
        print(f"Package version: {getattr(openai_agents, '__version__', 'unknown')}")