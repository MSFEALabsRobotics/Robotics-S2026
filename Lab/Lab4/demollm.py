#pip install google-genai pillow
from google import genai
from google.genai import types

# Initialize the client with the key that worked for you
client = genai.Client(api_key="AIzaSyDIwzFbYcypt7lZ3tpEVNk76ID9IvdTe1g")

# Use the same model ID that was working
MODEL_ID = "gemini-robotics-er-1.5-preview"

# Text-only prompt
PROMPT = "What is the capital of France?"

try:
    response = client.models.generate_content(
        model=MODEL_ID,
        contents=[PROMPT],  # Just send the text, no image
        config=types.GenerateContentConfig(
            temperature=0.5,
            # thinking_config=types.ThinkingConfig(thinking_budget=0) # You can likely remove this too for simple text
        )
    )
    print(response.text)

except Exception as e:
    print(f"Error: {e}")
