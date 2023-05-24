import json
from typing import Any, Dict
import openai


def call_openai_model(
    prompt: str,
    api: Any,
    model: str = "gpt-3.5-turbo",
    temperature: float = 0.7,
) -> Dict:
    response = openai.ChatCompletion.create(
        model=model,
        messages=[
            {
                "role": "system",
                "content": f"\
                    Assume the following JSON API: {str(api)}.\
                    Respond as a plain JSON list.\
                    Do not add comments.\
                    Do not add explanations.\
                "
            },
            {
                "role": "user",
                "content": prompt
            },
        ],
        temperature=temperature,
    )

    results = ""
    for choice in response.choices:
        results += choice.message.content

    results = results.replace("'", '"')
    results = json.loads(results)
    return results
