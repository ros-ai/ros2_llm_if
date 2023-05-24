import json
from typing import Any, Dict
import openai


def prompt_to_api_calls(
    prompt: str,
    api: Any,
    model: str = "gpt-3.5-turbo",
    temperature: float = 0.7,
) -> Dict:
    """Turns prompt into API calls.

    Args:
        prompt (str): Prompt.
        api (Any): API. The large language model is conditioned on generating a response that follows the API.
        model (str, optional): OpenAI model. Defaults to "gpt-3.5-turbo".
        temperature (float, optional): OpenAI temperature. Defaults to 0.7.

    Returns:
        Dict: API calls.
    """
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
                ",
            },
            {"role": "user", "content": prompt},
        ],
        temperature=temperature,
    )

    results = ""
    for choice in response.choices:
        results += choice.message.content

    results = results.replace("'", '"')
    results = json.loads(results)
    return results
