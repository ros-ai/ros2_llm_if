import json
from typing import Any, Dict
import openai


class OpenAIInterface:
    def __init__(self, api: Any, key: str) -> None:
        openai.api_key = key
        self.api_ = api
        self.system_prompt_ = f"\
            Use the following API to achieve the user's goals:\n\
            {str(api)}\n\
            Return the response as a JSON object with the format specified in the API.\n\
            Do not include explanations or conversation in the response.\n\
        "
        self.chat_history_ = []

    def prompt_to_api_calls(
        self,
        prompt: str,
        model: str = "gpt-3.5-turbo",
        temperature: float = 0.7,
    ) -> Dict:
        """Turns prompt into API calls.

        Args:
            prompt (str): Prompt.
            model (str, optional): OpenAI model. Defaults to "gpt-3.5-turbo".
            temperature (float, optional): OpenAI temperature. Defaults to 0.7.

        Returns:
            Dict: API calls.
        """
        self.chat_history_.append(  # prompt taken from https://github.com/Significant-Gravitas/Auto-GPT/blob/master/autogpt/prompts/default_prompts.py
            {
                "role": "user",
                "content": f"\
                    {prompt}\n\
                    Respond only with the output in the exact format specified in the system prompt, with no explanation or conversation.\
                ",
            }
        )

        response = openai.ChatCompletion.create(
            model=model,
            messages=[{"role": "system", "content": self.system_prompt_}]
            + self.chat_history_,
            temperature=temperature,
            n=1,
        )

        self.chat_history_.append(
            {"role": "assistant", "content": response.choices[0].message.content}
        )

        content = self.chat_history_[-1]["content"]
        print(f"Got response:\n{content}")
        return json.loads(content)
