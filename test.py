import json
import asyncio
import websockets
import openai

api = None
with open("api.json", "r") as f:
    api = json.load(f)

# content = f"Assume following JSON API: {str(api)}. Move the turtle to (4, 4, 0), then to (4, 8, 0). No comments."
content = f"\
    Assume following JSON API: {str(api)}.\
    Move the turtle left by 2, then rotate 180 degrees, and move back to (5, 5).\
    Finally, spawn a turtle named turtle2 at (10, 10) and kill turtle1.\
    Respond as a plain JSON list. Do not add comments.\
"

openai.api_key = "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"

model = "gpt-3.5-turbo"

response = openai.ChatCompletion.create(
    model=model,
    messages=[
        {
            "role": "user",
            "content": content,
        },
    ],
    temperature=0.7,
)

results = ""
for choice in response.choices:
    results += choice.message.content

results = results.replace("'", '"')
results = json.loads(results)

# ClientFactory
import roslibpy


client = roslibpy.Ros(host="localhost", port=9090)
client.run()

services = {}

for service in api:
    services[service["name"]] = roslibpy.Service(
        client, service["name"], service["service_type"]
    )

for result in results:
    print("Calling service {} with args {}".format(result["name"], result["args"]))
    input("Press Enter to continue...")
    service = services[result["name"]]
    request = roslibpy.ServiceRequest(result["args"])
    result = service.call(request)
