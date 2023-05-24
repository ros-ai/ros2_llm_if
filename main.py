import argparse
import openai
import json
import roslibpy

from prompt import create_services_from_api, call_openai_model


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--key", type=str, required=True, help="OpenAI API key.")
    parser.add_argument(
        "--prompt",
        type=str,
        default="\
            Move the turtle left by 2, then rotate 180 degrees, and move back to (5, 5).\
            Finally, spawn a turtle named turtle2 at (10, 10) and kill turtle1.\
        ",
        help="Prompt.",
    )
    parser.add_argument(
        "--api", type=str, default="api.json", help="Path to API JSON file."
    )
    parser.add_argument("--host", type=str, default="localhost", help="ROS host.")
    parser.add_argument("--port", type=int, default=9090, help="ROS port.")
    parser.add_argument(
        "--model", type=str, default="gpt-3.5-turbo", help="OpenAI model."
    )
    args = parser.parse_args()

    # configure your key
    openai.api_key = args.key

    # load the API
    api = None
    with open(args.api, "r") as f:
        api = json.load(f)

    # create a ROS client
    ros_client = roslibpy.Ros(host=args.host, port=args.port)
    ros_client.run()

    # turn prompt into API calls
    print("Generating API calls. This may take some time...")
    generated_api_calls = call_openai_model(args.prompt, api, model=args.model)
    print("Done.")

    # create services from the API
    services = create_services_from_api(ros_client, api)

    for call in generated_api_calls:
        print(
            "Calling service {} of type {} with args {}".format(
                call["name"], call["service_type"], call["args"]
            )
        )
        input("Press Enter to continue...")

        try:
            service = services[call["name"]]
            request = roslibpy.ServiceRequest(call["args"])
            service.call(request)
        except Exception as e:
            print(f"Failed to call service with {e}")


if __name__ == "__main__":
    main()
