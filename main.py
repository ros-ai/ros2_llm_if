import argparse
import openai
import json
import roslibpy

from prompt import get_service, prompt_to_api_calls


def args_factory() -> argparse.Namespace:
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
    return args


def main() -> None:
    args = args_factory()

    # configure your key
    openai.api_key = args.key

    # load the API
    api = None
    with open(args.api, "r") as f:
        api = json.load(f)

    # create a ROS client
    ros_client = roslibpy.Ros(host=args.host, port=args.port)
    ros_client.run()
    services = {}

    # turn prompt into API calls
    print("Generating API calls. This may take some time...")
    generated_api_calls = prompt_to_api_calls(args.prompt, api, model=args.model)
    print("Done.")

    for call in generated_api_calls:
        # get required service (in case they changed)
        print("Getting required service. This might take some time...")
        services = get_service(ros_client, call["name"], services)
        print("Done.")

        try:
            print(
                "Calling service {} of type {} with args {}".format(
                    call["name"], call["service_type"], call["args"]
                )
            )
            input("Press Enter to continue...")
            service = services[call["name"]]
            request = roslibpy.ServiceRequest(call["args"])
            service.call(request)
        except Exception as e:
            print(f"Failed to call service with {e}")

    ros_client.terminate()


if __name__ == "__main__":
    main()
