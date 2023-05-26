import argparse
import json
from glob import glob

import roslibpy
from prompt import OpenAIInterface, append_service


def args_factory() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--key", type=str, required=True, help="OpenAI API key.")
    parser.add_argument(
        "--api", type=str, default="turtlesim_msgs/srv", help="Path to API JSON file."
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

    # load the API
    api = []
    for api_file in glob(f"{args.api}/*.json"):
        with open(api_file, "r") as f:
            api.append(json.load(f))

    # configure your interface
    interface = OpenAIInterface(api=api, key=args.key)

    # create a ROS client
    ros_client = roslibpy.Ros(host=args.host, port=args.port)
    ros_client.run()
    services = {}

    while True:
        try:
            prompt = input("Enter a prompt: ")
            # turn prompt into API calls
            print("Generating API calls. This may take some time...")
            generated_api_calls = interface.prompt_to_api_calls(
                prompt, model=args.model
            )
            print("Done.")

            for call in generated_api_calls:
                # get required service (in case they changed)
                print("Getting required service. This might take some time...")
                services = append_service(ros_client, call["service"], services)
                print("Done.")

                try:
                    print(
                        "Calling service {} with args {}".format(
                            call["service"], call["args"]
                        )
                    )
                    input("Press Enter to continue...")
                    service = services[call["service"]]
                    request = roslibpy.ServiceRequest(call["args"])
                    service.call(request)
                except Exception as e:
                    print(f"Failed to call service with {e}.")
        except KeyboardInterrupt:
            break

    ros_client.terminate()


if __name__ == "__main__":
    main()
