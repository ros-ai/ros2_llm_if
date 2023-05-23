from typing import Any
import roslibpy


def create_services_from_api(client: roslibpy.Ros, api: Any) -> Any:
    """Create a dictionary of services from the API.

    Args:
        client (roslibpy.Ros): ROS client.
        api (Any): API JSON object.

    Returns:
        Any: Dictionary of services.
    """
    services = {}
    for service in api:
        services[service["name"]] = roslibpy.Service(
            client, service["name"], service["service_type"]
        )
    return services
