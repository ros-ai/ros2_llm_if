from typing import Dict
import roslibpy


def get_available_services(client: roslibpy.Ros) -> Dict:
    """Create a dictionary of current services.

    Args:
        client (roslibpy.Ros): ROS client.

    Returns:
        Dict: Dictionary of services.
    """
    services = client.get_services(None)
    services = {
        service: roslibpy.Service(client, service, client.get_service_type(service))
        for service in services
    }
    return services
