from rclpy.exceptions import InvalidHandle
import pkg_resources

"""These patches fix unhandled exceptions in rclpy humble. Without this patch,
dynamic subscriptions, client, timers etc might throw an InvalidHandle exception when they are
destroyed. This race condition was reported in the
following issue:
    * https://github.com/ros2/rclpy/issues/1142

This workaround here is based on the fix in the 'rolling' branch of rclpy.
    * https://github.com/ros2/rclpy/pull/1150
    * https://github.com/ros2/rclpy/commit/159ced49bb904511154d9f6e595b64e1f5c0d8c0

"""


def _take_subscription(sub):

    try:
        with sub.handle:
            msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
            if msg_info is not None:
                return msg_info[0]
    except InvalidHandle:
        # Subscription is a Destroyable, which means that on __enter__ it can throw an
        # InvalidHandle exception if the entity has already been destroyed.  Handle that here
        # by just returning an empty argument, which means we will skip doing any real work
        # in handler below
        pass
    return None


def _take_timer(tmr):
    try:
        with tmr.handle:
            tmr.handle.call_timer()
    except InvalidHandle:
        # Timer is a Destroyable, which means that on __enter__ it can throw an
        # InvalidHandle exception if the entity has already been destroyed.  Handle that here
        # by just returning an empty argument, which means we will skip doing any real work
        # in handler below
        pass
    return None


def _take_client(client):
    try:
        with client.handle:
            return client.handle.take_response(client.srv_type.Response)
    except InvalidHandle:
        # Client is a Destroyable, which means that on __enter__ it can throw an
        # InvalidHandle exception if the entity has already been destroyed.  Handle that here
        # by just returning an empty argument, which means we will skip doing any real work
        # in handler below
        pass
    return None


def _take_service(srv):
    try:
        with srv.handle:
            request_and_header = srv.handle.service_take_request(
                srv.srv_type.Request)
        return request_and_header
    except InvalidHandle:
        # Service is a Destroyable, which means that on __enter__ it can throw an
        # InvalidHandle exception if the entity has already been destroyed.  Handle that here
        # by just returning an empty argument, which means we will skip doing any real work
        # in handler below
        pass
    return None


def patch_executor(executor):
    """Patch the executor to avoid InvalidHandle exceptions when destroying
    entities."""
    # Make sure this patch is only applied to the exact version of rclpy that we expect.
    rclpy_version_major, rclpy_version_minor, rclpy_version_patch = pkg_resources.get_distribution(
        'rclpy').version.split(".")
    if not (rclpy_version_major == "3" and rclpy_version_minor == "3"):
        raise RuntimeError(
            "The patch_executor function is only guaranteed compatible with rclpy version 3.3.x")
    executor._take_subscription = _take_subscription
    executor._take_timer = _take_timer
    executor._take_client = _take_client
    executor._take_service = _take_service

    return executor
