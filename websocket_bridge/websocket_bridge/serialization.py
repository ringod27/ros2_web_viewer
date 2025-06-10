import array
import base64
import numpy as np
import compression

def ros2dict(msg):
    if type(msg) in (str, bool, int, float):
        return msg

    if type(msg) is tuple:
        return list(msg)

    if type(msg) is bytes:
        return base64.b64encode(msg).decode()

    output = {}

    if hasattr(msg, "get_fields_and_field_types"): # ROS2
        fields_and_field_types = msg.get_fields_and_field_types()
    elif hasattr(msg, "__slots__"): # ROS1
        fields_and_field_types = msg.__slots__
    else:
        raise ValueError("ros2dict: Does not appear to be a simple type or a ROS message: %s" % str(msg))

    for field in fields_and_field_types:
        # PointCloud2: extract only necessary fields, reduce precision
        if (msg.__module__ == "sensor_msgs.msg._PointCloud2" or \
            msg.__module__ == "sensor_msgs.msg._point_cloud2") \
            and field == "data" and len(msg.data) != 0:
            compression.compress_point_cloud2(msg, output)
            continue

        value = getattr(msg, field)
        if type(value) in (str, bool, int, float):
            output[field] = value

        elif type(value) is bytes:
            output[field] = base64.b64encode(value).decode()

        elif type(value) is tuple:
            output[field] = list(value)

        elif type(value) is list:
            output[field] = [ros2dict(el) for el in value]

        elif type(value) in (np.ndarray, array.array):
            output[field] = value.tolist()

        else:
            output[field] = ros2dict(value)

    return output

# if __name__ == "__main__":
#     # Run unit tests
#     print("str")
#     print(ros2dict("test"))
#     print("Path")
#     from nav_msgs.msg import Path
#     print(ros2dict(Path()))
#     print("NavSatFix")
#     from sensor_msgs.msg import NavSatFix
#     print(ros2dict(NavSatFix()))
#     print("Int32MultiArray")
#     from std_msgs.msg import Int32MultiArray
#     print(ros2dict(Int32MultiArray()))
#     print("object (this should not work)")
#     try:
#         print(ros2dict(object()))
#     except ValueError:
#         print("exception successfully caught")
#     print("all tests completed successfully")