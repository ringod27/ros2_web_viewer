import base64
import io
import numpy as np

_PCL2_DATATYPES_NUMPY_MAP = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}

def decode_pcl2(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a sensor_msgs.PointCloud2 message.

    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: empty list)
    :return: numpy.recarray with values for each point.
    """

    assert cloud.point_step * cloud.width * cloud.height == len(cloud.data), \
        'length of data does not match point_step * width * height'

    all_field_names = []
    np_struct = []
    total_used_bytes = 0
    for field in cloud.fields:
        all_field_names.append(field.name)
        assert field.datatype in _PCL2_DATATYPES_NUMPY_MAP, \
            'invalid datatype %d specified for field %s' % (field.datatype, field.name)
        field_np_datatype = _PCL2_DATATYPES_NUMPY_MAP[field.datatype]
        np_struct.append((field.name, field_np_datatype))
        total_used_bytes += np.nbytes[field_np_datatype]

    assert cloud.point_step >= total_used_bytes, \
        'error: total byte sizes of fields exceeds point_step'

    if cloud.point_step > total_used_bytes:
        np_struct.append(('unused_bytes', np.uint8, cloud.point_step - total_used_bytes))

    points = np.frombuffer(cloud.data, dtype=np_struct).view(dtype=np.recarray)

    if skip_nans:
        nan_indexes = None
        for field_name in all_field_names:
            if nan_indexes is None:
                nan_indexes = np.isnan(points[field_name])
            else:
                nan_indexes = nan_indexes | np.isnan(points[field_name])

        points = points[~nan_indexes]

    if uvs:
        fetch_indexes = [(v * cloud.width + u) for u, v in uvs]
        points = points[fetch_indexes]

    # if endianness between cloud and system doesn't match then byteswap everything
    if cloud.is_bigendian == np.little_endian:
        points = points.byteswap()

    if field_names is None:
        return points
    else:
        return points[list(field_names)]


def compress_point_cloud2(msg, output):
    # assuming fields are ('x', 'y', 'z', ...),
    # compression scheme is:
    # msg['_data_uint16'] = {
    #   bounds: [ xmin, xmax, ymin, ymax, zmin, zmax, ... ]
    #   points: string: base64 encoded bytearray of struct { uint16 x_frac; uint16 y_frac; uint16 z_frac;}
    # }
    # where x_frac = 0 maps to xmin and x_frac = 65535 maps to xmax
    # i.e. we are encoding all the floats as uint16 values where 0 represents the min value in the entire dataset and
    # 65535 represents the max value in the dataset, and bounds: [...] holds information on those bounds so the
    # client can decode back to a float

    output["data"] = []
    output["__comp"] = ["data"]

    field_names = [field.name for field in msg.fields]

    if "x" not in field_names or "y" not in field_names:
        output["_error"] = "PointCloud2 error: must contain at least 'x' and 'y' fields for visualization"
        return

    if "z" in field_names:
        decode_fields = ("x", "y", "z")
    else:
        decode_fields = ("x", "y")
    
    try:
        points = decode_pcl2(msg, field_names = decode_fields, skip_nans = True)
    except AssertionError as e:
        output["_error"] = "PointCloud2 error: %s" % str(e)
    
    if points.size > 65536:
        output["_warn"] = "Point cloud too large, randomly subsampling to 65536 points."
        idx = np.random.randint(points.size, size=65536)
        points = points[idx]

    xpoints = points['x'].astype(np.float32)
    xmax = np.max(xpoints)
    xmin = np.min(xpoints)
    if xmax - xmin < 1.0:
        xmax = xmin + 1.0
    xpoints_uint16 = (65535 * (xpoints - xmin) / (xmax - xmin)).astype(np.uint16)

    ypoints = points['y'].astype(np.float32)
    ymax = np.max(ypoints)
    ymin = np.min(ypoints)
    if ymax - ymin < 1.0:
        ymax = ymin + 1.0
    ypoints_uint16 = (65535 * (ypoints - ymin) / (ymax - ymin)).astype(np.uint16)
    
    if "z" in field_names:
        zpoints = points['z'].astype(np.float32)
        zmax = np.max(zpoints)
        zmin = np.min(zpoints)
        if zmax - zmin < 1.0:
            zmax = zmin + 1.0
        zpoints_uint16 = (65535 * (zpoints - zmin) / (zmax - zmin)).astype(np.uint16)
    else:
        zmax = 1.0
        zmin = 0.0
        zpoints_uint16 = ypoints_uint16 * 0
    
    bounds_uint16 = [xmin, xmax, ymin, ymax, zmin, zmax]
    if np.little_endian:
        points_uint16 = np.stack((xpoints_uint16, ypoints_uint16, zpoints_uint16),1).ravel().view(dtype=np.uint8)
    else:
        points_uint16 = np.stack((xpoints_uint16, ypoints_uint16, zpoints_uint16),1).ravel().byteswap().view(dtype=np.uint8)

    output["_data_uint16"] = {
        "type": "xyz",
        "bounds": list(map(float, bounds_uint16)),
        "points": base64.b64encode(points_uint16).decode(),
    }