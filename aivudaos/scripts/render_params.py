"""Translate the AivudaOS sensor settings to the native ROS node parameter names."""
import math
from pathlib import Path
from tempfile import NamedTemporaryFile

from jsonschema import Draft7Validator
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'
GROUPS = ('common', 'preprocess', 'mapping', 'publish', 'px4_bridge')
FLAT_KEYS = {'point_filter_num', 'filter_size_surf', 'filter_size_map', 'max_iteration'}


def read_yaml(path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def ros_value(value, schema):
    # ROS declares these as doubles even if a user enters a whole number in the UI.
    if schema['type'] == 'number':
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('ROS parameter must be finite')
    if schema['type'] == 'array':
        return [ros_value(item, schema['items']) for item in value]
    return value


def write_yaml(path, data):
    with NamedTemporaryFile('w', encoding='utf-8', dir=path.parent, delete=False) as out:
        pending = Path(out.name)
        try:
            yaml.safe_dump(data, out, sort_keys=False)
            out.flush()
            pending.replace(path)
        finally:
            pending.unlink(missing_ok=True)


def render(config_path, runtime, lio_template, bridge_template):
    active = read_yaml(config_path)
    if not isinstance(active, dict):
        raise ValueError(f'Invalid app configuration in {config_path}')
    defaults = read_yaml(CONFIG / 'default_config.yaml')
    properties = read_yaml(CONFIG / 'config_schema.yaml')['properties']
    lio_doc = read_yaml(lio_template)
    bridge_doc = read_yaml(bridge_template)
    lio = lio_doc['/**']['ros__parameters']
    bridge = bridge_doc['fastlio_to_px4_visual_odometry']['ros__parameters']
    for group in GROUPS:
        values = dict(defaults[group])
        values.update(active.get(group, {}))
        schema = properties[group]
        errors = list(Draft7Validator(schema).iter_errors(values))
        if errors:
            raise ValueError(f'{group}.{errors[0].json_path}: {errors[0].message}')
        for key, value in values.items():
            target = bridge if group == 'px4_bridge' else lio if key in FLAT_KEYS else lio[group]
            target[key] = ros_value(value, schema['properties'][key])
    runtime.mkdir(parents=True, exist_ok=True)
    lio_path = runtime / 'fastlio_params.yaml'
    bridge_path = runtime / 'bridge_params.yaml'
    write_yaml(lio_path, lio_doc)
    write_yaml(bridge_path, bridge_doc)
    return lio_path, bridge_path
