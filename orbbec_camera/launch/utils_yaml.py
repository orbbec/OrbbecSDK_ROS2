import os
import yaml
import logging
from ament_index_python.packages import get_package_share_directory

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def load_yaml(file_path):
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        logger.error(f"File {file_path} not found.")
        return {}
    except yaml.YAMLError as e:
        logger.error(f"Error parsing YAML file {file_path}: {e}")
        return {}


def parse_yaml(data, prefix='', params=None):
    if params is None:
        params = {}
    if isinstance(data, dict):
        for key, value in data.items():
            new_prefix = f'{key}.' if prefix else key + '.'
            params = parse_yaml(value, prefix=new_prefix, params=params)
    elif isinstance(data, list):
        for index, item in enumerate(data):
            params = parse_yaml(item, prefix=f'[{index}].', params=params)
    else:
        params[prefix[:-1]] = data
        #print(f'{prefix}: {data}')
    return params


def load_and_parse_yaml(file_path):
    data = load_yaml(file_path)
    return parse_yaml(data) if data else {}


def update_params(default_params, yaml_params):
    default_params.update(yaml_params)
    return default_params


def find_param(params, key, default_value=None):
    if isinstance(params, dict):
        return params.get(key, default_value)
    elif isinstance(params, list):
        for param_dict in params:
            if isinstance(param_dict, dict) and key in param_dict:
                return param_dict[key]
    return default_value


def get_absolute_path(config_file_path, package_name):
    if not os.path.isabs(config_file_path):
        logger.info(f"not Absolute path: {config_file_path}")
        return os.path.join(get_package_share_directory(package_name), 'config', config_file_path)
    logger.info(f"Absolute path: {config_file_path}")
    return config_file_path


def convert_value(value):
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            pass
        try:
            return float(value)
        except ValueError:
            pass
        if value.lower() == 'true':
            return True
        elif value.lower() == 'false':
            return False
    return value

def print_params(default_params):
    print(f"\n----------------------------------------")
    for key, value in default_params.items():
        print(f"Key: {key}, Value: {value}")
    print(f"----------------------------------------\n")