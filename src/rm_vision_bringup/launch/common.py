import os
import yaml
from ament_index_python.packages import get_package_share_directory

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'),
    'config',
    'launch_params.yaml'
), 'r', encoding='utf-8'))

ns = launch_params.get('namespace', '')

def resolve_node_params_block(raw_nodes: dict, node_name: str, namespace: str = '') -> dict:
    candidate_keys = []
    if namespace:
        candidate_keys.append(f'/{namespace}/{node_name}')
    candidate_keys.append(f'/{node_name}')
    candidate_keys.append(f'/**/{node_name}')

    for candidate_key in candidate_keys:
        if candidate_key in raw_nodes:
            return raw_nodes[candidate_key]

    for key, value in raw_nodes.items():
        if key.endswith(f'/{node_name}'):
            return value

    raise KeyError(f'Cannot find parameter block for node: {node_name}')

# 参数文件路径
node_params_path = os.path.join(
    get_package_share_directory('rm_vision_bringup'),
    'config',
    'node_params.yaml'
)

# 加载并处理参数
with open(node_params_path, 'r', encoding='utf-8') as f:
    raw_params = yaml.safe_load(f)

def build_node_params(raw_nodes: dict, namespace: str) -> dict:
    node_params = {}
    for key, value in raw_nodes.items():
        clean_key = key.replace('/**/', '')
        new_key = f'/{clean_key}' if not namespace else f'/{namespace}/{clean_key}'

        # 特殊处理 rm_auto_record 的 topic_name
        if namespace and clean_key == 'rm_auto_record' and 'ros__parameters' in value:
            if 'topic_name' in value['ros__parameters']:
                topics = value['ros__parameters']['topic_name']
                updated_topics = []
                for topic in topics:
                    if not topic.startswith('/'):
                        updated_topics.append(f'/{namespace}/{topic}')
                    elif not topic.startswith(f'/{namespace}/'):
                        updated_topics.append(f'/{namespace}{topic}')
                    else:
                        updated_topics.append(topic)
                value['ros__parameters']['topic_name'] = updated_topics

        node_params[new_key] = value

    return node_params

node_params = build_node_params(raw_params, ns)


