from __future__ import annotations
import json
from pathlib import Path

SCHEMA_VERSION = 2

def make_empty_session(aspect_ratio=1.0, source_image=''):
    return {
        'schema_version': SCHEMA_VERSION,
        'source_image': source_image,
        'calibration_aspect_ratio': float(aspect_ratio),
        'anchor_landmarks': {},
        'discrete_points': [],
        'paths': []
    }

def save_session(session, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    session['schema_version'] = SCHEMA_VERSION
    with open(path, 'w') as f:
        json.dump(session, f, indent=2)

def load_session(path):
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f'Session file not found: {path}')
    with open(path) as f:
        data = json.load(f)
    _validate(data)
    return data

def _validate(data):
    required = {'calibration_aspect_ratio', 'discrete_points', 'paths'}
    missing = required - data.keys()
    if missing:
        raise ValueError(f'Session JSON missing required keys: {missing}')
    if not isinstance(data['discrete_points'], list):
        raise ValueError("'discrete_points' must be a list")
    if not isinstance(data['paths'], list):
        raise ValueError("'paths' must be a list")
