import json
from pathlib import Path
import pytest
from massage_path_tool.io.session import load_session, make_empty_session, save_session

def test_defaults():
    s = make_empty_session()
    assert s['calibration_aspect_ratio'] == 1.0
    assert s['discrete_points'] == []
    assert s['paths'] == []

def test_roundtrip(tmp_path):
    s = make_empty_session(aspect_ratio=1.4, source_image='reference.png')
    s['discrete_points'].append({'id':'p1','label':'jian_jing','u':0.25,'v':0.15})
    s['paths'].append({'id':'path1','label':'spine','points':[{'u':0.5,'v':0.2}]})
    p = tmp_path / 'session.json'
    save_session(s, p)
    loaded = load_session(p)
    assert loaded['calibration_aspect_ratio'] == pytest.approx(1.4)
    assert loaded['discrete_points'][0]['id'] == 'p1'
    assert loaded['paths'][0]['label'] == 'spine'

def test_save_creates_dirs(tmp_path):
    p = tmp_path / 'deep' / 'nested' / 'session.json'
    save_session(make_empty_session(), p)
    assert p.exists()

def test_missing_key_raises(tmp_path):
    p = tmp_path / 'bad.json'
    p.write_text(json.dumps({'paths': []}))
    with pytest.raises(ValueError, match='missing required keys'):
        load_session(p)

def test_file_not_found():
    with pytest.raises(FileNotFoundError):
        load_session('/nonexistent/session.json')

def test_bad_discrete_points_type(tmp_path):
    p = tmp_path / 'bad2.json'
    p.write_text(json.dumps({'calibration_aspect_ratio':1.0,'discrete_points':'bad','paths':[]}))
    with pytest.raises(ValueError):
        load_session(p)

def test_schema_version_is_2():
    from massage_path_tool.io.session import make_empty_session
    assert make_empty_session()['schema_version'] == 2

def test_load_v1_session_accepted():
    import json, tempfile, os
    from massage_path_tool.io.session import load_session
    v1 = {
        'schema_version': 1,
        'calibration_aspect_ratio': 1.4,
        'discrete_points': [{'id': 'p1', 'label': 'p1', 'u': 0.5, 'v': 0.5}],
        'paths': [{'id': 'path1', 'label': 'path1',
                   'points': [{'u': 0.1, 'v': 0.2}]}],
    }
    with tempfile.NamedTemporaryFile(suffix='.json', mode='w', delete=False) as f:
        json.dump(v1, f); name = f.name
    try:
        s = load_session(name)
        assert 'x_stitch' not in s['discrete_points'][0]
        assert 'x_stitch' not in s['paths'][0]['points'][0]
    finally:
        os.unlink(name)

def test_save_load_v2_roundtrip():
    import tempfile, os
    from massage_path_tool.io.session import make_empty_session, save_session, load_session
    s = make_empty_session(source_image='/tmp/img.png')
    s['discrete_points'].append({
        'id': 'p1', 'label': 'p1',
        'u': 0.3, 'v': 0.4,
        'x_stitch': 120.0, 'y_stitch': 200.0,
    })
    s['paths'].append({'id': 'path1', 'label': 'path1', 'points': [
        {'u': 0.1, 'v': 0.2, 'x_stitch': 50.0, 'y_stitch': 80.0},
    ]})
    with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as f:
        name = f.name
    try:
        save_session(s, name)
        s2 = load_session(name)
        assert s2['schema_version'] == 2
        assert s2['discrete_points'][0]['x_stitch'] == 120.0
        assert s2['paths'][0]['points'][0]['y_stitch'] == 80.0
    finally:
        os.unlink(name)
