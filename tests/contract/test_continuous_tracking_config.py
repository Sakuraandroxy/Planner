import pytest

from config import load_config
from planner.domain.motion import ContinuousTrackingSettings
from planner.errors import ConfigurationError
from tests.fakes.configuration import main_config_data, write_config


def test_nested_tracking_settings_do_not_change_existing_motion_limits(tmp_path):
    raw = main_config_data()
    raw['motion']['continuous']['max_jerk_mps3'] = 2.5
    config = load_config(write_config(tmp_path/'tracking.yaml',raw))
    assert config.motion.continuous.max_jerk_mps3 == 2.5
    assert config.motion.limits.max_acceleration_mps2 == 1.5
    assert config.motion.limits.control_hz == 20
    raw['motion'].pop('continuous')
    assert load_config(write_config(tmp_path/'old.yaml',raw)).motion.continuous == ContinuousTrackingSettings()


@pytest.mark.parametrize('change', [dict(max_jerk_mps3=0),dict(max_yaw_jerk_deg_s3=True),
    dict(lookahead_time_s=float('nan')),dict(min_lookahead_m=2,max_lookahead_m=1),dict(unknown=1)])
def test_invalid_continuous_settings_fail_at_config_load(tmp_path,change):
    raw = main_config_data()
    raw['motion']['continuous'].update(change)
    with pytest.raises(ConfigurationError):
        load_config(write_config(tmp_path/'invalid.yaml',raw))
