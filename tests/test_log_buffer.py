import pytest

from planner.services.log_buffer import LogBuffer


def test_append_and_read_from_cursor():
    logs = LogBuffer(capacity=10)
    logs.append("first")
    logs.append("second", "WARNING")
    batch = logs.read(0)
    assert [item.message for item in batch.items] == ["first", "second"]
    assert [item.level for item in batch.items] == ["INFO", "WARNING"]
    assert batch.next_cursor == 2
    assert not batch.truncated

    rest = logs.read(1)
    assert [item.message for item in rest.items] == ["second"]
    assert rest.next_cursor == 2


def test_empty_read_returns_current_sequence():
    logs = LogBuffer(capacity=5)
    logs.append("one")
    logs.read(1)
    empty = logs.read(1)
    assert empty.items == ()
    assert empty.next_cursor == 1


def test_ring_buffer_reports_truncation():
    logs = LogBuffer(capacity=2)
    logs.append("one")
    logs.append("two")
    logs.append("three")
    assert [item.message for item in logs.read(0).items] == ["two", "three"]
    # The reader already consumed ids 1-2, so nothing was lost.
    assert not logs.read(2).truncated

    logs.append("four")
    assert [item.message for item in logs.read(0).items]
    # Id 2 has been dropped since the reader stopped at id 1.
    truncated = logs.read(1)
    assert [item.message for item in truncated.items] == ["three", "four"]
    assert truncated.truncated
    assert not logs.read(2).truncated


def test_invalid_inputs_are_rejected():
    with pytest.raises(ValueError):
        LogBuffer(capacity=0)
    with pytest.raises(ValueError):
        LogBuffer().read(-1)