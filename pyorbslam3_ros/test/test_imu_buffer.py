import threading
import time

import numpy as np

from pyorbslam3_ros.imu_buffer import ImuBuffer


def row(t):
    return (t, 0.1, 0.2, 0.3, 0.0, 0.0, 9.81)


def test_window_bounds_exclusive_inclusive():
    b = ImuBuffer()
    for t in (0.0, 0.1, 0.2, 0.3, 0.4):
        b.add(row(t))
    w = b.take_window(0.1, 0.3)
    np.testing.assert_allclose(w[:, 0], [0.2, 0.3])
    assert w.shape == (2, 7)
    # 0.0 and 0.1 were consumed (discarded), 0.4 stays
    assert b.span() == (0.4, 0.4)


def test_first_window_keeps_everything_up_to_frame():
    b = ImuBuffer()
    for t in (0.0, 0.1, 0.2):
        b.add(row(t))
    np.testing.assert_allclose(b.take_window(None, 0.15)[:, 0], [0.0, 0.1])


def test_empty_window():
    b = ImuBuffer()
    assert b.take_window(None, 1.0).shape == (0, 7)


def test_out_of_order_dropped():
    b = ImuBuffer()
    assert b.add(row(1.0))
    assert not b.add(row(0.5))
    assert not b.add(row(1.0))
    assert b.dropped_out_of_order == 2 and len(b) == 1


def test_trimming_records_discarded_time():
    b = ImuBuffer(max_duration=1.0)
    for i in range(30):
        b.add(row(i * 0.1))
    oldest, newest = b.span()
    assert newest - oldest <= 1.0 + 1e-9
    assert b.trimmed_until is not None and b.trimmed_until < oldest
    b.clear()
    assert len(b) == 0 and b.trimmed_until is None


def test_wait_for_times_out_and_wakes():
    b = ImuBuffer()
    t0 = time.monotonic()
    assert not b.wait_for(1.0, timeout=0.05)
    assert time.monotonic() - t0 >= 0.04

    def producer():
        time.sleep(0.05)
        b.add(row(1.0))

    threading.Thread(target=producer).start()
    assert b.wait_for(1.0, timeout=2.0)


def test_concurrent_add_and_take():
    b = ImuBuffer(max_duration=1e9)
    n = 5000
    taken = []

    def producer():
        for i in range(n):
            b.add(row(float(i)))

    th = threading.Thread(target=producer)
    th.start()
    last = None
    while th.is_alive() or len(b):
        w = b.take_window(last, float(n))
        if w.shape[0]:
            taken.extend(w[:, 0])
            last = w[-1, 0]
    th.join()
    assert taken == [float(i) for i in range(n)]
