from robot_console.recording import compute_guided_capture_duration_sec


def test_compute_guided_capture_duration_uses_override() -> None:
    duration = compute_guided_capture_duration_sec(
        target_images=200,
        extraction_fps=2.0,
        duration_override_sec=45.0,
        safety_buffer_sec=10.0,
    )
    assert duration == 45.0


def test_compute_guided_capture_duration_uses_target_images_and_buffer() -> None:
    duration = compute_guided_capture_duration_sec(
        target_images=120,
        extraction_fps=2.0,
        duration_override_sec=0.0,
        safety_buffer_sec=8.0,
    )
    assert duration == 68.0


def test_compute_guided_capture_duration_rejects_invalid_inputs() -> None:
    try:
        compute_guided_capture_duration_sec(
            target_images=0,
            extraction_fps=2.0,
            duration_override_sec=0.0,
            safety_buffer_sec=0.0,
        )
        raise AssertionError("expected ValueError for target_images <= 0")
    except ValueError:
        pass

    try:
        compute_guided_capture_duration_sec(
            target_images=120,
            extraction_fps=0.0,
            duration_override_sec=0.0,
            safety_buffer_sec=0.0,
        )
        raise AssertionError("expected ValueError for extraction_fps <= 0")
    except ValueError:
        pass
