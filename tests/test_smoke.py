import robotsim


def test_version_is_string():
    assert isinstance(robotsim.__version__, str)


def test_build_info_is_release_64bit():
    info = robotsim.build_info()
    assert info.startswith("robotsim")
    assert "64-bit" in info
    assert "Release" in info