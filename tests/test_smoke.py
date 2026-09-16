import robotsim


def test_add():
    assert robotsim.add(2, 3) == 5


def test_version_is_string():
    assert isinstance(robotsim.__version__, str)