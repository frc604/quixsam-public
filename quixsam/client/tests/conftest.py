import pytest

from networktables import NetworkTables


@pytest.fixture(scope="function")
def nt(request):
    """Starts/stops global networktables instance for testing"""
    NetworkTables.startTestMode(server=request)

    yield NetworkTables
    NetworkTables.shutdown()
