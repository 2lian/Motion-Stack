import pytest
from motion_stack.core.logger import setup_logger

@pytest.fixture(scope="session", autouse=True)
def configure_logging():
    """Configure logging once for all tests."""
    setup_logger(debug_path="ms_test.log.jsonl")
