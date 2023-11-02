import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join(  # Need to modify the sys.path since we launch from the ros2 installed path
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))
import rclpy
import signal
import pytest
import launch
from launch import LaunchDescription
import launch_pytest
from launch_pytest.tools import process as process_tools
import re

@pytest.fixture
def integration_test_proc():
    # Launch a process to test
    return launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'atos', 'launch_integration_testing.py'],
        shell=True,
        cached_output=True,
    )

# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description(integration_test_proc):
    """Launch a simple process to print 'hello_world'."""
    return launch.LaunchDescription([
        integration_test_proc,
        launch_pytest.actions.ReadyToTest()
    ])

@pytest.mark.launch(fixture=launch_description)
def test_read_stdout(integration_test_proc, launch_context):
    def validate_output(output):
        # this function can use assertions to validate the output or return a boolean.
        # pytest generates easier to understand failures when assertions are used.
        assert any(re.search('Scenario Execution result: NOK', line) for line in output.splitlines()), 'Scenario test failed'
    process_tools.assert_output_sync(
        launch_context, integration_test_proc, validate_output, timeout=30)
    yield
    # this is executed after launch service shutdown
    assert False

@pytest.fixture(scope='session', autouse=True)
def term_handler():
    orig = signal.signal(signal.SIGTERM, signal.getsignal(signal.SIGINT))
    yield
    signal.signal(signal.SIGTERM, orig)
    