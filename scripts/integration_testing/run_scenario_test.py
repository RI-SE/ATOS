import os
import signal
import pytest
import launch
import launch_pytest
from launch_pytest.tools import process as process_tools
import re
import psutil

def kill_process_by_name(name, signal):
    for proc in psutil.process_iter():
        if proc.name() == name:
            os.kill(proc.pid, signal)

@pytest.fixture
def integration_test_proc():
    # Launch a process to test
    return launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'atos', 'launch_integration_testing.py'],
        shell=True,
        cached_output=True,
        output='screen'
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
        assert any(re.search('Scenario Execution result: OK', line) for line in output.splitlines()), 'Scenario test failed'
    process_tools.assert_output_sync(
        launch_context, integration_test_proc, validate_output, timeout=3000)
    yield
    # this is executed after launch service shutdown
    kill_process_by_name("ros2", signal.SIGINT) # TODO: This is a hack to kill the process, need to find a better way
    