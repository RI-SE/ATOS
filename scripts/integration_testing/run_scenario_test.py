import os
import signal
import pytest
import launch
import launch_pytest
from launch_pytest.tools import process as process_tools
import re
import psutil

def kill_process_by_name(name, signal):
    """Kill process by its name.

    Args:
        name (str): Name of process to kill, e.g. 'ros2'
        signal (Signals): Which signal to send to the process, e.g. signal.SIGINT
    """
    for proc in psutil.process_iter():
        if proc.name() == name:
            os.kill(proc.pid, signal)


@pytest.fixture
def integration_test_proc():
    """Launch the integration test process.

    Returns:
        ExecuteProcess: The process to execute
    """
    # Launch a process to test
    return launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'atos', 'launch_integration_testing.py'],
        shell=True,
        cached_output=True,
        output='screen'
    )


@launch_pytest.fixture
def launch_description(integration_test_proc):
    """This function specifies the processes to be run for our test.

    Args:
        integration_test_proc (Any): Integration test process

    Yields:
        LaunchDescription: Launch description for our test
    """
    yield launch.LaunchDescription([
        integration_test_proc,
        launch_pytest.actions.ReadyToTest()
    ])
    kill_process_by_name("ros2", signal.SIGINT) # TODO: Is there a better way to do this?


@pytest.mark.launch(fixture=launch_description)
def test_scenario_execution(integration_test_proc, launch_context):
    """Test the scenario execution. Checks for if all states are followed and if it follows the trajectory.

    Args:
        integration_test_proc (Any): Integration test process
        launch_context (Any): Launch context
    """
    def validate_scenario_execution(output):
        # this function can use assertions to validate the output or return a boolean.
        # pytest generates easier to understand failures when assertions are used.
        assert any(re.search('State change result: OK', line) for line in output.splitlines()), 'State change test failed'
        assert any(re.search('Trajectory following result: OK', line) for line in output.splitlines()), 'Trajectory check test failed'
    process_tools.assert_output_sync(
        launch_context, integration_test_proc, validate_scenario_execution, timeout=30)