import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join(  # Need to modify the sys.path since we launch from the ros2 installed path
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))

import launch
import launch_pytest
from launch_pytest.tools import process as process_tools

import pytest
import launch_integration_testing


# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description(hello_world_proc):
    """Launch a simple process to print 'hello_world'."""
    return launch_integration_testing.generate_launch_description() 
    

@pytest.mark.launch(fixture=launch_description)
def test_read_stdout(hello_world_proc, launch_context):
    """Check if 'hello_world' was found in the stdout."""
    def validate_output(output):
        # this function can use assertions to validate the output or return a boolean.
        # pytest generates easier to understand failures when assertions are used.
        assert output.splitlines() == ['hello_world'], 'process never printed hello_world'
    process_tools.assert_output_sync(
        launch_context, hello_world_proc, validate_output, timeout=5)

    def validate_output(output):
        return output == 'this will never happen'
    assert not process_tools.wait_for_output_sync(
        launch_context, hello_world_proc, validate_output, timeout=0.1)
    yield
    # this is executed after launch service shutdown
    assert hello_world_proc.return_code == 0