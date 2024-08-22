from pathlib import Path
import pytest


@pytest.fixture()
def get_file_path():
    def _(file_path: str):
        """
        Returns the absolute file path.

        Args:
            file_path (str): The relative file path.

        Returns:
            str: The absolute file path.
        """
        return (Path(__file__).parent / file_path).absolute()

    return _
