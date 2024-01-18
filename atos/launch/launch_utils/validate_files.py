from pathlib import Path
from .generate_cert import cert_gen
from ament_index_python.packages import get_package_prefix
import os
import shutil
import rclpy.logging as logging


def validate_directory(directory_path, copy_from_path=None):
    if not os.path.exists(directory_path):
        if copy_from_path:
            shutil.copytree(copy_from_path, directory_path)
            logging.get_logger('launch').info(
                f"Missing directory {directory_path}, copied default from {copy_from_path}")
        else:
            os.makedirs(directory_path)
            logging.get_logger('launch').info(
                f"Missing directory {directory_path}, created it")
    return directory_path


def validate_file(file_path, copy_from_path=None):
    if not os.path.exists(file_path):
        if copy_from_path:
            shutil.copy(copy_from_path, file_path, follow_symlinks=True)
            logging.get_logger('launch').info(
                f"Missing file {file_path}, copied default from {copy_from_path}")
        else:
            with open(file_path, 'a'):
                os.utime(file_path, None)
                logging.get_logger('launch').info(
                    f"Missing file {file_path}, created it")
    return file_path

def validate_certs(directory_path):
    validate_directory(directory_path)

    # If certs do not exist, create them
    cert = directory_path / Path("selfsigned.crt")
    key = directory_path / Path("selfsigned.key")
    if not os.path.exists(cert) or not os.path.exists(key):
        cert_gen(KEY_FILE=key, CERT_FILE=cert)
    return [cert, key]


def validate_files():
    atos_dir = os.path.join(os.path.expanduser('~'), '.astazero', 'ATOS')
    dirs_to_validate = ["conf", "pointclouds", "logs"]
    for dir_to_validate in dirs_to_validate:
        validate_directory(atos_dir / Path(dir_to_validate))

    # OpenX dirs
    openx_dirs = ["odr", "osc", "Catalogs"]
    for openx_dir in openx_dirs:
        validate_directory(atos_dir / Path(openx_dir), get_package_prefix('atos') / Path("etc") / Path(openx_dir))

    # objects dir
    validate_directory(atos_dir / Path("objects"), get_package_prefix('atos') / Path("etc") / Path("objects"))

    # Certs
    [cert, key] = validate_certs(atos_dir / Path("certs"))

    # Params
    params = validate_file(atos_dir / Path("conf") / Path("params.yaml"), 
                           get_package_prefix('atos') / Path("etc") / Path("conf") / Path("params.yaml"))

    return {
        "params": params,
        "cert": cert,
        "key": key
    }
