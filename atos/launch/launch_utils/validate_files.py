from pathlib import Path
from .generate_cert import cert_gen
from ament_index_python.packages import get_package_prefix
import os
import shutil
import rclpy.logging as logging

def try_create_directory(directory_path):
    if not os.path.exists(directory_path):
        os.makedirs(directory_path),
        logging.get_logger('launch').info(
            f"Missing directory {directory_path}, created")
    else:
        logging.get_logger('launch').debug(
            f"Directory {directory_path} exists, skipping")

def validate_directory_contents(dir,copy_to_path):
    from_files = dir.glob('*')
    for from_file in from_files:
        if from_file.is_dir():
            validate_directory(from_file, copy_to_path / Path(from_file.name))
        elif from_file.is_file():
            validate_file(from_file, copy_to_path / Path(from_file.name))
        else:
            logging.get_logger('launch').info(
                f"File {from_file.name} is not a file or directory, skipping")
    return dir

def validate_directory(copy_from_path, copy_to_path):
    try_create_directory(copy_to_path)
    validate_directory_contents(copy_from_path, copy_to_path)
    return copy_to_path

def validate_file(copy_from_path, copy_to_path):
    if not os.path.exists(copy_to_path):
        shutil.copy(copy_from_path, copy_to_path, follow_symlinks=True)
        logging.get_logger('launch').info(
            f"Missing file {copy_to_path.name}, copied default from {copy_from_path}")
    return copy_to_path

def validate_certs(directory_path):
    try_create_directory(directory_path)

    # If certs do not exist, create them
    cert = directory_path / Path("selfsigned.crt")
    key = directory_path / Path("selfsigned.key")
    if not os.path.exists(cert) or not os.path.exists(key):
        cert_gen(KEY_FILE=key, CERT_FILE=cert)
    return [cert, key]

def validate_atos_dir():
    atos_dir = os.path.join(os.path.expanduser('~'), '.astazero', 'ATOS')
    dirs_to_validate = ["conf", "pointclouds", "logs", "odr", "osc", "Catalogs"]
    for dir_to_validate in dirs_to_validate:
        validate_directory(get_package_prefix('atos') / Path("etc") / Path(dir_to_validate), atos_dir / Path(dir_to_validate))

    # Certs
    [cert, key] = validate_certs(atos_dir / Path("certs"))

    return {
        "params": atos_dir / Path("conf") / Path("params.yaml"),
        "cert": cert,
        "key": key
    }
