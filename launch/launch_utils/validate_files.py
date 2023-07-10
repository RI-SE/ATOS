from pathlib import Path
from .generate_cert import cert_gen
from ament_index_python.packages import get_package_prefix
import os
import shutil
import rclpy.logging as logging


def validate_certs(atos_dir):
    # If certs path does not exist, create it
    certs_dir = atos_dir / Path("certs")
    if not os.path.exists(certs_dir):
        os.makedirs(certs_dir)

    # If certs do not exist, create them
    cert = certs_dir / Path("selfsigned.crt")
    key = certs_dir / Path("selfsigned.key")
    if not os.path.exists(cert) or not os.path.exists(key):
        cert_gen(KEY_FILE=key, CERT_FILE=cert)
    return [cert, key]


def validate_params(atos_dir):
    params_path = Path("conf") / Path("params.yaml")
    atos_params_path = atos_dir / params_path
    if not os.path.exists(atos_params_path):
        shutil.copy(get_package_prefix('atos') /
                    Path("etc") / Path("params.yaml"), atos_dir)
        logging.get_logger('launch').info(
            "Missing params.yaml, Copied default to " + str(atos_dir / params_path)) 
    return atos_params_path


def validate_openx_files(atos_dir):
    openx_dirs = ["odr", "osc", "Catalogs"]
    for openx_dir in openx_dirs:
        atos_openx_dir = atos_dir / Path(openx_dir)
        if not os.path.exists(atos_openx_dir):
            shutil.copytree(get_package_prefix('atos') /
                            Path("etc") / Path(openx_dir), atos_openx_dir)
            logging.get_logger('launch').info(
                "Missing " + openx_dir + " folder, Copied default to " + str(atos_openx_dir))


def validate_files():
    atos_dir = os.path.join(os.path.expanduser('~'), '.astazero', 'ATOS')
    params = validate_params(atos_dir)
    cert, key = validate_certs(atos_dir)
    validate_openx_files(atos_dir)
    return {
        "params": params,
        "cert": cert,
        "key": key
    }
