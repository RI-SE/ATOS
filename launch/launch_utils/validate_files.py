from pathlib import Path
from .generate_cert import cert_gen
import os

def validate_certs(atos_dir):
    # If certs path does not exist, create it
    certs_dir = atos_dir / Path("certs")
    if not os.path.exists(certs_dir):
        os.makedirs(certs_dir)

    # If certs do not exist, create them
    cert = certs_dir / Path("selfsigned.crt")
    key = certs_dir / Path("selfsigned.key")
    if not os.path.exists(cert) or not os.path.exists(key):
        cert_gen(commonName="ATOS", emailAddress="info@astazero.com"
        ,countryName="SE", localityName="Gothenburg", stateOrProvinceName="Gothenburg"
        ,organizationName="AstaZero", organizationUnitName="ATOS", serialNumber=0
        ,validityStartInSeconds=0, validityEndInSeconds=10*365*24*60*60,
        KEY_FILE = key, CERT_FILE=cert)
    return [cert, key]

def validate_params():
    #todo..
    pass


def validate_files():
    atos_dir = os.path.join(os.path.expanduser('~'), '.astazero', 'ATOS')
    params = os.path.join(atos_dir, 'conf', 'params.yaml')
    cert,key = validate_certs(atos_dir)
    return {
        "params": params,
        "cert": cert,
        "key": key
    }
