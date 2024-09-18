import xml.etree.ElementTree as ET
from pathlib import Path

from setuptools import setup, find_packages

package_xml = ET.parse('package.xml').getroot()
package_name = package_xml.find('name').text
data = Path('share') / package_name
setup(
    name=package_name,
    version=package_xml.find('version').text,
    packages=find_packages(),
    maintainer=package_xml.find('license').text,
    maintainer_email=package_xml.find('maintainer').attrib['email'],
    license=package_xml.find('license').text,
    data_files=[
        (str(data), ['package.xml']),
        (str(data / 'launch'), ['launch/gui.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'gui = atos_gui.main:main'
        ],
    },
)