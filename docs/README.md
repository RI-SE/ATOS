## Software architecture

![image](res/ATOS_architecture.drawio.png)


## Docs

Docs generated with mkdocs/doxygen and hosted on Read The Docs

When editing locally, install deps with 
"sudo apt install doxygen && pip install -r docs/requirements.txt"

Then build the doxygen html pages with "doxygen Doxyfile" from the ATOS root folder.

Then run "mkdocs serve" from the ATOS root folder.

When adding python deps, add these to requirements.in and then run the "pip-compile" from pip-tools https://pip-tools.readthedocs.io/en/latest/
