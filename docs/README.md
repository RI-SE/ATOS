## Software architecture
![image](https://github.com/RI-SE/ATOS/assets/97448034/b0eb4128-163f-42e5-88e4-4818d0c8ea87)


## Docs

Docs generated with mkdocs/doxygen and hosted on Read The Docs

When editing locally, install deps with 
"sudo apt install doxygen && pip install -r requirements.txt"

Then build the doxygen html pages with "doxygen Doxyfile" from the ATOS root folder.

Then run "mkdocs serve" from the ATOS root folder.

When adding python deps, add these to requirements.in and then run the "pip-compile" from pip-tools https://pip-tools.readthedocs.io/en/latest/
