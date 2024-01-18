from distutils.core import setup, Extension
iso22133 = Extension('_iso22133',
   sources=['iso22133.i','iso22133.c','positioning.c'],
)
positioning = Extension('_positioning',
   sources=['positioning.i','positioning.c'],
)

setup (name = 'iso22133',
   version = '0.1',
   author = "AstaZero",
   description = """iso22133 implementation wrapper""",
   ext_modules = [iso22133, positioning],
   py_modules = ["iso22133", "positioning"],
)
