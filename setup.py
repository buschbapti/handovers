#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup()
d['packages'] = ['reba_optim','reba_optim.tools']
d['package_dir'] = {'': 'src'}
setup(**d)
