import os
import re
import sys
import platform
import subprocess

from setuptools import Extension, find_packages
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion
from distutils.core import setup

__CMAKE_PREFIX_PATH__ = None
__ENVIRONMENT_PATH__ = None
__DEBUG__ = False

if "--CMAKE_PREFIX_PATH" in sys.argv:
    index = sys.argv.index('--CMAKE_PREFIX_PATH')
    __CMAKE_PREFIX_PATH__ = sys.argv[index+1]
    sys.argv.remove("--CMAKE_PREFIX_PATH")
    sys.argv.remove(__CMAKE_PREFIX_PATH__)

if "--Debug" in sys.argv:
    index = sys.argv.index('--Debug')
    sys.argv.remove("--Debug")
    __DEBUG__ = True

if "--env" in sys.argv:
    index = sys.argv.index('--env')
    environment = sys.argv[index+1]
    if environment == "anymal":
        __ENVIRONMENT_PATH__ = os.path.dirname(os.path.realpath(__file__)) + "/raisim_gym/env/env/ANYmal"
    elif environment == 'laikago':
        __ENVIRONMENT_PATH__ = os.path.dirname(os.path.realpath(__file__)) + "/raisim_gym/env/env/laikago"
    elif environment == 'quadrotor':
        __ENVIRONMENT_PATH__ = os.path.dirname(os.path.realpath(__file__)) + "/raisim_gym/env/env/hummingbird"
    else:
        __ENVIRONMENT_PATH__ = environment

    sys.argv.remove("--env")
    sys.argv.remove(environment)


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        if __CMAKE_PREFIX_PATH__ is not None:
            cmake_args.append('-DCMAKE_PREFIX_PATH=' + __CMAKE_PREFIX_PATH__)

        cmake_args.append('-DRSG_ENVIRONMENT_INCLUDE_PATH=' + __ENVIRONMENT_PATH__)

        cfg = 'Debug' if __DEBUG__ else 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)

setup(
    name='raisim_gym',
    version='0.5.0',
    author='Jemin Hwangbo',
    license="MIT",
    packages=find_packages(),
    author_email='jemin.hwangbo@gmail.com',
    description='gym for raisim.',
    long_description='',
    ext_modules=[CMakeExtension('_raisim_gym')],
    install_requires=['gym==0.10.9', 'ruamel.yaml', 'numpy', 'stable_baselines==2.8'],
    cmdclass=dict(build_ext=CMakeBuild),
    include_package_data=True,
    zip_safe=False,
)
