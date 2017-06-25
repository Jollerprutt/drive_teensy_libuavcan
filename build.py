# The build system in PlatformIO is based on SCon
import os
import subprocess


# create SCon environment
env = Environment()

# remember current working directory
cwd = os.getcwd()

dsdlc = os.path.join(cwd, 'libuavcan', 'dsdl_compiler', 'libuavcan_dsdlc')

print("Update git submodules...")
subprocess.call(['git', 'submodule', 'update', '--init', '--recursive'])

# print("Installing DSDL Compiler")
# os.chdir(os.path.join(cwd, 'libuavcan', 'dsdl_compiler'))
# subprocess.call(['python', 'setup.py', 'install', '--record', 'installed_files.log', '-q'])
# subprocess.call(['python', 'setup.py', 'build', '-q'])
# os.chdir(cwd)

# print("Finding *.uavcan files")
# files = list()
# for root, dirs, filenames in os.walk('dsdl' + os.path.sep + 'uavcan'):
#     for f in filenames:
#         if f.endswith('.uavcan'):
#             filepath = os.path.join(root, f)
#             print(filepath)
#             files.append(os.path.join(cwd, filepath))
dirs = list()
dirs.append(os.path.join(cwd, 'dsdl', 'uavcan'))
dirs.append(os.path.join(cwd, 'phoenix'))

print("Running DSDL Compiler")
# Run the DSDL compiler on all previously defined directories and put the
# output into the libuavcan/include directory
subprocess.call( ['python', dsdlc] + dirs + ['-Olibuavcan/include'])
