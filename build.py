# The build system in PlatformIO is based on SCon
import os
import subprocess


# create SCon environment
env = Environment()

# remember current working directory
cwd = os.getcwd()

dsdlc = os.path.join(cwd, 'libuavcan', 'dsdl_compiler', 'libuavcan_dsdlc')

# Update submodules dsdl, pyuavcan in order to be able to build headers
# with the dsdlc
print("Update git submodules...")
subprocess.call(['git', 'submodule', 'update', '--init', '--recursive'])

# Collect all paths that contain .uavcan files
dirs = list()
dirs.append(os.path.join(cwd, 'dsdl', 'uavcan'))
dirs.append(os.path.join(cwd, 'phoenix'))

print("Running DSDL Compiler")
# Run the DSDL compiler on all previously defined directories and put the
# output into the libuavcan/include directory
subprocess.call( ['python', dsdlc] + dirs + ['-Olibuavcan/include'])
