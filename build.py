# The build system in PlatformIO is based on SCon
import os
import subprocess
import shutil

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

print("Copying sources so PlatformIO will find them")
def copydir(src, dest):
    # print("Copying everything from " + str(src) + " to " + str(dest))
    try:
        for root, dirs, files in os.walk(src):
            for f in files:
                # shutil.copy will overwrite
                shutil.copy(os.path.join(root, f), dest)
            for d in dirs:
                # shutil.copytree will not overwrite, remove first
                if(os.path.exists(os.path.join(dest, d))):
                    shutil.rmtree(os.path.join(dest, d))
                shutil.copytree(os.path.join(root, d), os.path.join(dest, d))
            # dont dig deeper
            del dirs[:]
    except err:
        print("Some files/directories could not be copied: ", str(err))


copydir(os.path.join(cwd, 'libuavcan', 'src'), os.path.join(cwd, 'src'))
copydir(os.path.join(cwd, 'libuavcan_drivers', 'nxpk20', 'driver', 'src'), os.path.join(cwd, 'src'))

print("Done setting up build environment")
