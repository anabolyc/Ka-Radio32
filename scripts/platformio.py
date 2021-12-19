Import("env", "projenv")

# access to global build environment
# print(env)

# access to project build environment (is used source files in "src" folder)
# print(projenv)

# Dump build environment (for debug purpose)
# print(env.Dump())

#
# (Optional) Do not run extra script when IDE fetches C/C++ project metadata
#
from SCons.Script import COMMAND_LINE_TARGETS

if "idedata" in COMMAND_LINE_TARGETS:
    env.Exit(0)

#
# Custom actions when building program/firmware
#

def after_build(source, target, env):
    print("after_build")
    pioenv = env.get('PIOENV')
    # pioframework = env.get('PIOFRAMEWORK')[0]
    # ~/.platformio/packages/framework-{}/components/nvs_flash/nvs_partition_generator generates empty bin
    env.Execute('python ./scripts/nvs_partition_gen.py --version v1 --input ./boards/{}.csv --output ./boards/build/{}.bin --size 0x3000'
        .format(pioenv, pioenv))

env.AddPostAction("buildprog", after_build)

def before_upload(source, target, env):
    print("before_upload")
    pioenv = env.get('PIOENV')
    pio_pkg_dir = env.get('PROJECT_PACKAGES_DIR')
    env.Execute('{}/tool-esptoolpy/esptool.py --chip esp32 write_flash 0x3a2000 boards/build/{}.bin'
        .format(pio_pkg_dir, pioenv))

env.AddPreAction("upload", before_upload)