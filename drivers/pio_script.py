# Copyright 2014-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
STM32Cube HAL

STM32Cube embedded software libraries, including:
The HAL hardware abstraction layer, enabling portability between different STM32 devices via standardized API calls
The Low-Layer (LL) APIs, a light-weight, optimized, expert oriented set of APIs designed for both performance and runtime efficiency.

http://www.st.com/en/embedded-software/stm32cube-embedded-software.html?querycriteria=productId=LN1897
"""

from glob import glob
from os.path import basename, isfile, join
from shutil import copy
import sys

Import('env')
platform = env.PioPlatform()

MCU_FAMILY = env.BoardConfig().get("build.mcu")[0:7]

def get_startup_file(mcu):
    search_path = join(env["PROJECT_DIR"], "startup_stm32f103xe.[sS]")
    startup_file = glob(search_path)
    return basename(startup_file[0])


def get_linker_script(mcu):
    search_path = join(env["PROJECT_DIR"], "stm32f103rctx_flash.ld")
    linker_script = glob(search_path)
    return linker_script[0]

def generate_hal_config_file(mcu):
    config_path = join(env["PROJECT_DIR"], "drivers",
                       "stm32f1xx_hal_driver", "inc")

    if isfile(join(config_path, MCU_FAMILY + "xx_hal_conf.h")):
        return

    copy(join(config_path, MCU_FAMILY + "xx_hal_conf_template.h"),
         join(config_path, MCU_FAMILY + "xx_hal_conf.h"))


env.Replace(
    AS="$CC",
    ASCOM="$ASPPCOM",
    LDSCRIPT_PATH=get_linker_script(env.BoardConfig().get("build.mcu"))
)

env.Replace(
    ASFLAGS=["-x", "assembler-with-cpp"],

    CCFLAGS=[
        "-Os",  # optimize for size
        "-ffunction-sections",  # place each function in its own section
        "-fdata-sections",
        "-Wall",
        "-mthumb",
        "-mcpu=%s" % env.BoardConfig().get("build.cpu"),
        "-nostdlib"
    ],

    CPPDEFINES=[
        "USE_HAL_DRIVER",
        ("F_CPU", "$BOARD_F_CPU")
    ],

    CXXFLAGS=[
        "-fno-rtti",
        "-fno-exceptions"
    ],

    LINKFLAGS=[
        "-Os",
        "-Wl,--gc-sections,--relax",
        "-mthumb",
        "-mcpu=%s" % env.BoardConfig().get("build.cpu"),
        "--specs=nano.specs",
        "--specs=nosys.specs"
    ],

    LIBS=["c", "gcc", "m", "stdc++", "nosys"]
)

# copy CCFLAGS to ASFLAGS (-x assembler-with-cpp mode)
env.Append(ASFLAGS=env.get("CCFLAGS", [])[:])

cpp_flags = env.Flatten(env.get("CPPDEFINES", []))

env.Append(CPPDEFINES=["-DSTM32F103xE"])

env.Append(
    CPPPATH=[
        join(env["PROJECT_DIR"], "drivers", "cmsis", "inc"),
        join(env["PROJECT_DIR"], "drivers", "cmsis", "device",
             "st", "stm32f1xx", "inc"),
        join(env["PROJECT_DIR"], "drivers", "stm32f1xx_hal_driver", "inc")
    ],

    LIBPATH=[
        join(env["PROJECT_DIR"], "drivers", "cmsis", "lib", "gcc"),
        join(env["PROJECT_DIR"])
    ]
)

#
# Generate framework specific files
#

generate_hal_config_file(env.BoardConfig().get("build.mcu"))

#
# Target: Build HAL Library
#

libs = []


libs.append(env.BuildLibrary(
    join("$BUILD_DIR", "FrameworkHALDriver"),
    join(env["PROJECT_DIR"], "drivers", "stm32f1xx_hal_driver"),
    src_filter="-<*> +<src/stm32f1xx_hal_flash.c> +<src/stm32f1xx_hal_pwr.c> \
    +<src/stm32f1xx_hal_rcc.c> +<src/stm32f1xx_hal_tim.c> +<src/stm32f1xx_hal_tim_ex.c> \
    +<src/stm32f1xx_hal_gpio_ex.c> +<src/stm32f1xx_hal_adc_ex.c> +<src/stm32f1xx_hal_cortex.c> \
    +<src/stm32f1xx_hal_flash_ex.c> +<src/stm32f1xx_hal_gpio.c> +<src/stm32f1xx_hal_rcc_ex.c> \
    +<src/stm32f1xx_hal.c> +<src/stm32f1xx_hal_adc.c> +<src/stm32f1xx_hal_uart.c> \
    +<src/stm32f1xx_hal_i2c.c> +<src/stm32f1xx_hal_iwdg.c> +<src/stm32f1xx_hal_dma.c>"
))

libs.append(env.BuildLibrary(
    join("$BUILD_DIR", "FrameworkCMSISDevice"),
    join(env["PROJECT_DIR"], "drivers", "cmsis", "device", "st",
         "stm32f1xx", "src", "templates"),
    src_filter="-<*> +<*.c> +<gcc/%s>" % get_startup_file(
        env.BoardConfig().get("build.mcu"))
))

env.Append(LIBS=libs)