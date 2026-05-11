#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-eecs159-h755zi-cm7-builder:bookworm}"
DOCKERFILE_DIR="${DOCKERFILE_DIR:-${REPO_ROOT}/docker/h755zi-cm7}"
BUILD_DIR="${BUILD_DIR:-build/h755zi_cm7_docker}"

if [[ "${REBUILD_IMAGE:-0}" == "1" ]] || ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  docker build -t "${IMAGE}" "${DOCKERFILE_DIR}"
fi

docker run --rm -i \
  -v "${REPO_ROOT}:/work" \
  -w /work \
  "${IMAGE}" \
  bash -s -- "${BUILD_DIR}" <<'CONTAINER_SCRIPT'
set -euo pipefail

BUILD_DIR="$1"

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}/obj"

COMMON_FLAGS=(
  -mcpu=cortex-m7
  -std=gnu11
  -g3
  -DDEBUG
  -DCORE_CM7
  -DUSE_HAL_DRIVER
  -DSTM32H755xx
  -O0
  -ffunction-sections
  -fdata-sections
  -Wall
  -Werror
  -fstack-usage
  -MMD
  -MP
  --specs=nano.specs
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -mthumb
  -Icube_sat_nucleo/CM7/Core/Inc
  -Icube_sat_nucleo/CM7/Application/Algorithms/Inc
  -Icube_sat_nucleo/CM7/Application/Drivers/Inc
  -Icube_sat_nucleo/Drivers/STM32H7xx_HAL_Driver/Inc
  -Icube_sat_nucleo/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
  -Icube_sat_nucleo/Drivers/CMSIS/Device/ST/STM32H7xx/Include
  -Icube_sat_nucleo/Drivers/CMSIS/Include
  -Icube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/include
  -Icube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
  -Icube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

C_SOURCES_FILE="${BUILD_DIR}/c_sources.txt"
{
  find cube_sat_nucleo/CM7/Core/Src \
       cube_sat_nucleo/CM7/Application \
       cube_sat_nucleo/Common/Src \
       cube_sat_nucleo/Drivers/STM32H7xx_HAL_Driver/Src \
       -name '*.c' -type f
  printf '%s\n' \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/list.c \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
    cube_sat_nucleo/CM7/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
} | sort -u > "${C_SOURCES_FILE}"

OBJS=()
while IFS= read -r src; do
  obj="${BUILD_DIR}/obj/${src//\//_}.o"
  arm-none-eabi-gcc "${COMMON_FLAGS[@]}" -c "${src}" -o "${obj}"
  OBJS+=("${obj}")
done < "${C_SOURCES_FILE}"

startup="${BUILD_DIR}/obj/startup_stm32h755xx.o"
arm-none-eabi-gcc "${COMMON_FLAGS[@]}" -x assembler-with-cpp \
  -c cube_sat_nucleo/CM7/Core/Startup/startup_stm32h755xx.s \
  -o "${startup}"
OBJS+=("${startup}")

arm-none-eabi-gcc -o "${BUILD_DIR}/cube_sat_nucleo_cm7.elf" "${OBJS[@]}" \
  -mcpu=cortex-m7 \
  -T"cube_sat_nucleo/CM7/STM32H755ZITX_FLASH.ld" \
  --specs=nosys.specs \
  -Wl,-Map="${BUILD_DIR}/cube_sat_nucleo_cm7.map" \
  -Wl,--gc-sections \
  -static \
  --specs=nano.specs \
  -mfpu=fpv5-d16 \
  -mfloat-abi=hard \
  -mthumb \
  -u _printf_float \
  -Wl,--start-group -lc -lm -Wl,--end-group

arm-none-eabi-size "${BUILD_DIR}/cube_sat_nucleo_cm7.elf"
printf 'CM7 firmware build: PASS %s\n' "${BUILD_DIR}/cube_sat_nucleo_cm7.elf"
CONTAINER_SCRIPT
