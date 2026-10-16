# MRL_RAM=1(既定): コードと表を内部 RAM に置く(フラッシュのキャッシュ待ちを避ける)。MRL_RAM=0 でフラッシュのまま
set(C ${CMAKE_CURRENT_LIST_DIR}/../../..)
set(DEC ${C}/../decoder)
set(DEC_SRCS dsp.c bits.c tables.c decoder.c)
list(TRANSFORM DEC_SRCS PREPEND ${DEC}/src/)
if(NOT DEFINED MRL_RAM OR MRL_RAM STREQUAL "" OR MRL_RAM)
    set(LF mrl_ram.lf)
endif()
idf_component_register(
    SRCS "main.c" "${C}/tests/mcu/mcu_test.c" "${C}/src/encoder.c" ${DEC_SRCS}
    INCLUDE_DIRS "${C}/include" "${C}/tests/mcu" "${DEC}/include" "${DEC}/src"
    LDFRAGMENTS ${LF}
    EMBED_FILES "${C}/tests/vectors/speech.pcm" "${C}/tests/vectors/speech.expected.mrl")
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -ffp-contract=off)
target_compile_definitions(${COMPONENT_LIB} PRIVATE MRL_MAC16)   # ESP32(LX6)の MAC16 の 40 bit 累算器を使う
# エンコーダの探索だけ、積和を 1 命令(madd.s)に(デコーダの状態を進める dsp.c は原本とビット一致させるため off のまま)
if(NOT DEFINED MRL_ENC_FLAGS OR MRL_ENC_FLAGS STREQUAL "")
    set(MRL_ENC_FLAGS "-ffp-contract=fast")
endif()
separate_arguments(ENC_FLAGS UNIX_COMMAND "${MRL_ENC_FLAGS}")
set_source_files_properties(${C}/src/encoder.c PROPERTIES COMPILE_OPTIONS "${ENC_FLAGS}")
if(MRL_MICRO)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE MRL_MICRO)
endif()
if(MRL_PROFILE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE MRL_PROFILE)
endif()
