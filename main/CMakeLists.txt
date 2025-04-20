idf_component_register(SRCS
  "app_main.c" "wifimanager.c" "camera.c" "recorder.cpp" "events.c" "playback.c"
  INCLUDE_DIRS "."
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
