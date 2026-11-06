#Converts generated .hex into .bin for use in OTA update path
Import("env")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.hex",
    #Renames to "ski_fw.bin", convention for OTA release
    env.VerboseAction(
        '"$OBJCOPY" -I ihex -O binary "$BUILD_DIR/${PROGNAME}.hex" "$BUILD_DIR/ski_fw.bin"',
        "Building $BUILD_DIR/ski_fw.bin",
    ),
)