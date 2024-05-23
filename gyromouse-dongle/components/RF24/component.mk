# Component makefile for extras/bmp180

# expected anyone using bmp driver includes it as 'bmp180/bmp180.h'
INC_DIRS += $(RF24_ROOT)..

# args for passing into compile rule generation
RF24_SRC_DIR =  $(RF24_ROOT)

$(eval $(call component_compile_rules,RF24))
