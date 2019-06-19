deps_config := \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/app_trace/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/aws_iot/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/bt/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/driver/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/esp32/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/esp_adc_cal/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/esp_event/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/esp_http_client/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/esp_http_server/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/ethernet/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/fatfs/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/freemodbus/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/freertos/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/heap/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/libsodium/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/log/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/lwip/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/mbedtls/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/mdns/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/mqtt/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/nvs_flash/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/openssl/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/pthread/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/spi_flash/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/spiffs/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/tcpip_adapter/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/vfs/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/wear_levelling/Kconfig \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/main/Kconfig.projbuild \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/chuhainam/Desktop/project/pas-project/pas-valve-fw/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)

ifneq "$(IDF_CMAKE)" "n"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
